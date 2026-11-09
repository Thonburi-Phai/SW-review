/**
 * @file battery_soc_soh.c
 * @brief LFP SOC/SOH estimation: coulomb counting with graded endpoint anchors.
 *
 * Pack SOC is the lowest cell's SOC, integrated from the BQ76907 PASSQ counter; OCV
 * lookup seeds it at init only, as the plateau is flat.  Graded anchors at the steep
 * knees set a target that SOC converges toward (protection events clamp, never raise),
 * per-cell offsets track the spread, and capacity is learned TI-Qmax style from pairs of
 * qualified rest points.
 *
 * Design: docs/soc_anchoring_redesign_plan.md
 */

#include "battery_soc_soh.h"
#include "wire_fault.h"
#include "bq_controller.h"
#include "bq_config_4s_lfp.h"
#include "../bq7690x/bq7690x_status.h"
#include "hal_uart.h"
#include "hal_systime.h"
#include "parameter/parameter.h"
#include <stdlib.h>
#include <string.h>
#include <limits.h>

/* abs() is UB for INT32_MIN — use safe version for all current_ma values */
static inline int32_t safe_abs32(int32_t x)
{
    if (x == INT32_MIN) return INT32_MAX;
    return x < 0 ? -x : x;
}
static inline float fabs_f(float x) { return x < 0 ? -x : x; }
static inline float clamp_f(float x, float lo, float hi) { return x < lo ? lo : (x > hi ? hi : x); }

/* ========== LFP OCV Lookup Table ========== */

/* Rested (open-circuit) cell voltage vs SOC, discharge branch, as explicit (mV, SOC %)
 * pairs so the steep bottom knee can be placed exactly. */
typedef struct { uint16_t mv; float soc; } ocv_point_t;
static const ocv_point_t lfp_ocv_table[] = {
    { 2500,   0.0f },
    { 2800,   1.0f },
    { 3000,   3.0f },
    { 3100,   6.0f },
    { 3150,  10.0f },
    { 3200,  15.0f },
    { 3220,  30.0f },
    { 3240,  35.0f },
    { 3260,  40.0f },
    { 3270,  45.0f },
    { 3280,  50.0f },   /* flat region */
    { 3290,  55.0f },
    { 3300,  60.0f },
    { 3310,  65.0f },
    { 3320,  70.0f },
    { 3340,  75.0f },
    { 3360,  80.0f },
    { 3400,  85.0f },
    { 3500,  90.0f },
    { 3600,  95.0f },
    { 3650, 100.0f },
};
#define OCV_TABLE_SIZE (sizeof(lfp_ocv_table) / sizeof(lfp_ocv_table[0]))

/* ========== Private State ========== */

static battery_config_t g_config;
static battery_status_t g_status = {0};

/* 64-bit software coulomb accumulator (uAh); survives PASSQ hardware resets.
 * Tracks the LOWEST cell's charge, the pack SOC reference. */
static int64_t s_accumulated_uah = 0;

/* Last reading from bq_controller for delta tracking */
static int32_t s_last_passed_charge_uah = 0;

/* PASSQ debug: rate-limited periodic print */
static uint32_t s_passq_dbg_last_ms = 0;
#define PASSQ_DBG_INTERVAL_MS 10000u
static uint32_t s_v2_dbg_last_ms = 0;
#define V2_DBG_INTERVAL_MS 60000u

/* Rest detection: low current with IR-corrected OCV, dV/dt qualified */
static struct {
    uint32_t start_ms;
    bool     active;        /* |I| <= C x REST_C_X100/100 */
    bool     qualified;     /* min time reached and (dV/dt settled or ceiling) */
    bool     anchor_done;   /* one anchor evaluation per rest period (re-armed hourly) */
    uint32_t anchor_ms;     /* when the last rest anchor evaluation ran */
} s_rest = {0};
#define REST_REEVAL_MS 3600000u

/* dV/dt: two consecutive 100 s window averages (x100 mV) of vmin and vmax OCV */
#define DVDT_WINDOW_MS 100000u
static struct {
    int32_t  sum_min, sum_max;
    uint16_t n;
    int32_t  prev_min_x100, prev_max_x100;
    bool     prev_valid;
    uint32_t win_start_ms;
    bool     ok;
} s_dvdt = {0};

/* Taper (T2) with grace period */
static struct {
    uint32_t accum_ms;
    uint32_t last_ok_ms;
    bool     active;
} s_taper = {0};
#define ANCHOR_GRACE_MS  3000u

/* Bottom anchor: armed flag for hard 0% re-arm after recovery */
static struct { bool armed; } s_bottom = { .armed = true };

/* Hard 0% anchor: 3s stability timer to reject crank transients */
static struct { uint32_t accum_ms; bool active; } s_hard_bottom = {0};
#define HARD_BOTTOM_STABLE_MS 3000u

/* Recent current history.  A protection trip is reported only after the chip has opened
 * the FET, by which time the current has collapsed, so the rate of a trip is judged from
 * the samples taken shortly before it rather than from the current at the trip. */
#define ICUR_RING 16
static struct {
    uint32_t ms[ICUR_RING];
    int32_t  ma[ICUR_RING];
    uint8_t  idx;
} s_icur = {0};

static void icur_push(int32_t ma, uint32_t now)
{
    s_icur.ms[s_icur.idx] = now ? now : 1u;      /* 0 marks an empty slot */
    s_icur.ma[s_icur.idx] = ma;
    s_icur.idx = (uint8_t)((s_icur.idx + 1) % ICUR_RING);
}

/* The current flowing when a cell reached its limit: the median of the newest
 * CFG_SOC_TERM_PRETRIP_SAMPLES samples within window_ms that carried at least min_ma.
 * A median rejects single spikes without reaching back past a charger's taper the way a
 * window peak would. */
static int32_t icur_recent(uint32_t now, uint32_t window_ms, int32_t min_ma)
{
    int32_t recent[CFG_SOC_TERM_PRETRIP_SAMPLES];
    uint32_t stamp[CFG_SOC_TERM_PRETRIP_SAMPLES];
    uint8_t n = 0;

    for (uint8_t k = 0; k < ICUR_RING; k++) {
        if (s_icur.ms[k] == 0u) continue;
        if ((now - s_icur.ms[k]) > window_ms) continue;
        int32_t a = safe_abs32(s_icur.ma[k]);
        if (a < min_ma) continue;
        if (n < CFG_SOC_TERM_PRETRIP_SAMPLES) {
            recent[n] = a; stamp[n] = s_icur.ms[k]; n++;
        } else {
            /* keep the newest CFG_SOC_TERM_PRETRIP_SAMPLES: replace the oldest held */
            uint8_t oldest = 0;
            for (uint8_t j = 1; j < n; j++) if (stamp[j] < stamp[oldest]) oldest = j;
            if (s_icur.ms[k] > stamp[oldest]) { recent[oldest] = a; stamp[oldest] = s_icur.ms[k]; }
        }
    }
    if (n == 0u) return 0;

    for (uint8_t a = 0; a + 1u < n; a++)                  /* small n, insertion is fine */
        for (uint8_t b = (uint8_t)(a + 1u); b < n; b++)
            if (recent[b] < recent[a]) {
                int32_t t = recent[a]; recent[a] = recent[b]; recent[b] = t;
            }
    return recent[n / 2u];
}

/* COV / CUV one-shots */
static bool s_cov_fired = false;

/* Set when boot happened under load, so open-circuit voltage could not be read.  The
 * provisional 50 % stands until the current falls away and a real reading is possible;
 * any anchor firing first clears it. */
static bool s_seed_pending = false;

/* Termination-anchor latches, one per end */
static struct {
    bool  top_anchored;    /* already anchored this charge; cleared once the pack is used */
    bool  bot_anchored;    /* already anchored this discharge; cleared once it is recharged */
    float bot_soc;         /* SOC when the bottom anchor fired, for the clearing test */
} s_term = {0};
static bool s_cuv_anchor_fired = false;
static float s_charge_complete_soc = 0.0f;

/* Soft correction state (bottom) */
static struct { uint32_t hold_ms; bool active; } s_soft_corr = {0};
/* Top soft correction hold */
static struct { uint32_t hold_ms; bool active; } s_top_soft_corr = {0};

/* Drain compensation remainder (microA*ms) */
static int64_t s_drain_remainder = 0;

/* Internal update timer */
static uint32_t s_last_update_ms = 0;

/* Cycle tracking */
static struct {
    bool in_charge_cycle;
    bool in_discharge_cycle;
    uint32_t cycle_charge_mah;
    uint32_t cycle_discharge_mah;
    int64_t charge_remainder;
    int64_t discharge_remainder;
} s_cycle_tracker = {0};

/* Raw coulomb counter — never touched by corrections, never reset by anchors.
 * Qualified points store its value; capacity = delta raw / delta SOC. */
static int64_t  s_raw_coulomb_uah = 0;
static int64_t  s_raw_drain_remainder = 0;
static uint32_t s_raw_epoch = 0;   /* bumped when counter continuity is lost */

/* Per-cell offsets (percent above the lowest cell) */
static float s_cell_offset_pct[SOC_NCELL] = {0};

/* Convergence engine.  It carries the correction still owed, in % SOC, not the SOC to
 * arrive at: a correction is a finite amount of charge to pay out, after which it is
 * done, leaving the coulomb count free to move SOC underneath it meanwhile. */
static struct {
    bool        active;
    float       owed;       /* signed % SOC still to apply */
    float       target;     /* what the anchor claimed, for reporting only */
    soc_grade_t grade;
    uint8_t     reason;
} s_conv = {0};

/* Capacity learning: qualified points */
typedef struct {
    bool     valid;
    int64_t  raw_uah;
    float    soc_pct;
    int16_t  temp_c;
    uint32_t sec;          /* systime_seconds(), which does not wrap in service */
    uint32_t epoch;
} soc_qpoint_t;
static soc_qpoint_t s_qp_pack = {0};
static soc_qpoint_t s_qp_cell[SOC_NCELL] = {{0}};
static uint8_t s_cell_learn_count[SOC_NCELL] = {0};

typedef enum {
    SOC_OWNER_NONE   = 0,
    SOC_OWNER_PULL   = 1,   /* continuous nudge from an IR-corrected voltage */
    SOC_OWNER_ANCHOR = 2,   /* discrete measurement with an error bound */
    SOC_OWNER_CLAMP  = 3,   /* safety: a cell is at its limit, the display must not lie */
} soc_owner_t;

static struct {
    soc_owner_t owner;
    float       target;
    uint8_t     grade;
    uint8_t     reason;
    bool        step_now;   /* apply at once rather than converge */
} s_prop = {0};

static void soc_propose(soc_owner_t owner, float target, uint8_t grade,
                        uint8_t reason, bool step_now)
{
    if (owner < s_prop.owner) return;                 /* weaker evidence, ignored */
    s_prop.owner = owner;  s_prop.target = target;  s_prop.grade = grade;
    s_prop.reason = reason; s_prop.step_now = step_now;
}


/* There is no runtime current-sign guard; a backwards shunt is covered by the production
 * self-test (checklist item 40).  `sign_fault` and SOC_FLAG2_SIGN_FAULT remain in the
 * status struct, always false, so the telemetry layout is unaffected. */

/* ========== Private Prototypes ========== */

static float voltage_to_soc(int16_t voltage_mv);
static int16_t soc_to_voltage(float soc_percent);
static void set_soc_direct(float soc_pct);
static void sync_acc_from_soc(void);
static void soc_set_target(float target_pct, soc_grade_t grade, uint8_t reason);
static void soc_anchor_target(float target_pct, soc_grade_t grade, uint8_t reason);
static void soc_converge_step(uint32_t dt_ms, bool low_current, int32_t current_ma);
static void learn_record_point(soc_qpoint_t *qp, float soc_pct, int16_t temp_c, uint32_t now, int cell);
static void soc_anchor_bottom(bool low_current, int16_t temp_c, uint32_t now);
static void update_cycle_tracking(int32_t current_ma, float soc_percent, uint32_t delta_time_ms);
static uint32_t calculate_internal_resistance_ocv(int16_t voltage_mv, int32_t current_ma, float soc_percent);
#if CFG_SOC_TEMP_COMPENSATION_EN
static uint32_t lfp_temp_capacity_factor(int16_t temp_c);
#endif

/* ========== Small helpers ========== */

static inline uint32_t cap_mah(void) { return g_status.soh.actual_capacity_mah; }
static inline int64_t  cap_uah(void) { return (int64_t)g_status.soh.actual_capacity_mah * 1000; }

static inline int32_t rest_current_ma(void)
{
    return (int32_t)((int64_t)cap_mah() * param_cache[PARAM_SOC_REST_C_X100].u8 / 100);
}

/* IR-corrected cell OCV estimate. current_ma > 0 = charge (terminal above OCV). */
static inline int16_t cell_ocv_est(int16_t v_mv, int32_t current_ma)
{
    int32_t r_x10 = param_cache[PARAM_SOC_CELL_R_CHG_X10MOHM].u16;
    int32_t drop_mv = (int32_t)(((int64_t)current_ma * r_x10) / 10000);
    return (int16_t)(v_mv - drop_mv);
}

static inline bool in_disq_band(int16_t ocv_mv)
{
    return ocv_mv >= (int16_t)param_cache[PARAM_SOC_DISQ_LO_MV].u16 &&
           ocv_mv <= (int16_t)param_cache[PARAM_SOC_DISQ_HI_MV].u16;
}

/* A learned capacity is eased in rather than applied in one step.  The accumulator is
 * carried across unchanged, so the charge held does not move and SOC stays free to
 * follow the anchor's convergence. */
static struct {
    bool     active;
    uint32_t target_mah;
    uint32_t span_mah;      /* travel at launch; the step is a fixed fraction of this */
} s_cap_ramp = {0};

static void cap_ramp_step(uint32_t dt_ms)
{
    if (!s_cap_ramp.active) return;
    uint32_t cur = g_status.soh.actual_capacity_mah;
    uint32_t tgt = s_cap_ramp.target_mah;
    if (cur == tgt) { s_cap_ramp.active = false; s_cap_ramp.span_mah = 0u; return; }

    /* Full travel in CFG_SOC_CAP_RAMP_S: the step is dt/ramp of the span at launch, not
     * of what is left, which would decay exponentially and never arrive. */
    uint32_t span = (cur > tgt) ? (cur - tgt) : (tgt - cur);
    if (s_cap_ramp.span_mah == 0u) s_cap_ramp.span_mah = span;
    uint32_t step = (uint32_t)((uint64_t)s_cap_ramp.span_mah * dt_ms
                               / (CFG_SOC_CAP_RAMP_S * 1000u));
    if (step == 0u) step = 1u;
    if (step > span) step = span;

    int64_t acc_before = s_accumulated_uah;
    g_status.soh.actual_capacity_mah = (cur > tgt) ? (cur - step) : (cur + step);
    if (g_status.soh.actual_capacity_mah == tgt) { s_cap_ramp.active = false; s_cap_ramp.span_mah = 0u; }

    /* carry the charge across unchanged, clamped to the new container */
    s_accumulated_uah = acc_before;
    if (s_accumulated_uah > cap_uah()) s_accumulated_uah = cap_uah();
    g_status.soh.usable_capacity_mah = cap_mah() - g_status.soh.imbalance_loss_mah;
}

static void sync_acc_from_soc(void)
{
    s_accumulated_uah = (int64_t)(g_status.soc.soc_percent / 100.0f * (float)cap_uah());
    g_status.soc.soc_coulomb_percent = g_status.soc.soc_percent;
    g_status.soc.coulomb_count_mah = (float)(s_accumulated_uah / 1000);
}

static void set_soc_direct(float soc_pct)
{
    g_status.soc.soc_percent = clamp_f(soc_pct, 0.0f, 100.0f);
    sync_acc_from_soc();
}

static void log_pct(const char *label, float v)
{
    int32_t x = (int32_t)(v * 10);
    if (x < 0) { hal_uart_printf("%s-%ld.%ld", label, (long)(-x / 10), (long)(-x % 10)); }
    else       { hal_uart_printf("%s%ld.%ld",  label, (long)(x / 10),  (long)(x % 10)); }
}

/* ========== Public Functions ========== */

int battery_soc_soh_init(const battery_config_t *config, float initial_soc_percent)
{
    if (config != NULL) {
        g_config = *config;
    } else {
        battery_config_t default_config = BATTERY_DEFAULT_CONFIG;
        g_config = default_config;
    }

    /* Override capacity from parameter system (runtime-settable) */
    g_config.nominal_capacity_mah = param_cache[PARAM_DESIGN_CAPACITY_MAH].u32;

    memset(&g_status, 0, sizeof(g_status));
    g_status.method = SOC_METHOD_COULOMB_COUNTING;
    g_status.initialized = true;

    g_status.soh.soh_percent = 100.0f;
    g_status.soh.actual_capacity_mah = g_config.nominal_capacity_mah;
    g_status.soh.nominal_capacity_mah = g_config.nominal_capacity_mah;
    g_status.soh.usable_capacity_mah = g_config.nominal_capacity_mah;
    g_status.soh.internal_resistance_mohm = 5;
    g_status.soh.initial_resistance_mohm = 5;
    g_status.soh.coulombic_efficiency = 99.5f;
    for (int i = 0; i < SOC_NCELL; i++) {
        g_status.soh.cell_capacity_mah[i] = g_config.nominal_capacity_mah;
        s_cell_offset_pct[i] = 0.0f;
        s_qp_cell[i].valid = false;
    }
    s_qp_pack.valid = false;
    s_raw_coulomb_uah = 0;
    s_raw_drain_remainder = 0;
    s_raw_epoch = 0;
    g_status.soh.raw_coulomb_active = true;
    memset(&s_conv, 0, sizeof(s_conv));
    memset(&s_rest, 0, sizeof(s_rest));
    memset(&s_dvdt, 0, sizeof(s_dvdt));
    s_cov_fired = false;
    s_prop.owner = SOC_OWNER_NONE;
    s_cap_ramp.active = false;
    s_cap_ramp.span_mah = 0u;
    memset(&s_icur, 0, sizeof(s_icur));
    s_seed_pending = false;
    s_term.top_anchored = false;
    s_term.bot_anchored = false;
    s_cuv_anchor_fired = false;

    if (initial_soc_percent >= 0 && initial_soc_percent <= 100) {
        set_soc_direct(initial_soc_percent);
    } else {
        /* Cold start: OCV lookup from the minimum cell voltage, or a 50 % default when
         * |current| > 5 A, where IR drop makes that lookup unusable. */
        const bq_battery_status_t *bq = bq_controller_get_status();
        int32_t boot_current = bq->current_ma;
        if (safe_abs32(boot_current) > 5000) {
            /* Terminal voltage is not open-circuit voltage here, so take 50 %
             * provisionally and mark the seed owed; the first quiet tick replaces it. */
            set_soc_direct(50.0f);
            s_seed_pending = true;
            hal_uart_printf("[SOC] Under load at boot (%ldmA): 50%% provisional, "
                            "will seed from OCV when the current settles\r\n",
                            (long)boot_current);
        } else {
            float ocv_soc = voltage_to_soc(cell_ocv_est(bq->cell_min_mv, boot_current));
            set_soc_direct(ocv_soc);
            {
                int32_t v = (int32_t)(ocv_soc * 10);
                hal_uart_printf("[SOC] OCV init: %ld.%ld%% (cell_min=%dmV)\r\n",
                                (long)(v / 10), (long)(v % 10), (int)bq->cell_min_mv);
            }
        }
    }

    /* Sync delta tracker and internal timer */
    s_last_passed_charge_uah = bq_controller_get_passed_charge_uah();
    s_last_update_ms = systime_milli();

    g_status.soc.soc_voltage_percent = g_status.soc.soc_percent;
    g_status.soc.remaining_capacity_mah =
        (int32_t)((g_status.soc.soc_percent / 100.0f) * g_config.nominal_capacity_mah);

    {
        int32_t v = (int32_t)(g_status.soc.soc_percent * 10);
        hal_uart_printf("[SOC] Init v2: %ld.%ld%% method=CoulombCounting cap=%lumAh learn=%d\r\n",
                        (long)(v / 10), (long)(v % 10),
                        (unsigned long)g_config.nominal_capacity_mah,
                        (int)param_cache[PARAM_SOC_LEARN_EN].u8);
    }

    return 0;
}

/* ---------- building blocks ---------- */

static void dvdt_reset(uint32_t now)
{
    s_dvdt.sum_min = s_dvdt.sum_max = 0;
    s_dvdt.n = 0;
    s_dvdt.prev_valid = false;
    s_dvdt.win_start_ms = now;
    s_dvdt.ok = false;
}

static void dvdt_accum(int16_t vmin_ocv, int16_t vmax_ocv, uint32_t now)
{
    s_dvdt.sum_min += vmin_ocv;
    s_dvdt.sum_max += vmax_ocv;
    s_dvdt.n++;
    if ((now - s_dvdt.win_start_ms) >= DVDT_WINDOW_MS && s_dvdt.n > 0) {
        int32_t avg_min = (s_dvdt.sum_min * 100) / (int32_t)s_dvdt.n;
        int32_t avg_max = (s_dvdt.sum_max * 100) / (int32_t)s_dvdt.n;
        if (s_dvdt.prev_valid) {
            /* Threshold in x10 µV/s, which over a 100 s window is x0.01 mV, matching the
             * x100 window averages.  It is relaxed 10x in the bottom knee and 5x in the
             * top one: the knees are that much steeper than the plateau, so the same
             * bound on OCV-lookup SOC error permits proportionally more drift there. */
            int32_t thr = param_cache[PARAM_SOC_REST_DVDT_X10UV_S].u8;
            if (vmin_ocv <= (int16_t)param_cache[PARAM_SOC_BOT_REST_MV].u16)      thr *= 10;
            else if (vmax_ocv >= (int16_t)param_cache[PARAM_SOC_TOP_REST_MV].u16) thr *= 5;
            int32_t dmin = avg_min - s_dvdt.prev_min_x100; if (dmin < 0) dmin = -dmin;
            int32_t dmax = avg_max - s_dvdt.prev_max_x100; if (dmax < 0) dmax = -dmax;
            s_dvdt.ok = (dmin <= thr) && (dmax <= thr);
        }
        s_dvdt.prev_min_x100 = avg_min;
        s_dvdt.prev_max_x100 = avg_max;
        s_dvdt.prev_valid = true;
        s_dvdt.sum_min = s_dvdt.sum_max = 0;
        s_dvdt.n = 0;
        s_dvdt.win_start_ms = now;
    }
}

static void rest_update(int32_t current_ma, int16_t vmin_ocv, int16_t vmax_ocv, uint32_t now)
{
    bool low = safe_abs32(current_ma) <= rest_current_ma();
    if (low) {
        if (!s_rest.active) {
            s_rest.active = true;
            s_rest.start_ms = now;
            s_rest.anchor_done = false;
            s_rest.qualified = false;
            dvdt_reset(now);
        }
        dvdt_accum(vmin_ocv, vmax_ocv, now);
        uint32_t rest_ms = now - s_rest.start_ms;
        uint32_t min_ms = (uint32_t)param_cache[PARAM_SOC_REST_MIN_MIN].u16 * 60000u;
        uint32_t max_ms = (uint32_t)param_cache[PARAM_SOC_REST_MAX_MIN].u16 * 60000u;
        s_rest.qualified = (rest_ms >= min_ms) && (s_dvdt.ok || rest_ms >= max_ms);
    } else {
        s_rest.active = false;
        s_rest.qualified = false;
        s_dvdt.ok = false;
    }
}

/* ---------- who may move the charge count ----------
 *
 * charge = SOC x capacity: charge is measured, capacity is believed, SOC is derived.
 * Ownership is explicit:
 *   - the coulomb integrator owns the charge count; only measured charge changes it
 *   - the learner owns capacity; nothing else writes it but an explicit operator override
 *   - SOC is derived and is never written directly
 *
 * Everything that wants to move SOC submits a proposal here with a priority, and one
 * arbiter applies the winner.  Priority is evidence quality, not execution order. */
/* ---------- convergence engine ---------- */

/* Anchors call this.  It writes nothing itself: it submits at ANCHOR priority, so a clamp
 * can override it and the pull cannot. */
static void soc_anchor_target(float target_pct, soc_grade_t grade, uint8_t reason)
{
    soc_propose(SOC_OWNER_ANCHOR, target_pct, (uint8_t)grade, reason, false);
}

static void soc_set_target(float target_pct, soc_grade_t grade, uint8_t reason)
{
    target_pct = clamp_f(target_pct, 0.0f, 100.0f);
    float err = target_pct - g_status.soc.soc_percent;
    s_conv.active = true;
    s_conv.owed   = err;
    s_conv.target = target_pct;
    s_conv.grade  = grade;
    s_conv.reason = reason;
    g_status.soc.drift_at_anchor_pct = err;
    g_status.soc.last_anchor_event = reason;
    g_status.soc.anchor_grade = (uint8_t)grade;
    g_status.soc.is_calibrated = true;
    hal_uart_printf("[SOC] anchor ev=%d grade=%d target=", (int)reason, (int)grade);
    log_pct("", target_pct); log_pct(" drift=", err); hal_uart_printf("\r\n");
}

static void soc_converge_step(uint32_t dt_ms, bool low_current, int32_t current_ma)
{
    if (!s_conv.active) return;
    float err = s_conv.owed;
    if (fabs_f(err) < 0.05f) {
        set_soc_direct(g_status.soc.soc_percent + err);
        s_conv.active = false; s_conv.owed = 0.0f;
        return;
    }
    float rate_pct_per_min;
    if (low_current) {
        rate_pct_per_min = (float)param_cache[PARAM_SOC_CONV_REST_PCT_PER_MIN].u8;
    } else {
        float lo = (float)param_cache[PARAM_SOC_CONV_LOAD_MIN_X10_PCT_PER_MIN].u8 / 10.0f;
        float hi = (float)param_cache[PARAM_SOC_CONV_LOAD_MAX_PCT_PER_MIN].u8;
        uint32_t t_min = (current_ma < 0) ? g_status.soc.time_to_empty_min
                                          : g_status.soc.time_to_full_min;
        if (t_min == 0) rate_pct_per_min = lo;
        else            rate_pct_per_min = fabs_f(err) / (float)t_min;
        rate_pct_per_min = clamp_f(rate_pct_per_min, lo, hi);
    }
    float step = rate_pct_per_min * (float)dt_ms / 60000.0f;
    if (step > fabs_f(err)) step = fabs_f(err);
    float delta = (err > 0.0f) ? step : -step;
    s_conv.owed -= delta;
    set_soc_direct(g_status.soc.soc_percent + delta);
}

/* Apply the winning proposal: the only place a correction reaches the charge count.
 * A clamp steps at once, since a cell is already at its limit; an anchor and the pull
 * both converge instead, so nothing jumps. */
static void soc_arbitrate(uint32_t dt_ms, bool low_current, int32_t current_ma)
{
    if (s_prop.owner != SOC_OWNER_NONE) {
        if (s_prop.step_now) {
            set_soc_direct(s_prop.target);
            s_conv.active = false; s_conv.owed = 0.0f;
        } else {
            soc_set_target(s_prop.target, s_prop.grade, s_prop.reason);
        }
        g_status.soc.last_anchor_event = s_prop.reason;
        g_status.soc.anchor_grade      = s_prop.grade;
        s_prop.owner = SOC_OWNER_NONE;
    }
    soc_converge_step(dt_ms, low_current, current_ma);
}

/* ---------- capacity learning (TI Qmax method) ---------- */

static void learn_try(soc_qpoint_t *qp, float soc_now, int16_t temp_now, int cell)
{
    (void)temp_now;   /* recorded on the point, no longer a condition */
    uint8_t reason = SOC_LEARN_OK;
    uint32_t nominal = g_config.nominal_capacity_mah;
    if (!param_cache[PARAM_SOC_LEARN_EN].u8)      reason = SOC_LEARN_DISABLED;
    else if (!qp->valid)                           reason = SOC_LEARN_NO_PRIOR;
    else if (qp->epoch != s_raw_epoch)             reason = SOC_LEARN_EPOCH;
    else {
        float dsoc = fabs_f(soc_now - qp->soc_pct);
        /* Age of the stored point, which decides whether the charge counted between the
         * two ends can be trusted; the limit is CFG_SOC_LEARN_MAX_AGE_H.  In seconds, not
         * milliseconds: the millisecond counter aliases after 49.7 days and would make an
         * ancient point look fresh.  Temperature is recorded but is not a condition. */
        uint32_t age_s   = systime_seconds() - qp->sec;
        uint32_t age_max = (uint32_t)param_cache[PARAM_SOC_LEARN_MAX_AGE_H].u16 * 3600u;
        if (dsoc < (float)param_cache[PARAM_SOC_LEARN_MIN_DSOC_PCT].u8) reason = SOC_LEARN_DSOC;
        else if (age_s > age_max)                                       reason = SOC_LEARN_AGE;
        else {
            int64_t draw = s_raw_coulomb_uah - qp->raw_uah;
            if (draw < 0) draw = -draw;
            float q_mah = (float)(draw / 1000) / (dsoc / 100.0f);

            uint32_t *slot = (cell < 0) ? &g_status.soh.actual_capacity_mah
                                        : &g_status.soh.cell_capacity_mah[cell];
            uint32_t old = *slot;
            /* Both the sanity window and the per-update step are measured against what
             * the gauge has LEARNED, not against the nameplate, so the gauge can follow a
             * pack all the way down; the nameplate is only the starting value and the
             * reference SOH is reported against.  A percentage step therefore shrinks in
             * absolute terms as capacity falls. */
            float ref = (old > 0u) ? (float)old : (float)nominal;
            if (q_mah < ref * 0.5f || q_mah > ref * 1.5f) {
                reason = SOC_LEARN_SANITY;
            } else {
                uint8_t prior = (cell < 0) ? g_status.soh.learn_count : s_cell_learn_count[cell];
                float blended;
                if (prior == 0) {
                    /* First qualified measurement: blend it with the nameplate by
                     * PARAM_SOC_LEARN_FIRST_W_PCT, with no step bound. */
                    float w = (float)param_cache[PARAM_SOC_LEARN_FIRST_W_PCT].u8 / 100.0f;
                    blended = (float)old * (1.0f - w) + q_mah * w;
                } else {
                    blended = ((float)old * 7.0f + q_mah * 3.0f) / 10.0f;
                    /* Step allowance 1 + 3/(prior+1) times the configured percentage:
                     * wide at first, tightening as measurements accumulate. */
                    float conf = 1.0f + 3.0f / (float)(prior + 1u);
                    float step_dn = ref * param_cache[PARAM_SOC_LEARN_STEP_DOWN_PCT].u8 / 100.0f * conf;
                    float step_up = ref * param_cache[PARAM_SOC_LEARN_STEP_UP_PCT].u8 / 100.0f * conf;
                    if (blended < (float)old - step_dn) blended = (float)old - step_dn;
                    if (blended > (float)old + step_up) blended = (float)old + step_up;
                }
                if (cell >= 0) *slot = (uint32_t)blended;     /* per-cell applies at once */
                if (cell >= 0 && s_cell_learn_count[cell] < 255) s_cell_learn_count[cell]++;
                if (cell < 0) {
                    g_status.soh.soh_percent = blended / (float)nominal * 100.0f;
                    g_status.soh.capacity_fade_percent =
                        ((float)nominal - blended) / (float)nominal * 100.0f;
                    g_status.soh.last_learned_capacity_mah = (uint32_t)q_mah;
                    g_status.soh.last_learn_ms = systime_milli();
                    if (g_status.soh.learn_count < 255) g_status.soh.learn_count++;
                    /* The learner owns capacity only; it does NOT touch the charge count,
                     * since revising the container cannot change the coulombs counted
                     * into it.  Capacity is eased in rather than stepped because SOC is
                     * charge over capacity, so a capacity step would step SOC. */
                    s_cap_ramp.target_mah = (uint32_t)blended;
                    s_cap_ramp.span_mah = 0u;   /* captured on the first step */
                    s_cap_ramp.active = true;
                }
                hal_uart_printf("[SOH] learn %s%d: measured=%lu mAh over ", cell < 0 ? "pack" : "cell", cell < 0 ? 0 : cell + 1,
                                (unsigned long)q_mah);
                log_pct("", dsoc);
                hal_uart_printf("%% -> %lu (was %lu)%s\r\n", (unsigned long)blended,
                                (unsigned long)old, cell < 0 ? ", easing in" : "");
            }
        }
    }
    if (cell < 0) g_status.soh.learn_refused_reason = reason;
    if (reason != SOC_LEARN_OK && cell < 0) {
        if (reason == SOC_LEARN_AGE)
            hal_uart_printf("[SOH] learn refused: stored point is %lu h old, limit %u h\r\n",
                            (unsigned long)((systime_seconds() - qp->sec) / 3600u),
                            (unsigned)param_cache[PARAM_SOC_LEARN_MAX_AGE_H].u16);
        else
            hal_uart_printf("[SOH] learn refused: reason=%d\r\n", (int)reason);
    }
}

static void learn_record_point(soc_qpoint_t *qp, float soc_pct, int16_t temp_c, uint32_t now, int cell)
{
    (void)now;        /* the point stamps itself in seconds */
    learn_try(qp, soc_pct, temp_c, cell);
    qp->valid   = true;
    qp->raw_uah = s_raw_coulomb_uah;
    qp->soc_pct = soc_pct;
    qp->temp_c  = temp_c;
    qp->sec     = systime_seconds();
    qp->epoch   = s_raw_epoch;
}

/* Decide whether the pull may act while a convergence is still running.  delta is the
 * direction the pull wants to go: if it agrees with what the convergence still owes, the
 * pull yields rather than double-counting the anchor; if it disagrees, the convergence is
 * retired, since a cell in a knee is live evidence while the anchor's target describes
 * the pack at the moment it fired. */
static bool pull_vs_conv(float delta)
{
    if (!s_conv.active || s_conv.grade < SOC_GRADE_MED) return true;
    if ((delta > 0.0f) == (s_conv.owed > 0.0f)) return false;   /* same way: yield */
    hal_uart_printf("[SOC] pull retires a stale convergence ev=%d, owed=",
                    (int)s_conv.reason);
    log_pct("", s_conv.owed); hal_uart_printf("%% contradicted by the cell voltage\r\n");
    s_conv.active = false; s_conv.owed = 0.0f;
    return true;
}

/* ---------- endpoint pull ----------
 *
 * One correction law in two zones, at both ends of the curve.
 *
 * Outer zone: remove err * (dq / q_tau) of the error, a fraction per amp-hour moved
 * rather than per minute, so the correction scales with C-rate and stops at zero current
 * where the voltage is still relaxing.  Gated on current direction.
 *
 * Inner zone, the last stretch before the cell's limit: fixed
 * PARAM_SOC_PULL_HARD_PCT_PER_MIN rate, gated on current direction but not on
 * current flowing. */
static void soc_pull_update(int16_t vmin_ocv, int16_t vmax_ocv, int32_t current_ma,
                            uint32_t dt_ms, bool allowed)
{
    if (!allowed) return;

    /* The pull submits at SOC_OWNER_PULL, so the arbiter discards it whenever an anchor
     * or a clamp has also spoken this tick; whether it yields to a running convergence is
     * decided per proposal by pull_vs_conv(). */

    int16_t bot_entry = (int16_t)param_cache[PARAM_SOC_BOT_PULL_ENTRY_MV].u16;
    int16_t bot_hard  = (int16_t)param_cache[PARAM_SOC_BOT_HARD_MV].u16;
    int16_t top_entry = (int16_t)param_cache[PARAM_SOC_TOP_PULL_ENTRY_MV].u16;
    int16_t top_hard  = (int16_t)param_cache[PARAM_SOC_TOP_HARD_MV].u16;
    float   soc       = g_status.soc.soc_percent;

    /* ---- inner zones: fixed rate, direction-gated ---- */
    float hard_step = (float)param_cache[PARAM_SOC_PULL_HARD_PCT_PER_MIN].u8
                      * (float)dt_ms / 60000.0f;
    /* Both inner zones are gated on current direction: the IR correction over-compensates,
     * making a cell look further toward the end the current is NOT taking it.  So pull
     * down only when not charging, up only when not discharging. */
    if (vmin_ocv <= bot_hard && current_ma < (int32_t)CFG_SOC_SOFT_CORR_MIN_MA) {
        float target = voltage_to_soc(vmin_ocv);
        if (soc - target > 0.05f && pull_vs_conv(-1.0f)) {
            float step = (soc - target) < hard_step ? (soc - target) : hard_step;
            soc_propose(SOC_OWNER_PULL, soc - step, SOC_GRADE_MED,
                        SOC_ANCHOR_BOT_SOFT_CORR, true);
        }
        return;
    }
    /* Top inner zone: pulls up only when not discharging, since under heavy discharge the
     * IR correction can add enough to fake a full cell. */
    if (vmax_ocv >= top_hard && current_ma > -(int32_t)CFG_SOC_SOFT_CORR_MIN_MA) {
        float target = voltage_to_soc(vmax_ocv);
        if (target - soc > 0.05f && pull_vs_conv(1.0f)) {
            float step = (target - soc) < hard_step ? (target - soc) : hard_step;
            soc_propose(SOC_OWNER_PULL, soc + step, SOC_GRADE_MED,
                        SOC_ANCHOR_TOP_SOFT_CORR, true);
        }
        return;
    }

    /* ---- outer zones: scaled by charge moved, gated on direction ---- */
    int32_t i_abs = safe_abs32(current_ma);
    if (i_abs <= (int32_t)CFG_SOC_SOFT_CORR_MIN_MA) return;      /* no current, no pull */

    float dq_mah = (float)i_abs * (float)dt_ms / 3600000.0f;
    /* time constant as a fraction of capacity, not a fixed amp-hour figure */
    float q_tau  = (float)cap_mah() * (float)param_cache[PARAM_SOC_PULL_Q_TAU_PCT].u8 / 100.0f;
    if (q_tau < 1.0f) return;
    float frac = dq_mah / q_tau;
    if (frac > 0.25f) frac = 0.25f;                              /* never more than a quarter at once */

    if (current_ma < 0 && vmin_ocv < bot_entry) {
        float target = voltage_to_soc(vmin_ocv);
        float err    = soc - target;
        if (err > 0.0f && pull_vs_conv(-1.0f)) {
            soc_propose(SOC_OWNER_PULL, soc - err * frac, SOC_GRADE_MED,
                        SOC_ANCHOR_BOT_SOFT_CORR, true);
        }
    } else if (current_ma > 0 && vmax_ocv > top_entry) {
        float target = voltage_to_soc(vmax_ocv);
        float err    = target - soc;
        if (err > 0.0f && pull_vs_conv(1.0f)) {
            soc_propose(SOC_OWNER_PULL, soc + err * frac, SOC_GRADE_MED,
                        SOC_ANCHOR_TOP_SOFT_CORR, true);
        }
    }
}

/* ---------- T4 / B4: charge or discharge termination as an endpoint anchor ---------- */

/* Called when a charge or discharge ends because a cell reached its limit.
 *
 * A termination means "full" (or "empty") only if the cell was at its knee rather than
 * pushed there by the rate, so the gap is bounded directly: it is the voltage the current
 * holds the cell away from its rested value, over how steeply voltage moves with charge
 * there,
 *
 *     error in % SOC = (|I| * R_cell) / (dV/dSOC at the terminating voltage)
 *
 * with R the polarization-inclusive figure and the slope read from the OCV table at the
 * terminating voltage, so the test tightens in the flat region and relaxes at the knees.
 * Terminations above PARAM_SOC_TERM_C_X100, or whose bound exceeds
 * PARAM_SOC_TERM_ERR_X10_PCT, are refused. */
static void term_anchor_check(bool top, int32_t current_ma, int16_t v_term_mv,
                              float soc_at_term, int16_t learn_temp, uint32_t now_ms)
{
    if (top ? s_term.top_anchored : s_term.bot_anchored) return;

    /* The current that caused the trip, not the one left after the FET opened: look back
     * over CFG_SOC_TERM_PRETRIP_MS, and take whichever is larger of that and the current
     * reading. */
    int32_t i_abs = icur_recent(now_ms, CFG_SOC_TERM_PRETRIP_MS,
                                (int32_t)CFG_SOC_SOFT_CORR_MIN_MA);
    if (i_abs < safe_abs32(current_ma)) i_abs = safe_abs32(current_ma);
    int32_t i_lim = (int32_t)((int64_t)cap_mah() * param_cache[PARAM_SOC_TERM_C_X100].u8 / 100);
    if (i_abs > i_lim) {
        hal_uart_printf("[SOC] %s termination: %ld mA before the trip exceeds the rate "
                        "ceiling %ld mA, no anchor\r\n",
                        top ? "charge" : "discharge", (long)i_abs, (long)i_lim);
        return;
    }

    /* How far the current is holding this cell from its rested value, in mV.  Uses the
     * measured short-timescale resistance, not the OCV-estimation one, with a separate
     * discharge-side figure. */
    int32_t r_x10   = top ? (int32_t)param_cache[PARAM_SOC_TERM_R_X10MOHM].u16
                          : (int32_t)param_cache[PARAM_SOC_TERM_R_DSG_X10MOHM].u16;
    int32_t err_mv  = (int32_t)(((int64_t)i_abs * r_x10) / 10000);

    /* local steepness of the curve, in points per 30 mV, where the cell actually is */
    float pts_per_30mv = voltage_to_soc((int16_t)(v_term_mv + 15)) -
                         voltage_to_soc((int16_t)(v_term_mv - 15));
    if (pts_per_30mv < 0) pts_per_30mv = -pts_per_30mv;

    float err_pct = (float)err_mv * pts_per_30mv / 30.0f;
    float tol     = (float)param_cache[PARAM_SOC_TERM_ERR_X10_PCT].u8 / 10.0f;

    if (err_pct > tol) {
        hal_uart_printf("[SOC] %s termination at %ld mA before the trip is worth ",
                        top ? "charge" : "discharge", (long)i_abs);
        log_pct("", err_pct);
        hal_uart_printf("%% SOC, past the limit: no anchor\r\n");
        return;
    }

    /* Anchor at the corrected endpoint rather than a flat 100 or 0: a termination under
     * load always happens early, so the computed shortfall is taken off the endpoint and
     * the result is clamped on the 50 % side.  The discharge side is the weaker of the
     * two, its resistance near empty being unmeasured. */
    if (top) {
        soc_at_term -= err_pct;
        if (soc_at_term < 50.0f) soc_at_term = 50.0f;
    } else {
        soc_at_term += err_pct;
        if (soc_at_term > 50.0f) soc_at_term = 50.0f;
    }

    uint32_t old_cap = cap_mah();
    learn_record_point(&s_qp_pack, soc_at_term, learn_temp, now_ms, -1);
    if (top) {
        soc_anchor_target(soc_at_term, SOC_GRADE_MED, SOC_ANCHOR_TOP_TERM);
        s_term.top_anchored = true;
    } else {
        /* The undervoltage clamp has already put the display on its reserve, so only the
         * telemetry fields are set here; the learning point above is recorded at the
         * physical 0 %, which is what the capacity arithmetic needs. */
        g_status.soc.last_anchor_event = SOC_ANCHOR_BOT_TERM;
        g_status.soc.anchor_grade      = SOC_GRADE_MED;
        s_term.bot_anchored = true;
        s_term.bot_soc      = g_status.soc.soc_percent;
    }
    hal_uart_printf("[SOC] %s termination at %ld mA before the trip, error bound ",
                    top ? "charge" : "discharge", (long)i_abs);
    log_pct("", err_pct);
    hal_uart_printf("%%: anchored at ");
    log_pct("", soc_at_term);
    hal_uart_printf("%%, capacity %lu -> %lu mAh\r\n",
                    (unsigned long)old_cap, (unsigned long)cap_mah());
}

/* ---------- main update ---------- */

int battery_soc_soh_update(int16_t voltage_mv, int32_t current_ma,
                           int16_t temp_c, uint32_t delta_time_ms)
{
    if (!g_status.initialized) return -1;

    (void)delta_time_ms;

    /* Internal dt: don't trust caller */
    uint32_t now_ms = systime_milli();
    uint32_t dt_ms = (s_last_update_ms == 0) ? 1000 : (now_ms - s_last_update_ms);
    s_last_update_ms = now_ms;
    if (dt_ms == 0) dt_ms = 1;
    if (dt_ms > 60000) dt_ms = 60000;

    const bq_battery_status_t *bq = bq_controller_get_status();
    int16_t learn_temp = (temp_c != -999) ? temp_c : bq->internal_temp_c;

    /* Ease a learned capacity in before anything reads it, and ahead of the SOC
     * derivation below: a correction that writes the accumulator back from SOC must use
     * the same capacity SOC was derived from. */
    cap_ramp_step(dt_ms);

    /* Capacity and temperature derating */
    int64_t cap = cap_uah();
#if CFG_SOC_TEMP_COMPENSATION_EN
    uint32_t tfactor = lfp_temp_capacity_factor(temp_c);
#else
    (void)temp_c;
    uint32_t tfactor = 1000;
#endif
    int64_t eff_cap_uah = (cap * (int64_t)tfactor) / 1000;

    /* === Coulomb Counting: Delta-based PASSQ tracking === */
    {
        int32_t passq_raw = bq->passq_raw;
        int32_t current_uah = bq_controller_get_passed_charge_uah();
        int32_t delta = current_uah - s_last_passed_charge_uah;

        /* Max plausible delta at configured C-rate for this tick */
        int32_t max_delta_uah = (int32_t)((int64_t)cap_mah() *
                        param_cache[PARAM_SOC_JUMP_DETECT_C_X10].u8 * dt_ms / 36000);

        if (delta < -100000) {
            /* PASSQ hardware reset: counter was zeroed */
            s_last_passed_charge_uah = current_uah;
        } else if (max_delta_uah > 0 && safe_abs32(delta) > max_delta_uah) {
            s_last_passed_charge_uah = current_uah;
            s_raw_epoch++;
            if (g_status.soc.passq_jump_count < 255)
                g_status.soc.passq_jump_count++;
            hal_uart_printf("[SOC] PASSQ jump: delta=%ld max=%ld uah\r\n",
                            (long)delta, (long)max_delta_uah);
        } else {
            s_accumulated_uah += (int64_t)delta;
            s_raw_coulomb_uah += (int64_t)delta;
            s_last_passed_charge_uah = current_uah;
        }

        /* Subtract invisible MCU drain with remainder accumulator */
        if (CFG_SOC_INVISIBLE_DRAIN_UA > 0) {
            int64_t num = (int64_t)CFG_SOC_INVISIBLE_DRAIN_UA * (int64_t)dt_ms + s_drain_remainder;
            int64_t drain = num / 3600000LL;
            s_drain_remainder = num - (drain * 3600000LL);
            s_accumulated_uah -= drain;
            int64_t rnum = (int64_t)CFG_SOC_INVISIBLE_DRAIN_UA * (int64_t)dt_ms + s_raw_drain_remainder;
            int64_t rdrain = rnum / 3600000LL;
            s_raw_drain_remainder = rnum - (rdrain * 3600000LL);
            s_raw_coulomb_uah -= rdrain;
        }

        /* Clamp accumulator to valid range */
        if (s_accumulated_uah < 0) s_accumulated_uah = 0;
        if (s_accumulated_uah > cap) s_accumulated_uah = cap;

        /* SOC from accumulator with temperature derating */
        int64_t inaccessible_uah = cap - eff_cap_uah;
        int64_t eff_acc = s_accumulated_uah - inaccessible_uah;
        if (eff_cap_uah > 0)
            g_status.soc.soc_percent = clamp_f((float)eff_acc / (float)eff_cap_uah * 100.0f, 0.0f, 100.0f);
        g_status.soc.soc_coulomb_percent = g_status.soc.soc_percent;
        g_status.soc.coulomb_count_mah = (float)(s_accumulated_uah / 1000);
        g_status.soh.raw_coulomb_mah = (int32_t)(s_raw_coulomb_uah / 1000);
        g_status.soh.raw_coulomb_active = true;

        /* PASSQ overflow prevention */
        if (s_last_passed_charge_uah > 1000000000 || s_last_passed_charge_uah < -1000000000) {
            if (bq_controller_reset_passq() == 0) s_last_passed_charge_uah = 0;
        }

        if ((now_ms - s_passq_dbg_last_ms) >= PASSQ_DBG_INTERVAL_MS) {
            s_passq_dbg_last_ms = now_ms;
            /* up= is the MCU clock, el= the BQ76907's; the two should advance together,
             * and a drift between them is the clock fault of checklist item 41. */
            hal_uart_printf("[PASSQ] raw=%ld uah=%ld delta=%ld acc=%ldmAh el=%lus up=%lus\r\n",
                            (long)passq_raw, (long)current_uah, (long)delta,
                            (long)(s_accumulated_uah / 1000),
                            (unsigned long)bq->passq_elapsed_time_sec,
                            (unsigned long)systime_seconds());
        }
    }

    /* Voltage SOC for debug display only (never feeds actual SOC) */
    g_status.soc.soc_voltage_percent = voltage_to_soc(voltage_mv);

    /* === Per-cell IR-corrected OCV estimates === */
    int16_t v_ocv[SOC_NCELL];
    int16_t vmin_ocv = 32767, vmax_ocv = -32768;
    uint8_t vmax_idx = 0;
    bool cells_sane = true;
    for (int i = 0; i < SOC_NCELL; i++) {
        int16_t raw = bq->cell_voltage_mv[i];
        if (raw < 1500 || raw > 4000) cells_sane = false;   /* open-wire / mirrored node */
        v_ocv[i] = cell_ocv_est(raw, current_ma);
        g_status.soc.v_ocv_mv[i] = v_ocv[i];
        if (v_ocv[i] < vmin_ocv) { vmin_ocv = v_ocv[i]; }
        if (v_ocv[i] > vmax_ocv) { vmax_ocv = v_ocv[i]; vmax_idx = (uint8_t)i; }
    }
    int16_t spread_ocv = (int16_t)(vmax_ocv - vmin_ocv);
    int32_t rest_ma = rest_current_ma();
    icur_push(current_ma, now_ms);
    bool low_current = safe_abs32(current_ma) <= rest_ma;

    /* Redeem a seed that boot could not take because current was flowing: the same lookup
     * the cold-start path would have done, still only a plateau-grade estimate that any
     * later anchor overrides. */
    if (s_seed_pending) {
        if (g_status.soc.last_anchor_event != SOC_ANCHOR_NONE) {
            s_seed_pending = false;            /* an anchor already did better */
        } else if (low_current && cells_sane) {
            float ocv_soc = voltage_to_soc(vmin_ocv);
            set_soc_direct(ocv_soc);
            s_seed_pending = false;
            hal_uart_printf("[SOC] seeded from OCV once settled: ");
            log_pct("", ocv_soc);
            hal_uart_printf("%% (cell_min %d mV)\r\n", (int)vmin_ocv);
        }
    }

    /* === Rest detection (low current, dV/dt qualified) === */
    rest_update(current_ma, vmin_ocv, vmax_ocv, now_ms);
    bool anchors_allowed = cells_sane;

    /* === Bottom anchors, downward only === */
    {
        /* IR-compensated lowest cell, using the real cell resistance; crank transients are
         * rejected by the hold time rather than by an inflated R. */
        uint16_t v_compensated = (uint16_t)vmin_ocv;
        if (vmin_ocv < 0) v_compensated = 0;
        bool in_steep = (bq->cell_min_mv > 0) && (v_compensated < CFG_SOC_STEEP_ENTRY_MV);
        bool is_discharging = (current_ma < -(int32_t)CFG_SOC_SOFT_CORR_MIN_MA);

        if (!s_bottom.armed && bq->cell_min_mv > CFG_SOC_BOTTOM_ANCHOR_RECOVER_MV)
            s_bottom.armed = true;

        /* Hard 0 %: raw cell_min below CFG_SOC_HARD_BOTTOM_MV, held HARD_BOTTOM_STABLE_MS */
        if (bq->cell_min_mv > 0 && bq->cell_min_mv < CFG_SOC_HARD_BOTTOM_MV && s_bottom.armed && anchors_allowed) {
            if (!s_hard_bottom.active) { s_hard_bottom.active = true; s_hard_bottom.accum_ms = 0; }
            s_hard_bottom.accum_ms += dt_ms;
            if (s_hard_bottom.accum_ms >= HARD_BOTTOM_STABLE_MS) {
                soc_anchor_bottom(low_current, learn_temp, now_ms);
                s_bottom.armed = false;
                s_hard_bottom.active = false;
            }
        } else {
            s_hard_bottom.active = false;
            s_hard_bottom.accum_ms = 0;
        }

        /* Correction in this region is done by soc_pull_update() below, for both ends. */
        (void)in_steep; (void)is_discharging;

        /* CUV fault -> reserve (downward only), LOW grade, no learning.  Suppressed on a
         * wire fault, where a phantom CUV from an open tap must not anchor SOC. */
        if ((bq->safety_status_a & BQ_SAFETY_STATUS_A_CUV) && !s_cuv_anchor_fired
            && !wire_fault_soc_anchors_suppressed()) {
            uint8_t reserve_x10 = param_cache[PARAM_SOC_CUV_RESERVE_X10].u8;
            float reserve_pct = (float)reserve_x10 / 10.0f;
            if (g_status.soc.soc_percent > reserve_pct) {
                hal_uart_printf("[SOC] CUV clamp -> "); log_pct("", reserve_pct); hal_uart_printf("%%\r\n");
                soc_propose(SOC_OWNER_CLAMP, reserve_pct, SOC_GRADE_LOW,
                            SOC_ANCHOR_BOT_CUV, true);
            }
            g_status.soc.last_anchor_event = SOC_ANCHOR_BOT_CUV;
            g_status.soc.anchor_grade = SOC_GRADE_LOW;
            s_cuv_anchor_fired = true;
            s_bottom.armed = false;
            /* B4, the mirror of the charge-side anchor: the same computed error bound
             * decides whether this discharge termination means the pack is empty. */
            term_anchor_check(false, current_ma, vmin_ocv, 0.0f, learn_temp, now_ms);
        }
        if (!(bq->safety_status_a & BQ_SAFETY_STATUS_A_CUV)) s_cuv_anchor_fired = false;

        g_status.soc.anchor_flags = 0;
        if (in_steep && is_discharging) g_status.soc.anchor_flags |= SOC_FLAG_IN_STEEP;
    }

    /* Endpoint pull, both ends.  Runs after the bottom safety clamps so a hard limit
     * wins, and before the termination anchors so a qualified termination overrides it. */
    soc_pull_update(vmin_ocv, vmax_ocv, current_ma, dt_ms, anchors_allowed);

    /* === T3: COV event -> ceiling only (LOW) === */
    {
        /* Also suppressed on a wire fault, where a mirrored cell gives a phantom COV. */
        bool cov = ((bq->safety_status_a & BQ_SAFETY_STATUS_A_COV) != 0)
                   && !wire_fault_soc_anchors_suppressed();
        if (cov && !s_cov_fired && anchors_allowed) {
            s_cov_fired = true;
            g_status.soc.charge_complete = true;
            s_charge_complete_soc = g_status.soc.soc_percent;
            /* Deliberately does NOT cancel a running convergence: an anchor converging
             * toward a measured target may still raise SOC, and upward drift is bounded
             * by the 100 % clamp and by the pull standing aside while an anchor runs. */
            g_status.soc.last_anchor_event = SOC_ANCHOR_TOP_COV;
            g_status.soc.anchor_grade = SOC_GRADE_LOW;
            /* COV does NOT mean the tripping cell is full: under load a cell can reach
             * its knee first and relax back into the plateau band afterwards.  So no
             * cell is set to 100 % and the offsets are left alone, consistent with this
             * anchor's LOW grade; offsets come from the rest anchor, which reads them
             * from settled OCV outside the band. */
            hal_uart_printf("[SOC] COV on cell%d (ocv %d mV, spread %d mV): charge complete, "
                            "offsets unchanged, pack stays at ",
                            (int)vmax_idx + 1, (int)vmax_ocv, (int)spread_ocv);
            log_pct("", g_status.soc.soc_percent); hal_uart_printf("%%\r\n");
            term_anchor_check(true, current_ma, vmax_ocv, 100.0f, learn_temp, now_ms);
        }
        if (!cov) s_cov_fired = false;
        if (g_status.soc.charge_complete && g_status.soc.soc_percent < s_charge_complete_soc - 2.0f) {
            g_status.soc.charge_complete = false;
            s_term.top_anchored = false;   /* the pack has been used: a new charge may anchor */
        }
        /* Mirror for the bottom latch: once the pack has taken 2 % SOC back, the next
         * discharge is a new event and may anchor again. */
        if (s_term.bot_anchored && g_status.soc.soc_percent > s_term.bot_soc + 2.0f) {
            s_term.bot_anchored = false;
        }
    }

    /* === T2: taper (MEDIUM) === */
    {
        int32_t entry_ma = (int32_t)((int64_t)cap_mah() * param_cache[PARAM_SOC_TAPER_ENTRY_C_X100].u8 / 100);
        int32_t exit_ma  = (int32_t)((int64_t)cap_mah() * param_cache[PARAM_SOC_TAPER_EXIT_C_X100].u8 / 100);
        bool entry = (current_ma > 0 && current_ma <= entry_ma);
        bool exitc = (current_ma >= exit_ma) || (current_ma <= 0);
        bool all_hi = true;
        for (int i = 0; i < SOC_NCELL; i++)
            if (v_ocv[i] < (int16_t)param_cache[PARAM_SOC_TAPER_CELL_MV].u16) all_hi = false;
        bool spread_ok = spread_ocv <= (int16_t)param_cache[PARAM_SOC_SPREAD_MED_MV].u8;
        bool no_cov = !(bq->safety_status_a & BQ_SAFETY_STATUS_A_COV);
        bool hold = s_taper.active ? !exitc : entry;
        bool ok = all_hi && hold && no_cov && spread_ok && anchors_allowed;
        if (ok) {
            if (!s_taper.active) { s_taper.active = true; s_taper.accum_ms = 0; }
            s_taper.accum_ms += dt_ms;
            s_taper.last_ok_ms = now_ms;
            if (s_taper.accum_ms >= (uint32_t)param_cache[PARAM_SOC_TAPER_HOLD_S].u16 * 1000u) {
                /* highest cell = 100; others by table difference */
                float top = voltage_to_soc(vmax_ocv);
                float minsoc = 100.0f;
                float cell_soc[SOC_NCELL];
                for (int i = 0; i < SOC_NCELL; i++) {
                    cell_soc[i] = 100.0f - (top - voltage_to_soc(v_ocv[i]));
                    if (cell_soc[i] < minsoc) minsoc = cell_soc[i];
                }
                for (int i = 0; i < SOC_NCELL; i++) s_cell_offset_pct[i] = cell_soc[i] - minsoc;
                g_status.soc.charge_complete = true;
                s_charge_complete_soc = minsoc;
                soc_anchor_target(minsoc, SOC_GRADE_MED, SOC_ANCHOR_TOP_TAPER);
                s_taper.active = false; s_taper.accum_ms = 0;
            }
        } else if (s_taper.active && (now_ms - s_taper.last_ok_ms) > ANCHOR_GRACE_MS) {
            s_taper.active = false; s_taper.accum_ms = 0;
        }
        if (s_taper.active) g_status.soc.anchor_flags |= SOC_FLAG_TAPER_ACTIVE;
    }

    /* === Rest anchors T1 / T1' / bottom rest (HIGH), one evaluation per rest period === */
    if (s_rest.qualified && s_rest.anchor_done && (now_ms - s_rest.anchor_ms) >= REST_REEVAL_MS)
        s_rest.anchor_done = false;   /* long rest: re-read with a more relaxed OCV */
    if (s_rest.qualified && !s_rest.anchor_done && anchors_allowed) {
        s_rest.anchor_done = true;
        s_rest.anchor_ms = now_ms;
        int16_t top_mv = (int16_t)param_cache[PARAM_SOC_TOP_REST_MV].u16;
        if (g_status.soc.charge_complete) {
            /* After a terminated charge, a rest just above the plateau band is still a
             * valid top point, so lower the top threshold to the band edge + 5 mV. */
            int16_t band_top = (int16_t)param_cache[PARAM_SOC_DISQ_HI_MV].u16 + 5;
            if (band_top < top_mv) top_mv = band_top;
        }
        int16_t bot_mv = (int16_t)param_cache[PARAM_SOC_BOT_REST_MV].u16;
        int16_t spread_high = (int16_t)param_cache[PARAM_SOC_SPREAD_HIGH_MV].u8;
        float prev_soc = g_status.soc.soc_percent;
        bool was_low_grade = (g_status.soc.anchor_grade != SOC_GRADE_HIGH);

        if (vmin_ocv >= top_mv && spread_ocv <= spread_high) {
            /* T1: balanced and full */
            for (int i = 0; i < SOC_NCELL; i++) s_cell_offset_pct[i] = 0.0f;
            soc_anchor_target(100.0f, SOC_GRADE_HIGH, SOC_ANCHOR_TOP_REST_FULL);
            learn_record_point(&s_qp_pack, 100.0f, learn_temp, now_ms, -1);
            for (int i = 0; i < SOC_NCELL; i++) learn_record_point(&s_qp_cell[i], 100.0f, learn_temp, now_ms, i);
        } else if (vmax_ocv >= top_mv || vmin_ocv <= bot_mv) {
            /* T1' (top, unbalanced) or bottom rest: per-cell OCV, band-refused cells keep their offset */
            float cell_soc[SOC_NCELL];
            bool  valid[SOC_NCELL];
            float ref_top = voltage_to_soc(vmax_ocv);
            for (int i = 0; i < SOC_NCELL; i++) {
                valid[i] = !in_disq_band(v_ocv[i]);
                if (valid[i]) {
                    if (g_status.soc.charge_complete && vmax_ocv >= top_mv)
                        cell_soc[i] = 100.0f - (ref_top - voltage_to_soc(v_ocv[i]));   /* tripped cell = full */
                    else
                        cell_soc[i] = voltage_to_soc(v_ocv[i]);
                } else {
                    cell_soc[i] = g_status.soc.soc_percent + s_cell_offset_pct[i];          /* carry */
                }
                cell_soc[i] = clamp_f(cell_soc[i], 0.0f, 100.0f);
            }
            float minsoc = 100.0f;
            for (int i = 0; i < SOC_NCELL; i++) if (cell_soc[i] < minsoc) minsoc = cell_soc[i];
            for (int i = 0; i < SOC_NCELL; i++) s_cell_offset_pct[i] = cell_soc[i] - minsoc;
            uint8_t ev = (vmax_ocv >= top_mv) ? SOC_ANCHOR_TOP_REST_CELL : SOC_ANCHOR_BOT_REST;
            soc_anchor_target(minsoc, SOC_GRADE_HIGH, ev);
            if (!in_disq_band(vmin_ocv)) learn_record_point(&s_qp_pack, minsoc, learn_temp, now_ms, -1);
            for (int i = 0; i < SOC_NCELL; i++)
                if (valid[i]) learn_record_point(&s_qp_cell[i], cell_soc[i], learn_temp, now_ms, i);
        } else {
            hal_uart_printf("[SOC] rest qualified in plateau (min %d max %d mV): no anchor\r\n",
                            (int)vmin_ocv, (int)vmax_ocv);
        }
        if (was_low_grade && s_conv.active) {
            g_status.soc.reconcile_delta_pct = s_conv.target - prev_soc;
        }
    }

    /* === The one place a correction reaches the charge count === */
    soc_arbitrate(dt_ms, low_current, current_ma);

    /* === Derived per-cell values === */
    {
        float mx = g_status.soc.soc_percent;
        float off_max = 0.0f;
        for (int i = 0; i < SOC_NCELL; i++) {
            if (s_cell_offset_pct[i] < 0) s_cell_offset_pct[i] = 0;
            g_status.soc.cell_offset_pct[i] = s_cell_offset_pct[i];
            g_status.soc.soc_cell_percent[i] = clamp_f(g_status.soc.soc_percent + s_cell_offset_pct[i], 0.0f, 100.0f);
            if (g_status.soc.soc_cell_percent[i] > mx) mx = g_status.soc.soc_cell_percent[i];
            if (s_cell_offset_pct[i] > off_max) off_max = s_cell_offset_pct[i];
        }
        g_status.soc.soc_max_percent = mx;
        g_status.soh.imbalance_loss_mah = (uint32_t)((float)cap_mah() * off_max / 100.0f);
        g_status.soh.usable_capacity_mah = cap_mah() - g_status.soh.imbalance_loss_mah;
    }

    /* === Remaining, times, flags === */
    g_status.soc.remaining_capacity_mah = (int32_t)(g_status.soc.soc_percent / 100.0f * (float)cap_mah());
    if (current_ma < -100) {
        g_status.soc.time_to_empty_min = battery_get_time_to_empty(current_ma);
        g_status.soc.time_to_full_min = 0;
    } else if (current_ma > 100) {
        g_status.soc.time_to_full_min = battery_get_time_to_full(current_ma);
        g_status.soc.time_to_empty_min = 0;
    }

    update_cycle_tracking(current_ma, g_status.soc.soc_percent, dt_ms);

    if (safe_abs32(current_ma) > 10000) {
        uint32_t ir = calculate_internal_resistance_ocv(voltage_mv, current_ma, g_status.soc.soc_percent);
        if (ir > 0 && ir < 1000)
            g_status.soh.internal_resistance_mohm = (g_status.soh.internal_resistance_mohm * 9 + ir) / 10;
    }

    {
        uint8_t f = g_status.soc.anchor_flags;
        if (s_rest.active)                 f |= SOC_FLAG_RESTING;
        if (s_rest.qualified)              f |= SOC_FLAG_REST_VALID;
        if (g_status.soc.is_calibrated)    f |= SOC_FLAG_CALIBRATED;
        if (g_status.soc.passq_jump_count) f |= SOC_FLAG_PASSQ_JUMP;
        g_status.soc.anchor_flags = f;
        uint8_t f2 = 0;
        if (s_rest.qualified)                f2 |= SOC_FLAG2_REST_QUALIFIED;
        if (g_status.soc.charge_complete)    f2 |= SOC_FLAG2_CHARGE_COMPLETE;
        if (s_conv.active)                   f2 |= SOC_FLAG2_CONVERGING;
        if (s_dvdt.ok)                       f2 |= SOC_FLAG2_DVDT_OK;
        if (param_cache[PARAM_SOC_LEARN_EN].u8) f2 |= SOC_FLAG2_LEARN_EN;
        if (low_current)                     f2 |= SOC_FLAG2_REST_LOWCURR;
        g_status.soc.anchor_flags2 = f2;
        g_status.soc.sign_fault = false;   /* always false; see the note at the top */
        g_status.soc.rest_qualified = s_rest.qualified;
    }
    g_status.soc.rest_duration_s = s_rest.active ? (uint16_t)((now_ms - s_rest.start_ms) / 1000) : 0;
    g_status.soc.is_fully_charged = (g_status.soc.soc_max_percent >= 99.5f);
    g_status.soc.is_empty = (g_status.soc.soc_percent <= 0.5f);
    g_status.soc.last_update_ms = now_ms;

    /* Periodic log line for the bench */
    if ((now_ms - s_v2_dbg_last_ms) >= V2_DBG_INTERVAL_MS) {
        s_v2_dbg_last_ms = now_ms;
        hal_uart_printf("[SOCV2] pack="); log_pct("", g_status.soc.soc_percent);
        hal_uart_printf(" max="); log_pct("", g_status.soc.soc_max_percent);
        hal_uart_printf(" off=[");
        for (int i = 0; i < SOC_NCELL; i++) { log_pct(i ? "," : "", s_cell_offset_pct[i]); }
        hal_uart_printf("] ocv=[%d,%d,%d,%d] I=%ld rest=%d q=%d dvdt=%d cc=%d sf=%d conv=%d grade=%d ev=%d cap=%lu loss=%lu\r\n",
                        (int)v_ocv[0], (int)v_ocv[1], (int)v_ocv[2], (int)v_ocv[3], (long)current_ma,
                        (int)s_rest.active, (int)s_rest.qualified, (int)s_dvdt.ok,
                        (int)g_status.soc.charge_complete, 0, (int)s_conv.active,
                        (int)g_status.soc.anchor_grade, (int)g_status.soc.last_anchor_event,
                        (unsigned long)cap_mah(), (unsigned long)g_status.soh.imbalance_loss_mah);
    }

    return 0;
}

const battery_status_t* battery_soc_soh_get_status(void)
{
    return &g_status;
}

int battery_soc_soh_set_method(soc_method_t method)
{
    g_status.method = method;
    return 0;
}

int battery_soc_calibrate(float soc_percent)
{
    if (soc_percent < 0 || soc_percent > 100) return -1;

    {
        int32_t v = (int32_t)(soc_percent * 10);
        hal_uart_printf("[SOC CAL] Setting SOC to %ld.%ld%%\r\n", (long)(v / 10), (long)(v % 10));
    }

    set_soc_direct(soc_percent);
    g_status.soc.soc_voltage_percent = soc_percent;
    g_status.soc.coulomb_count_error = 0;
    g_status.soc.used_capacity_mah = 0;
    g_status.soc.is_calibrated = true;
    s_soft_corr.active = false; s_soft_corr.hold_ms = 0;
    s_top_soft_corr.active = false; s_top_soft_corr.hold_ms = 0;
    s_conv.active = false; s_conv.owed = 0.0f;

    battery_soc_soh_save();
    return 0;
}

int battery_soh_calibrate(uint32_t measured_capacity_mah)
{
    if (measured_capacity_mah == 0) return -1;

    hal_uart_printf("[SOH CAL] Measured capacity: %lu mAh\r\n", (unsigned long)measured_capacity_mah);

    g_status.soh.actual_capacity_mah = measured_capacity_mah;
    sync_acc_from_soc();

    int32_t fade = (int32_t)g_config.nominal_capacity_mah - (int32_t)measured_capacity_mah;
    g_status.soh.capacity_fade_percent =
        ((float)fade / (float)g_config.nominal_capacity_mah) * 100.0f;

    float soh_capacity =
        ((float)measured_capacity_mah / (float)g_config.nominal_capacity_mah) * 100.0f;

    float soh_resistance = 100.0f;
    if (g_status.soh.internal_resistance_mohm > 0 && g_status.soh.initial_resistance_mohm > 0) {
        float r_increase =
            ((float)g_status.soh.internal_resistance_mohm /
             (float)g_status.soh.initial_resistance_mohm);
        soh_resistance = 100.0f / r_increase;
    }

    g_status.soh.soh_percent = (soh_capacity * 0.7f) + (soh_resistance * 0.3f);
    if (g_status.soh.soh_percent < 0) g_status.soh.soh_percent = 0;
    if (g_status.soh.soh_percent > 100) g_status.soh.soh_percent = 100;

    g_status.soh.last_calibration_ms = systime_milli();

    {
        int32_t cf = (int32_t)(g_status.soh.capacity_fade_percent * 10);
        int32_t sh = (int32_t)(g_status.soh.soh_percent * 10);
        hal_uart_printf("[SOH] Capacity fade: %ld.%ld%%, SOH: %ld.%ld%%\r\n",
                        (long)(cf / 10), (long)(cf % 10),
                        (long)(sh / 10), (long)(sh % 10));
    }
    return 0;
}

int battery_soc_soh_reset_stats(void)
{
    g_status.soh.cycle_count = 0;
    g_status.soh.full_cycles = 0;
    g_status.soh.total_charge_mah = 0;
    g_status.soh.total_discharge_mah = 0;
    g_status.soh.time_in_operation_hours = 0;
    hal_uart_printf("[SOC/SOH] Statistics reset\r\n");
    return 0;
}

int battery_soc_soh_save(void)
{
    /* TODO: Implement flash persistence for S32K116 (checklist item 30) */
    hal_uart_printf("[SOC/SOH] Save to flash (not implemented)\r\n");
    return 0;
}

int battery_soc_soh_load(void)
{
    /* TODO: Implement flash persistence for S32K116 (checklist item 30) */
    hal_uart_printf("[SOC/SOH] Load from flash (not implemented)\r\n");
    return 0;
}

void battery_soc_soh_print_status(void)
{
    if (!g_status.initialized) {
        hal_uart_printf("[ERROR] SOC/SOH not initialized\r\n");
        return;
    }

    hal_uart_printf("\r\n=== Battery SOC/SOH Status (v2) ===\r\n");

    {
        int32_t s = (int32_t)(g_status.soc.soc_percent * 10);
        int32_t sm = (int32_t)(g_status.soc.soc_max_percent * 10);
        int32_t sv = (int32_t)(g_status.soc.soc_voltage_percent * 10);
        hal_uart_printf("SOC pack(lowest cell): %ld.%ld%%  highest cell: %ld.%ld%%  Voltage(ref): %ld.%ld%%\r\n",
                        (long)(s / 10), (long)(s % 10),
                        (long)(sm / 10), (long)(sm % 10),
                        (long)(sv / 10), (long)(sv % 10));
    }
    hal_uart_printf("Cells SOC:");
    for (int i = 0; i < SOC_NCELL; i++) { log_pct(" ", g_status.soc.soc_cell_percent[i]); }
    hal_uart_printf("  offsets:");
    for (int i = 0; i < SOC_NCELL; i++) { log_pct(" +", s_cell_offset_pct[i]); }
    hal_uart_printf("\r\nOCV est (mV): %d %d %d %d\r\n",
                    (int)g_status.soc.v_ocv_mv[0], (int)g_status.soc.v_ocv_mv[1],
                    (int)g_status.soc.v_ocv_mv[2], (int)g_status.soc.v_ocv_mv[3]);
    hal_uart_printf("Remaining: %ld mAh / %lu mAh  usable: %lu  imbalance loss: %lu\r\n",
                    (long)g_status.soc.remaining_capacity_mah,
                    (unsigned long)g_status.soh.actual_capacity_mah,
                    (unsigned long)g_status.soh.usable_capacity_mah,
                    (unsigned long)g_status.soh.imbalance_loss_mah);
    {
        int32_t acc_mah = (int32_t)(s_accumulated_uah / 1000);
        hal_uart_printf("Accumulator: %ld mAh  raw: %ld mAh (epoch %lu)\r\n",
                        (long)acc_mah, (long)(s_raw_coulomb_uah / 1000), (unsigned long)s_raw_epoch);
    }
    {
        const bq_battery_status_t *bq_st = bq_controller_get_status();
        hal_uart_printf("PASSQ: raw=%ld uah=%ld last=%ld\r\n",
                        (long)(bq_st ? bq_st->passq_raw : 0),
                        (long)(bq_st ? bq_st->passed_charge_uah : 0),
                        (long)s_last_passed_charge_uah);
    }
    hal_uart_printf("Rest: %s", s_rest.active ? "yes" : "no");
    if (s_rest.active) {
        uint32_t rest_sec = (systime_milli() - s_rest.start_ms) / 1000;
        hal_uart_printf(" (%lus, dvdt=%s, qualified=%s, anchored=%s)",
                        (unsigned long)rest_sec, s_dvdt.ok ? "ok" : "settling",
                        s_rest.qualified ? "yes" : "no", s_rest.anchor_done ? "yes" : "no");
    }
    hal_uart_printf("\r\nAnchor: ev=%d grade=%d", (int)g_status.soc.last_anchor_event, (int)g_status.soc.anchor_grade);
    log_pct(" drift=", g_status.soc.drift_at_anchor_pct);
    log_pct(" reconcile=", g_status.soc.reconcile_delta_pct);
    hal_uart_printf(" converging=%d charge_complete=%d sign_fault=%d\r\n",
                    (int)s_conv.active, (int)g_status.soc.charge_complete, 0);
    hal_uart_printf("Learn: en=%d count=%d last_refusal=%d last=%lu mAh  cell caps: %lu %lu %lu %lu\r\n",
                    (int)param_cache[PARAM_SOC_LEARN_EN].u8, (int)g_status.soh.learn_count,
                    (int)g_status.soh.learn_refused_reason, (unsigned long)g_status.soh.last_learned_capacity_mah,
                    (unsigned long)g_status.soh.cell_capacity_mah[0], (unsigned long)g_status.soh.cell_capacity_mah[1],
                    (unsigned long)g_status.soh.cell_capacity_mah[2], (unsigned long)g_status.soh.cell_capacity_mah[3]);
    hal_uart_printf("Qpoints: pack %s", s_qp_pack.valid ? "" : "none");
    if (s_qp_pack.valid) { log_pct("", s_qp_pack.soc_pct); hal_uart_printf("%% raw=%ld mAh", (long)(s_qp_pack.raw_uah / 1000)); }
    hal_uart_printf("\r\n");

    if (g_status.soc.time_to_empty_min > 0) {
        hal_uart_printf("Time to empty: %luh %lum\r\n",
                        (unsigned long)(g_status.soc.time_to_empty_min / 60),
                        (unsigned long)(g_status.soc.time_to_empty_min % 60));
    }
    if (g_status.soc.time_to_full_min > 0) {
        hal_uart_printf("Time to full: %luh %lum\r\n",
                        (unsigned long)(g_status.soc.time_to_full_min / 60),
                        (unsigned long)(g_status.soc.time_to_full_min % 60));
    }

    {
        int32_t sh = (int32_t)(g_status.soh.soh_percent * 10);
        hal_uart_printf("\nSOH: %ld.%ld%%\r\n", (long)(sh / 10), (long)(sh % 10));
    }
    {
        int32_t cf = (int32_t)(g_status.soh.capacity_fade_percent * 10);
        hal_uart_printf("Capacity: %lu / %lu mAh (fade: %ld.%ld%%)\r\n",
                        (unsigned long)g_status.soh.actual_capacity_mah,
                        (unsigned long)g_status.soh.nominal_capacity_mah,
                        (long)(cf / 10), (long)(cf % 10));
    }
    hal_uart_printf("Internal R: %lu mOhm\r\n",
                    (unsigned long)g_status.soh.internal_resistance_mohm);
    hal_uart_printf("Cycles: %lu (full: %lu)\r\n",
                    (unsigned long)g_status.soh.cycle_count,
                    (unsigned long)g_status.soh.full_cycles);
    {
        int32_t eff = (int32_t)(g_status.soh.coulombic_efficiency * 100);
        int32_t ei = eff / 100, ef = eff % 100;
        if (ef < 0) ef = -ef;
        hal_uart_printf("Efficiency: %ld.%02ld%%\r\n", (long)ei, (long)ef);
    }

    hal_uart_printf("===========================\r\n\r\n");
}

uint32_t battery_get_time_to_empty(int32_t current_ma)
{
    if (current_ma >= 0) return 0;
    int32_t abs_current = -current_ma;
    if (abs_current < 100) return 0;
    return (uint32_t)((g_status.soc.remaining_capacity_mah * 60) / abs_current);
}

uint32_t battery_get_time_to_full(int32_t current_ma)
{
    if (current_ma <= 0 || current_ma < 100) return 0;
    /* full = the highest cell reaching 100 % */
    float headroom_pct = 100.0f - g_status.soc.soc_max_percent;
    if (headroom_pct <= 0) return 0;
    uint32_t needed = (uint32_t)((float)g_status.soh.actual_capacity_mah * headroom_pct / 100.0f);
    return (needed * 60 * 100) / ((uint32_t)current_ma * 95);
}

uint32_t battery_measure_internal_resistance(int16_t voltage_before_mv,
                                              int16_t voltage_after_mv,
                                              int32_t current_change_ma)
{
    if (safe_abs32(current_change_ma) < 10000) return 0;
    int32_t voltage_drop_mv = abs(voltage_after_mv - voltage_before_mv);
    return (voltage_drop_mv * 1000) / safe_abs32(current_change_ma);
}

void battery_soc_auto_calibration_enable(bool enable)
{
    /* No-op: endpoint anchoring calibrates automatically */
    (void)enable;
}

const char* battery_get_recommendation(void)
{
    if (g_status.soc.soc_percent < 10) {
        return "Critical: Charge immediately!";
    } else if (g_status.soc.soc_percent < 20) {
        return "Low: Charge soon";
    } else if (g_status.soh.soh_percent < 70) {
        return "Health degraded: Consider battery replacement";
    } else if (!g_status.soc.is_calibrated) {
        return "Calibration needed: Fully charge battery";
    } else if (g_status.soh.needs_calibration) {
        return "SOH calibration recommended";
    }
    return "OK";
}

/* === CONFIG_UPDATE hooks === */

void battery_soc_soh_pre_config_update(void)
{
    if (!g_status.initialized) return;

    /* Accumulate the latest PASSQ delta before CONFIG_UPDATE clears the counter */
    int32_t current_uah = bq_controller_get_passed_charge_uah();
    int32_t delta = current_uah - s_last_passed_charge_uah;
    if (delta > -100000) {
        s_accumulated_uah += (int64_t)delta;
        s_raw_coulomb_uah += (int64_t)delta;
    }
    s_last_passed_charge_uah = 0;
}

void battery_soc_soh_post_config_update(void)
{
    /* The caller has just reset PASSQ to 0: resync the delta tracker */
    s_last_passed_charge_uah = 0;

    battery_soc_soh_update_capacity();
}

void battery_soc_soh_update_capacity(void)
{
    uint32_t new_cap = param_cache[PARAM_DESIGN_CAPACITY_MAH].u32;
    if (new_cap > 0 && new_cap != g_config.nominal_capacity_mah) {
        hal_uart_printf("[SOC] Capacity changed: %lu -> %lu mAh\r\n",
                        (unsigned long)g_config.nominal_capacity_mah,
                        (unsigned long)new_cap);
        g_config.nominal_capacity_mah = new_cap;
        g_status.soh.nominal_capacity_mah = new_cap;
        g_status.soh.actual_capacity_mah = new_cap;
        for (int i = 0; i < SOC_NCELL; i++) g_status.soh.cell_capacity_mah[i] = new_cap;
        sync_acc_from_soc();
        g_status.soc.remaining_capacity_mah =
            (int32_t)((g_status.soc.soc_percent / 100.0f) * new_cap);
    }
}

bool battery_soc_is_rested(void)
{
    return s_rest.qualified;
}

/* ========== Private Functions ========== */

static float voltage_to_soc(int16_t voltage_mv)
{
    if (voltage_mv <= (int16_t)lfp_ocv_table[0].mv) return 0.0f;
    if (voltage_mv >= (int16_t)lfp_ocv_table[OCV_TABLE_SIZE - 1].mv) return 100.0f;
    for (int i = 0; i < (int)(OCV_TABLE_SIZE - 1); i++) {
        int16_t v0 = (int16_t)lfp_ocv_table[i].mv, v1 = (int16_t)lfp_ocv_table[i + 1].mv;
        if (voltage_mv >= v0 && voltage_mv <= v1) {
            float s0 = lfp_ocv_table[i].soc, s1 = lfp_ocv_table[i + 1].soc;
            return s0 + (s1 - s0) * ((float)(voltage_mv - v0) / (float)(v1 - v0));
        }
    }
    return 50.0f;
}

static int16_t soc_to_voltage(float soc_percent)
{
    if (soc_percent <= 0) return (int16_t)lfp_ocv_table[0].mv;
    if (soc_percent >= 100) return (int16_t)lfp_ocv_table[OCV_TABLE_SIZE - 1].mv;
    for (int i = 0; i < (int)(OCV_TABLE_SIZE - 1); i++) {
        float s0 = lfp_ocv_table[i].soc, s1 = lfp_ocv_table[i + 1].soc;
        if (soc_percent >= s0 && soc_percent <= s1) {
            float frac = (s1 > s0) ? (soc_percent - s0) / (s1 - s0) : 0.0f;
            return (int16_t)(lfp_ocv_table[i].mv +
                   (float)(lfp_ocv_table[i + 1].mv - lfp_ocv_table[i].mv) * frac);
        }
    }
    return (int16_t)lfp_ocv_table[OCV_TABLE_SIZE - 1].mv;
}

/* Hard bottom: the lowest cell is empty.  Sets SOC to 0 at once and records a learning
 * point only when the reading was taken at low current.  Offsets are untouched. */
static void soc_anchor_bottom(bool low_current, int16_t temp_c, uint32_t now)
{
    int32_t prev_x10 = (int32_t)(g_status.soc.soc_percent * 10);
    hal_uart_printf("[SOC] Bottom anchor: SOC=0%% (was %ld.%ld%%) lowcurr=%d\r\n",
                    (long)(prev_x10 / 10), (long)(prev_x10 % 10), (int)low_current);
    set_soc_direct(0.0f);
    s_conv.active = false; s_conv.owed = 0.0f;
    g_status.soc.is_calibrated = true;
    g_status.soc.last_anchor_event = SOC_ANCHOR_BOT_HARD0;
    g_status.soc.anchor_grade = low_current ? SOC_GRADE_HIGH : SOC_GRADE_MED;
    s_soft_corr.active = false; s_soft_corr.hold_ms = 0;
    s_top_soft_corr.active = false; s_top_soft_corr.hold_ms = 0;
    if (low_current) learn_record_point(&s_qp_pack, 0.0f, temp_c, now, -1);
}

static void update_cycle_tracking(int32_t current_ma, float soc_percent, uint32_t delta_time_ms)
{
    if (current_ma > 1000 && !s_cycle_tracker.in_charge_cycle) {
        s_cycle_tracker.in_charge_cycle = true;
        s_cycle_tracker.cycle_charge_mah = 0;
        s_cycle_tracker.charge_remainder = 0;
    }

    if (current_ma < -1000 && !s_cycle_tracker.in_discharge_cycle) {
        s_cycle_tracker.in_discharge_cycle = true;
        s_cycle_tracker.cycle_discharge_mah = 0;
        s_cycle_tracker.discharge_remainder = 0;
    }

    /* Track charge/discharge with remainder to prevent truncation */
    if (s_cycle_tracker.in_charge_cycle && current_ma > 0) {
        int64_t num = (int64_t)current_ma * (int64_t)delta_time_ms
                      + s_cycle_tracker.charge_remainder;
        int32_t delta_mah = (int32_t)(num / 3600000LL);
        s_cycle_tracker.charge_remainder = num % 3600000LL;
        s_cycle_tracker.cycle_charge_mah += delta_mah;
        g_status.soh.total_charge_mah += delta_mah;
    }
    if (s_cycle_tracker.in_discharge_cycle && current_ma < 0) {
        int64_t num = (int64_t)(-current_ma) * (int64_t)delta_time_ms
                      + s_cycle_tracker.discharge_remainder;
        int32_t delta_mah = (int32_t)(num / 3600000LL);
        s_cycle_tracker.discharge_remainder = num % 3600000LL;
        s_cycle_tracker.cycle_discharge_mah += delta_mah;
        g_status.soh.total_discharge_mah += delta_mah;
    }

    if (s_cycle_tracker.in_charge_cycle && soc_percent >= 99) {
        g_status.soh.cycle_count++;
        s_cycle_tracker.in_charge_cycle = false;
    }
    if (s_cycle_tracker.in_discharge_cycle && soc_percent <= 1) {
        s_cycle_tracker.in_discharge_cycle = false;
    }

    if (g_status.soh.total_discharge_mah > 0) {
        g_status.soh.full_cycles =
            g_status.soh.total_discharge_mah / g_config.nominal_capacity_mah;
    }

    if (g_status.soh.total_charge_mah > 0) {
        g_status.soh.coulombic_efficiency =
            ((float)g_status.soh.total_discharge_mah /
             (float)g_status.soh.total_charge_mah) * 100.0f;
    }
}

static uint32_t calculate_internal_resistance_ocv(int16_t voltage_mv, int32_t current_ma, float soc_percent)
{
    int16_t ocv_mv = soc_to_voltage(soc_percent);
    int32_t voltage_drop_mv = ocv_mv - voltage_mv;

    if (current_ma > 0) voltage_drop_mv = -voltage_drop_mv;
    if (voltage_drop_mv < 0) voltage_drop_mv = -voltage_drop_mv;

    int32_t abs_i = safe_abs32(current_ma);
    if (abs_i == 0) return 0;
    return (voltage_drop_mv * 1000) / abs_i;
}

#if CFG_SOC_TEMP_COMPENSATION_EN
static uint32_t lfp_temp_capacity_factor(int16_t temp_c)
{
    static const struct { int16_t t; uint16_t f; } tbl[] = {
        { -30, 550 }, { -20, 650 }, { -10, 780 }, {  0, 880 },
        {  10, 940 }, {  25, 1000}, {  45, 1000}, { 60, 980 },
    };
    enum { N = sizeof(tbl) / sizeof(tbl[0]) };

    if (temp_c <= -999) return 1000;
    if (temp_c <= tbl[0].t) return tbl[0].f;
    if (temp_c >= tbl[N - 1].t) return tbl[N - 1].f;

    for (int i = 0; i < N - 1; i++) {
        if (temp_c >= tbl[i].t && temp_c <= tbl[i + 1].t) {
            int32_t dt = tbl[i + 1].t - tbl[i].t;
            int32_t df = (int32_t)tbl[i + 1].f - (int32_t)tbl[i].f;
            return (uint32_t)((int32_t)tbl[i].f + df * (temp_c - tbl[i].t) / dt);
        }
    }
    return 1000;
}
#endif
