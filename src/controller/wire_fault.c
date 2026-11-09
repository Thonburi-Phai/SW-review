/* wire_fault.c -- response to a confirmed open cell tap; see wire_fault.h. */

#include "wire_fault.h"
#include "bq_controller.h"
#include "bq7690x_protection.h"
#include "bq7690x_fet.h"
#include "parameter.h"
#include "hal_systime.h"
#include "hal_uart.h"

/* Grace window default. Its cost is no autonomous undervoltage protection,
 * substituted by software at the 600 ms tick. Runtime parameter. */
#define WF_GRACE_DEFAULT_MS   (10u * 60u * 1000u)

/* Margin above CUV for the software substitute, doubled because the quantity is
 * a two-cell pair sum. Generous: the substitute runs at the 600 ms tick, so it
 * must trip before the real cells get anywhere near trouble. */
#define WF_UV_MARGIN_MV       150

static wf_status_t s_st;
static uint8_t     s_dsg_map_production;
static bool        s_production_map_saved;

static uint8_t dsg_map_now(void)
{
    param_value_t v;
    if (param_get(PARAM_DSG_FET_PROTECTIONS_A, &v)) {
        return v.u8;
    }
    return 0;
}

static bool dsg_map_write(uint8_t mask)
{
    param_value_t v;
    v.u8 = mask;
    if (!param_set(PARAM_DSG_FET_PROTECTIONS_A, &v)) {
        hal_uart_printf("[WF] DSG map %u rejected by bounds\r\n", mask);
        return false;
    }
    /* One batched CONFIG_UPDATE for the whole cache. It clears latched faults and
     * resets PASSQ as a side effect; the fault clear is what lets the FET
     * re-close once CUV is out of the map. */
    if (param_apply_to_bq() != 0) {
        hal_uart_printf("[WF] apply FAILED -- profile not in effect\r\n");
        return false;
    }
    return true;
}

void wire_fault_init(void)
{
    s_st.state       = WF_INACTIVE;
    s_st.kind        = OW_NONE;
    s_st.tap         = 0;
    s_st.entered_ms  = systime_milli();
    s_st.grace_left_ms = 0;
    s_st.reapply_count = 0;
    s_st.uv_substitute_trips = 0;
    s_st.balancing_locked = false;
    s_st.soc_anchors_suppressed = false;
    s_production_map_saved = false;
}

static bool gracable(ow_kind_t k)
{
    /* Only the two signatures the classifier can localise with confidence.
     * OW_BOTTOM_TAP is excluded although it is localised: it can fire the VSS
     * diagnostic, which sits in Both FET Protections B above the per-FET maps,
     * out of the limp profile's reach. */
    return (k == OW_MID_TAP) || (k == OW_TOP_TAP);
}

/*
 * Latch SERVICE: hold the power path open, lock balancing, suppress SOC anchors.
 *
 * Every route into WF_SERVICE goes through here except the exit from an active
 * limp profile, which leave_limp() owns because it must restore the production
 * map first. Keeping the steps in one place is the point: three of them are easy
 * to omit, and omitting them leaves a confirmed open wire with the power path
 * conducting, balancing permitted and SOC still anchoring on phantom voltages.
 *
 * The FETs are opened with the host OFF bits, which suspend no protection and
 * survive CONFIG_UPDATE. Locking balancing matters as much as the FETs:
 * run_balancing_task() gates on wire_fault_balancing_blocked(), and balancing
 * into a floating node is what drives the chip into SHUTDOWN with I2C dead.
 */
static void enter_service(const ow_verdict_t *v, const char *why)
{
    (void)bq_controller_request_fets_off(FET_OFF_OWNER_WIRE_FAULT,
                                         (uint8_t)(BQ_FET_DSG_OFF | BQ_FET_CHG_OFF));
    (void)bq_controller_manual_shutdown();

    s_st.state                  = WF_SERVICE;
    s_st.kind                   = v->kind;
    s_st.tap                    = v->tap;
    s_st.entered_ms             = systime_milli();
    s_st.grace_left_ms          = 0;
    s_st.balancing_locked       = true;
    s_st.soc_anchors_suppressed = true;

    hal_uart_printf("[WF] SERVICE REQUIRED (%s) -- pack out of use, FETs held open "
                    "by host override. Open wire must be repaired.\r\n", why);
}

static void enter_limp(const ow_verdict_t *v)
{
    uint8_t limp;

    if (!s_production_map_saved) {
        s_dsg_map_production = dsg_map_now();
        s_production_map_saved = true;
    }
    limp = (uint8_t)(s_dsg_map_production & (uint8_t)~BQ_DSGPROTA_CUV);

    hal_uart_printf("[WF] %s tap %u -- entering limp profile, DSG map 0x%02X -> 0x%02X\r\n",
                    ow_kind_name(v->kind), v->tap, s_dsg_map_production, limp);

    /* Clear any stale host override FIRST. Both the OFF and the ON bits persist
     * across CONFIG_UPDATE, so an override left from a manual command would
     * silently block autonomy in the new profile. Drop this owner's and the
     * manual owner's requests, then resync: resync writes unconditionally,
     * which the change-detecting path cannot do for a stale force-ON. Another
     * owner's live request survives. */
    (void)bq_controller_request_fets_off(FET_OFF_OWNER_MANUAL, 0);
    if (bq_controller_request_fets_off(FET_OFF_OWNER_WIRE_FAULT, 0) != 0 ||
        bq_controller_fets_off_resync() != 0) {
        enter_service(v, "could not clear FETControl; limp profile refused");
        return;
    }

    if (!dsg_map_write(limp)) {
        /* Production protection is still in force, which is the safe side of the
         * failure, but the pack still has a confirmed open wire. */
        enter_service(v, "could not write the limp DSG map");
        return;
    }

    /*
     * A top tap needs charging blocked explicitly. The limp profile's charge-side
     * safety is the PHANTOM COV from the mirrored cell above the break, which for
     * a mid-tap reads far above PARAM_CELL_OV_THRESHOLD_MV so the chip opens CHG
     * by itself; a top tap has no cell above it, so no cell reads high and COV --
     * the design's only overvoltage protection -- never asserts. The host OFF bit
     * suspends no protection and survives CONFIG_UPDATE, and is written after the
     * map write so CONFIG_UPDATE cannot disturb it. If it cannot be established,
     * the window is refused.
     */
    if (v->kind == OW_TOP_TAP) {
        if (bq_controller_request_fets_off(FET_OFF_OWNER_WIRE_FAULT, BQ_FET_CHG_OFF) != 0) {
            hal_uart_printf("[WF] could not block charging for a top tap -- "
                            "refusing limp profile\r\n");
            (void)dsg_map_write(s_dsg_map_production);
            enter_service(v, "could not block charging for a top tap");
            return;
        }
        hal_uart_printf("[WF] top tap: charging blocked by host CHG_OFF "
                        "(no mirrored cell, so no phantom COV)\r\n");
    }

    s_st.state        = WF_LIMP;
    s_st.kind         = v->kind;
    s_st.tap          = v->tap;
    s_st.entered_ms   = systime_milli();
    s_st.grace_left_ms = param_cache[PARAM_WF_GRACE_MS].u32;
    s_st.balancing_locked = true;
    s_st.soc_anchors_suppressed = true;

    hal_uart_printf("[WF] limp active, grace %lu s. Discharge continues; CHG stays "
                    "autonomous so charging is blocked by the phantom COV.\r\n",
                    (unsigned long)(s_st.grace_left_ms / 1000u));
}

static void leave_limp(const char *why, wf_state_t next)
{
    hal_uart_printf("[WF] leaving limp profile: %s\r\n", why);

    if (s_production_map_saved) {
        /* Restoring the production mask re-arms CUV, and with the wire still open
         * the chip re-trips immediately, since an already-true condition faults on
         * CONFIG_UPDATE exit without waiting out the delay. That is NOT what keeps
         * the pack out of use, though: CUV self-recovers when the voltage returns.
         * See the WF_SERVICE branch below. */
        (void)dsg_map_write(s_dsg_map_production);
    }

    /* Release any host override on recovery: a top-tap limp profile sets CHG_OFF
     * (see enter_limp) and those bits persist across CONFIG_UPDATE, so without
     * this charging would stay blocked for the rest of the power cycle. The
     * WF_SERVICE branch below deliberately does the opposite. */
    if (next == WF_INACTIVE) {
        if (bq_controller_request_fets_off(FET_OFF_OWNER_WIRE_FAULT, 0) != 0 ||
            bq_controller_fets_off_resync() != 0) {
            hal_uart_printf("[WF] WARNING: could not clear FETControl on recovery -- "
                            "charging may stay blocked\r\n");
        }
    }

    s_st.state = next;
    s_st.entered_ms = systime_milli();
    s_st.grace_left_ms = 0;
    s_st.balancing_locked = (next == WF_SERVICE);
    s_st.soc_anchors_suppressed = (next == WF_SERVICE);

    if (next == WF_SERVICE) {
        /*
         * Hold the power path open here as well: restoring the production map is
         * NOT enough, because CUV is not a permanent latch and recovers
         * autonomously once the cell voltage comes back, so an intermittent tap
         * reconnecting would re-close the FETs while this module still reports
         * SERVICE REQUIRED. The host OFF bits suspend no protection, unlike a
         * force-on, and survive CONFIG_UPDATE in both directions. Written after
         * the map restore so the CONFIG_UPDATE inside dsg_map_write() cannot
         * disturb them. Idempotent.
         */
        (void)bq_controller_manual_shutdown();

        hal_uart_printf("[WF] SERVICE REQUIRED -- pack out of use, FETs held open by "
                        "host override, chip left configured and reporting. Open "
                        "wire must be repaired.\r\n");
    }
}

/*
 * The software replacement for the one protection the limp profile drops: a floor
 * on the pair spanning the floating node. That pair is trustworthy as a SUM even
 * when the individual readings are nonsense, because it is the stack minus the
 * cells that still read true, and acting on it only writes the host OFF bits,
 * which suspend nothing.
 *
 * WHICH cells form the pair comes from the classifier's verdict, not from a
 * plausibility test on the readings: on tap 1 the collapsed cell settles at a
 * plausible cell voltage, so such a test counts it as intact and subtracts it
 * from the stack, yielding the other cell's share alone instead of the pair sum.
 */
static void uv_substitute(const ow_verdict_t *v, const int16_t *cell_mv, int16_t stack_mv)
{
    int32_t pair_sum;
    int32_t floor_mv;

    /* Only a localised mid-tap gets here: that is the only verdict the limp
     * profile is granted for, and the only one where "the pair" is defined. */
    if ((v->kind != OW_MID_TAP) || (v->tap == 0u) || (v->tap >= OW_CELLS)) {
        return;
    }

    pair_sum = ow_pair_sum_mv(v->tap, cell_mv, stack_mv);
    floor_mv = 2 * ((int32_t)param_cache[PARAM_CELL_UV_THRESHOLD_MV].u16 + WF_UV_MARGIN_MV);

    if (pair_sum < floor_mv) {
        s_st.uv_substitute_trips++;
        hal_uart_printf("[WF] UV substitute: tap %u pair-sum %ld mV below floor %ld mV -- FETs off\r\n",
                        v->tap, (long)pair_sum, (long)floor_mv);
        (void)bq_controller_manual_shutdown();
        leave_limp("software undervoltage floor reached", WF_SERVICE);
    }
}

void wire_fault_tick(const ow_verdict_t *v, bool fault_stands,
                     const int16_t *cell_mv, int16_t stack_mv)
{
    uint32_t now = systime_milli();

    switch (s_st.state) {
    case WF_INACTIVE:
        if (!fault_stands || (v->kind == OW_NONE)) {
            break;
        }
        if (!gracable(v->kind)) {
            /*
             * An unlocalised or unexplained verdict gets no grace: the production
             * profile stays in force so the chip keeps acting on everything, and
             * the power path is held open here too, because a fault that never
             * crosses CUV -- a bottom-tap open, which arrives as OW_AMBIGUOUS --
             * gives the chip nothing to object to. Host OFF bits, not a force-on:
             * they suspend no protection and survive CONFIG_UPDATE in both
             * directions. No map write is needed, since this branch never changed
             * the production profile.
             *
             * Deliberately asymmetric with `gracable()`: a verdict that cannot be
             * localised is treated as more serious than one that can, because the
             * limp profile has no defensible pair to substitute for.
             */
            hal_uart_printf("[WF] %s is not gracable -- no limp profile\r\n",
                            ow_kind_name(v->kind));
            enter_service(v, "verdict not gracable");
            break;
        }
        enter_limp(v);
        break;

    case WF_LIMP:
        /*
         * Wire came back and the classifier has finished proving it:
         * `fault_stands` stays true through RECOVERY and goes false only once the
         * full OW_CLEAR_TICKS debounce has elapsed. Holding the profile for that
         * whole window is the point: leaving it re-arms CUV and costs a
         * CONFIG_UPDATE, which opens both FET drivers, and because the exit
         * returns to WF_INACTIVE each flicker would otherwise earn a FRESH grace
         * window.
         */
        if (!fault_stands || (v->kind == OW_NONE)) {
            leave_limp("readings normalised, wire appears restored", WF_INACTIVE);
            s_st.kind = OW_NONE;
            s_st.tap  = 0;
            break;
        }

        /* An AFE reset reverts data memory to factory defaults, so the profile
         * silently disappears and the wire fault comes back unfiltered. Cheap
         * check, no CONFIG_UPDATE. */
        if ((dsg_map_now() & (uint8_t)BQ_DSGPROTA_CUV) != 0u) {
            hal_uart_printf("[WF] limp profile lost (AFE reset?) -- re-applying\r\n");
            s_st.reapply_count++;
            enter_limp(v);
            break;
        }

        uv_substitute(v, cell_mv, stack_mv);
        if (s_st.state != WF_LIMP) {
            break;          /* substitute tripped and already changed state */
        }

        {
            uint32_t elapsed = now - s_st.entered_ms;
            uint32_t grace   = param_cache[PARAM_WF_GRACE_MS].u32;
            s_st.grace_left_ms = (elapsed >= grace) ? 0u : (grace - elapsed);
            if (s_st.grace_left_ms == 0u) {
                leave_limp("grace window expired", WF_SERVICE);
            }
        }
        break;

    case WF_SERVICE:
        /*
         * Latched on purpose: re-entering the limp profile after the window has
         * been spent would hand out an unbounded series of grace periods for one
         * unrepaired fault. KNOWN LIMITATION: the latch is RAM only -- parameters
         * and state are re-seeded from defaults every boot, so an MCU reset grants
         * a fresh window.
         */
        break;
    }
}

const wf_status_t *wire_fault_status(void) { return &s_st; }

bool wire_fault_balancing_blocked(void) { return s_st.balancing_locked; }

bool wire_fault_soc_anchors_suppressed(void) { return s_st.soc_anchors_suppressed; }
