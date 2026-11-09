/**
 * @file battery_soc_soh.h
 * @brief LiFePO4 SOC/SOH estimation: interface, status structs, anchor and refusal codes.
 *
 * Pack SOC is the lowest cell's SOC, integrated from the BQ76907 PASSQ counter into a
 * 64-bit accumulator; OCV lookup seeds it at init only, as the plateau is flat.  Graded
 * anchors at the two steep knees set a target that SOC converges toward (protection
 * events clamp only), per-cell offsets give the usable window and imbalance loss, and
 * capacity is learned TI-Qmax style from pairs of qualified rest points.
 *
 * Design: docs/soc_anchoring_redesign_plan.md
 */

#pragma once
#include <stdint.h>
#include <stdbool.h>

#define SOC_NCELL 4

/* ========== Battery Configuration ========== */

typedef struct {
    uint32_t nominal_capacity_mah;
    uint16_t nominal_voltage_mv;
    uint16_t full_voltage_mv;
    uint16_t empty_voltage_mv;
    uint8_t  cell_count;
    int16_t  nominal_temp_c;
} battery_config_t;

#define BATTERY_DEFAULT_CONFIG { \
    .nominal_capacity_mah = BATTERY_CAPACITY_MAH, \
    .nominal_voltage_mv = 3200,      /* 3.2V nominal */ \
    .full_voltage_mv = 3650,         /* 3.65V full */ \
    .empty_voltage_mv = 2500,        /* 2.5V empty */ \
    .cell_count = 4,                 /* 4S */ \
    .nominal_temp_c = 25             /* 25°C */ \
}

/* ========== SOC Estimation Methods ========== */

typedef enum {
    SOC_METHOD_COULOMB_COUNTING,    // Pure coulomb counting; the LFP default
    SOC_METHOD_VOLTAGE_BASED,       // Voltage lookup table (init only)
} soc_method_t;

/* Anchor event types for telemetry (values are stable for the app) */
#define SOC_ANCHOR_NONE          0
#define SOC_ANCHOR_TOP_REST_FULL 1   /* T1:  qualified rest, balanced, all cells high -> 100 % */
#define SOC_ANCHOR_TOP_REST_PART 2   /* unused */
#define SOC_ANCHOR_TOP_TAPER     3   /* T2:  charge taper, MEDIUM */
#define SOC_ANCHOR_TOP_COV       4   /* T3:  COV event -> ceiling only, LOW */
#define SOC_ANCHOR_BOT_SOFT_CORR 5   /* rate-limited steep-region correction */
#define SOC_ANCHOR_BOT_GROSS_ERR 6   /* gross error hard snap (SOC >> voltage SOC) */
#define SOC_ANCHOR_BOT_HARD0     7   /* hard bottom 0 % */
#define SOC_ANCHOR_BOT_CUV       8   /* CUV fault -> reserve, LOW */
#define SOC_ANCHOR_TOP_SOFT_CORR 9   /* rate-limited steep-top upward correction, MEDIUM */
#define SOC_ANCHOR_TOP_GROSS_ERR 10  /* unused */
#define SOC_ANCHOR_TOP_REST_CELL 11  /* T1': qualified rest, per-cell OCV, HIGH */
#define SOC_ANCHOR_BOT_REST      12  /* qualified rest with lowest cell in the bottom knee, HIGH */
#define SOC_ANCHOR_SIGN_FAULT    13  /* unused; never reported */
#define SOC_ANCHOR_TOP_TERM      14  /* T4: qualified charge termination -> full, MEDIUM */
#define SOC_ANCHOR_BOT_TERM      15  /* B4: qualified discharge termination -> empty, MEDIUM */

/* Anchor grade */
typedef enum {
    SOC_GRADE_NONE = 0,
    SOC_GRADE_LOW  = 1,   /* protection event, clamp only */
    SOC_GRADE_MED  = 2,   /* taper / under-load IR-compensated */
    SOC_GRADE_HIGH = 3,   /* qualified rest, OCV outside the plateau band */
} soc_grade_t;

/* Live anchor state flags (byte 1) */
#define SOC_FLAG_RESTING       (1 << 0)
#define SOC_FLAG_REST_VALID    (1 << 1)   /* rest qualified: min time + (dV/dt settled or ceiling) */
#define SOC_FLAG_TAPER_ACTIVE  (1 << 2)
#define SOC_FLAG_IN_STEEP      (1 << 3)
#define SOC_FLAG_CALIBRATED    (1 << 4)
#define SOC_FLAG_PASSQ_JUMP    (1 << 5)
#define SOC_FLAG_IN_STEEP_TOP  (1 << 6)

/* Flags (byte 2) */
#define SOC_FLAG2_SIGN_FAULT      (1 << 0)   /* always clear */
#define SOC_FLAG2_REST_QUALIFIED  (1 << 1)
#define SOC_FLAG2_CHARGE_COMPLETE (1 << 2)
#define SOC_FLAG2_CONVERGING      (1 << 3)
#define SOC_FLAG2_DVDT_OK         (1 << 4)
#define SOC_FLAG2_LEARN_EN        (1 << 5)
#define SOC_FLAG2_REST_LOWCURR    (1 << 6)   /* |I| <= C/20 */

/* Capacity-learning refusal reasons */
#define SOC_LEARN_OK        0
#define SOC_LEARN_DISABLED  1
#define SOC_LEARN_NO_PRIOR  2   /* first qualified point, nothing to pair with */
#define SOC_LEARN_EPOCH     3   /* raw counter continuity broken between points */
#define SOC_LEARN_DSOC      4   /* points too close (< min ΔSOC) */
#define SOC_LEARN_TEMP      5   /* unused; never reported */
#define SOC_LEARN_SANITY    6   /* result outside 0.5..1.5 × the learned reference */
#define SOC_LEARN_SIGN      7   /* unused; never reported */
#define SOC_LEARN_AGE       8   /* the stored point is older than the age limit */

typedef struct {
    float soc_percent;               /* pack SOC = LOWEST cell's SOC (displayed) */
    float soc_coulomb_percent;
    float soc_voltage_percent;

    int32_t remaining_capacity_mah;
    int32_t used_capacity_mah;

    float coulomb_count_mah;
    float coulomb_count_error;

    uint32_t time_to_empty_min;
    uint32_t time_to_full_min;

    bool is_fully_charged;
    bool is_empty;
    bool is_calibrated;

    uint8_t last_anchor_event;
    uint8_t anchor_flags;
    uint16_t rest_duration_s;
    uint8_t passq_jump_count;

    float    soc_cell_percent[SOC_NCELL];   /* soc_percent + cell_offset_pct[i] */
    float    cell_offset_pct[SOC_NCELL];    /* >= 0; 0 for the lowest cell */
    float    soc_max_percent;               /* highest cell */
    int16_t  v_ocv_mv[SOC_NCELL];           /* IR-corrected cell voltages used for anchoring */
    uint8_t  anchor_grade;                  /* soc_grade_t of the last anchor */
    uint8_t  anchor_flags2;                 /* SOC_FLAG2_* */
    float    drift_at_anchor_pct;           /* target - soc when the last anchor fired */
    float    reconcile_delta_pct;           /* same, for the last rest reconciliation */
    bool     sign_fault;                    /* always false */
    bool     rest_qualified;
    bool     charge_complete;

    uint32_t last_update_ms;
} battery_soc_t;

typedef struct {
    float soh_percent;

    uint32_t actual_capacity_mah;           /* pack (lowest-cell reference) learned capacity */
    uint32_t nominal_capacity_mah;
    float capacity_fade_percent;

    uint32_t internal_resistance_mohm;
    uint32_t initial_resistance_mohm;
    float resistance_increase_percent;

    uint32_t cycle_count;
    uint32_t full_cycles;
    float discharge_depth_avg;

    uint32_t total_charge_mah;
    uint32_t total_discharge_mah;

    float coulombic_efficiency;

    uint32_t time_in_operation_hours;
    uint32_t last_calibration_ms;

    bool needs_calibration;

    /* Raw coulomb counter for capacity learning: never corrected, never reset by anchors */
    int32_t  raw_coulomb_mah;
    uint32_t last_learned_capacity_mah;
    bool     raw_coulomb_active;

    uint32_t cell_capacity_mah[SOC_NCELL];
    uint32_t usable_capacity_mah;           /* actual_capacity - imbalance_loss */
    uint32_t imbalance_loss_mah;            /* actual_capacity * offset_max / 100 */
    uint8_t  learn_count;
    uint8_t  learn_refused_reason;          /* SOC_LEARN_* of the last attempt */
    uint32_t last_learn_ms;
} battery_soh_t;

typedef struct {
    battery_soc_t soc;
    battery_soh_t soh;
    soc_method_t method;
    bool initialized;
} battery_status_t;

/* ========== Public Functions ========== */

int battery_soc_soh_init(const battery_config_t *config, float initial_soc_percent);

int battery_soc_soh_update(int16_t voltage_mv, int32_t current_ma,
                           int16_t temp_c, uint32_t delta_time_ms);

const battery_status_t* battery_soc_soh_get_status(void);

int battery_soc_soh_set_method(soc_method_t method);

int battery_soc_calibrate(float soc_percent);

int battery_soh_calibrate(uint32_t measured_capacity_mah);

int battery_soc_soh_reset_stats(void);

int battery_soc_soh_save(void);

int battery_soc_soh_load(void);

void battery_soc_soh_print_status(void);

uint32_t battery_get_time_to_empty(int32_t current_ma);

uint32_t battery_get_time_to_full(int32_t current_ma);

uint32_t battery_measure_internal_resistance(int16_t voltage_before_mv,
                                              int16_t voltage_after_mv,
                                              int32_t current_change_ma);

void battery_soc_auto_calibration_enable(bool enable);

const char* battery_get_recommendation(void);

void battery_soc_soh_pre_config_update(void);

void battery_soc_soh_post_config_update(void);

/** Re-read capacity from param_cache without resetting PASSQ tracker */
void battery_soc_soh_update_capacity(void);

/** @return true when the pack is at a qualified rest (low current, settled dV/dt or ceiling) */
bool battery_soc_is_rested(void);
