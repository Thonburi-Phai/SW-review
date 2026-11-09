/**
 * @file    parameter.h
 * @brief   BMS Parameter system definitions and API
 *
 * Provides:
 * - Parameter ID enumeration for BMS protection/config settings
 * - Type system (union-based, 32-bit aligned)
 * - Metadata structure with bounds and defaults
 * - PARAM_ADD macro for compile-time table generation
 * - Runtime get/set/validate functions
 */

#ifndef PARAMETER_H
#define PARAMETER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <limits.h>

/* ============================================================================
 * Parameter Type System
 * ============================================================================ */

typedef enum {
    PARAM_TYPE_UINT8  = 0,
    PARAM_TYPE_INT8   = 1,
    PARAM_TYPE_UINT16 = 2,
    PARAM_TYPE_INT16  = 3,
    PARAM_TYPE_UINT32 = 4,
    PARAM_TYPE_INT32  = 5,
    PARAM_TYPE_FLOAT  = 6,
} param_type_t;

/**
 * @brief Parameter value union (32-bit, 4 bytes)
 *
 * All parameters are stored as 32-bit values regardless of actual type.
 */
typedef union {
    uint8_t  u8;
    int8_t   i8;
    uint16_t u16;
    int16_t  i16;
    uint32_t u32;
    int32_t  i32;
    float    f32;
} param_value_t;

_Static_assert(sizeof(param_value_t) == 4, "param_value_t must be 4 bytes");

/* ============================================================================
 * Parameter ID Enumeration
 * ============================================================================ */

typedef enum {
    /* === System (0) === */
    PARAM_VERSION = 0,              /**< u32: Parameter structure version */

    /* === Cell Voltage Protection (1-6) === */
    PARAM_CELL_OV_THRESHOLD_MV,     /**< u16: Cell overvoltage threshold (mV) */
    PARAM_CELL_OV_DELAY_MS,         /**< u16: OV delay (ms) */
    PARAM_CELL_OV_RECOVERY_MV,      /**< u16: OV recovery hysteresis (mV) */
    PARAM_CELL_UV_THRESHOLD_MV,     /**< u16: Cell undervoltage threshold (mV) */
    PARAM_CELL_UV_DELAY_MS,         /**< u16: UV delay (ms) */
    PARAM_CELL_UV_RECOVERY_MV,      /**< u16: UV recovery hysteresis (mV) */

    /* === Current Protection (7-16) === */
    PARAM_OCC_THRESHOLD_A,          /**< u16: Overcurrent charge threshold (A) */
    PARAM_OCD1_THRESHOLD_A,         /**< u16: Overcurrent discharge 1 threshold (A) */
    PARAM_OCD2_THRESHOLD_A,         /**< u16: Overcurrent discharge 2 threshold (A) */
    PARAM_SCD_THRESHOLD_A,          /**< u16: Short circuit discharge threshold (A) */
    PARAM_LATCH_LIMIT,              /**< u8:  Latch limit (0=disabled, 1-7) */
    PARAM_CURRENT_RECOVERY_TIME_S,  /**< u8:  Current protection recovery time (s) */
    PARAM_OCC_DELAY_REG,            /**< u8:  OCC delay register (LSB=3.3ms) */
    PARAM_OCD1_DELAY_REG,           /**< u8:  OCD1 delay register (LSB=3.3ms) */
    PARAM_OCD2_DELAY_REG,           /**< u8:  OCD2 delay register (LSB=3.3ms) */
    PARAM_SCD_DELAY_REG,            /**< u8:  SCD delay register (0-10, see TRM) */

    /* === Temperature Protection (17-31) === */
    PARAM_OTC_THRESHOLD_C,          /**< i16: Overtemp charge threshold (C) */
    PARAM_UTC_THRESHOLD_C,          /**< i16: Undertemp charge threshold (C) */
    PARAM_OTD_THRESHOLD_C,          /**< i16: Overtemp discharge threshold (C) */
    PARAM_UTD_THRESHOLD_C,          /**< i16: Undertemp discharge threshold (C) */
    PARAM_OTINT_THRESHOLD_C,        /**< u8:  Internal overtemp threshold (C) */
    PARAM_OTC_DELAY_S,              /**< u8:  OTC delay (measurements, not seconds) */
    PARAM_UTC_DELAY_S,              /**< u8:  UTC delay (measurements) */
    PARAM_OTD_DELAY_S,              /**< u8:  OTD delay (measurements) */
    PARAM_UTD_DELAY_S,              /**< u8:  UTD delay (measurements) */
    PARAM_OTINT_DELAY_S,            /**< u8:  OTINT delay (measurements) */
    PARAM_OTC_RECOVERY_C,           /**< i16: OTC recovery threshold (C) */
    PARAM_UTC_RECOVERY_C,           /**< i16: UTC recovery threshold (C) */
    PARAM_OTD_RECOVERY_C,           /**< i16: OTD recovery threshold (C) */
    PARAM_UTD_RECOVERY_C,           /**< i16: UTD recovery threshold (C) */
    PARAM_OTINT_RECOVERY_C,         /**< u8:  OTINT recovery threshold (C) */

    /* === Cell Balancing (32-36) === */
    PARAM_BAL_START_MV,             /**< u16: Balancing start delta (mV) */
    PARAM_BAL_STOP_MV,              /**< u16: Balancing stop delta (mV) */
    PARAM_BAL_MIN_CELL_MV,          /**< u16: Min cell voltage for balancing (mV) */
    PARAM_BAL_MAX_TEMP_C,           /**< u8:  Max temp for balancing (C) */
    PARAM_BAL_MAX_DISCHARGE_A,      /**< u8:  Max discharge to allow balancing (A). Charge any value OK. */

    /* === IC: Sleep/Wake/Shutdown (37-43) === */
    PARAM_SLEEP_CURRENT_MA,         /**< u16: Sleep entry current threshold (mA) */
    PARAM_SLEEP_VOLTAGE_TIME_S,     /**< u8:  Sleep voltage settling time (s) */
    PARAM_WAKE_COMPARATOR_CURR,     /**< u8:  Wake comparator current (1-10, unit=500uV) */
    PARAM_SHUTDOWN_CELL_MV,         /**< i16: Auto shutdown cell voltage (mV) */
    PARAM_SHUTDOWN_STACK_MV,        /**< u16: Auto shutdown stack voltage (mV) */
    PARAM_SHUTDOWN_TEMP_C,          /**< u8:  Auto shutdown temperature (C) */
    PARAM_AUTO_SHUTDOWN_TIME_MIN,   /**< u8:  Auto shutdown time (minutes, TRM 0x9058) */

    /* === IC: FET/Config (44-46) === */
    PARAM_FET_OPTIONS,              /**< u8:  FET options register (bitmask) */
    PARAM_CHARGE_DETECTOR_TIME_100MS, /**< u8:  Charge detector time (unit=100ms, TRM 0x901F) */
    PARAM_BODY_DIODE_THRESHOLD,     /**< u16: Body diode threshold (userA, 0-32767) */

    /* === IC: Watchdog (47) === */
    PARAM_HOST_WATCHDOG_TIMEOUT_S,  /**< u8:  Host watchdog timeout (0=disabled) */

    /* === IC: Alarm/Protection Enable (48-53) === */
    PARAM_ALARM_ENABLE_MASK,        /**< u16: Alarm enable bitmask */
    PARAM_ENABLED_PROTECTIONS_A,    /**< u8:  Enabled protections A (bitmask) */
    PARAM_ENABLED_PROTECTIONS_B,    /**< u8:  Enabled protections B (bitmask) */
    PARAM_DSG_FET_PROTECTIONS_A,    /**< u8:  DSG FET protection actions (bitmask) */
    PARAM_CHG_FET_PROTECTIONS_A,    /**< u8:  CHG FET protection actions (bitmask) */
    PARAM_BOTH_FET_PROTECTIONS_B,   /**< u8:  Both FET protection actions (bitmask) */

    /* === IC: ADC/Coulomb Config (54-56) === */
    PARAM_DA_CONFIG,                /**< u16: DA Config register (IADCSPEED, CCMODE, TSMODE) */
    PARAM_POWER_CONFIG,             /**< u8:  Power Config register (LOOP_SLOW, CB_LOOP_SLOW, SLEEP_EN) */
    PARAM_DEFAULT_ALARM_MASK,       /**< u16: Default Alarm Mask (ALERT routing at POR) */

    /* === SW: System Config (57-) === */
    PARAM_RSENSE_MICRO_OHM,         /**< u32: Sense resistor value (uOhm) */
    PARAM_CELL_COUNT,               /**< u8:  Number of cells in series */
    PARAM_NOMINAL_CELL_MV,          /**< u16: Nominal cell voltage (mV) */
    PARAM_DESIGN_CAPACITY_MAH,      /**< u32: Design capacity (mAh) */
    PARAM_BAL_ALLOW_ADJACENT,       /**< u8:  Allow adjacent cell balancing (0/1) */
    PARAM_USE_RAW_CURRENT,          /**< u8:  Use raw current register (0/1) */
    PARAM_MAX_CURRENT_A,            /**< u16: Max expected current (A) */
    PARAM_BATTERY_MAX_VOLTAGE_MV,   /**< u16: Max safe cell voltage (mV) */
    PARAM_BATTERY_MIN_VOLTAGE_MV,   /**< u16: Min safe cell voltage (mV) */

    /* === SW: Overcurrent Protection (66-70) === */
    PARAM_SW_OC_DSG_THRESHOLD_A,    /**< u16: SW OC discharge threshold (A) */
    PARAM_SW_OC_CHG_THRESHOLD_A,    /**< u16: SW OC charge threshold (A) */
    PARAM_SW_OC_DSG_TIME_S,         /**< u16: SW OC discharge trip time (s) */
    PARAM_SW_OC_CHG_TIME_S,         /**< u16: SW OC charge trip time (s) */
    PARAM_SW_OC_DECAY_RATE,         /**< u8:  SW OC counter decay per tick */

    /* === SW: Fast Overcurrent (71-76) === */
    PARAM_SW_OC_FAST1_THRESHOLD_A,  /**< u16: Fast1 DSG threshold (A) */
    PARAM_SW_OC_FAST1_TIME_S,       /**< u16: Fast1 DSG trip time (s) */
    PARAM_SW_OC_FAST1_DECAY_RATE,   /**< u8:  Fast1 counter decay per tick */
    PARAM_SW_OC_FAST2_THRESHOLD_A,  /**< u16: Fast2 DSG threshold (A) */
    PARAM_SW_OC_FAST2_TIME_S,       /**< u16: Fast2 DSG trip time (s) */
    PARAM_SW_OC_FAST2_DECAY_RATE,   /**< u8:  Fast2 counter decay per tick */

    /* === Balance Duty Cycle (77-78) === */
    PARAM_BAL_CYCLE_TICKS,          /**< u8:  Total ticks per balance cycle (1 tick=600ms) */
    PARAM_BAL_ON_PCT,               /**< u8:  % into cycle when balance turns ON (duty=100-this) */

    /* === SOC Estimation (79-83) === */
    PARAM_SOC_CUV_RESERVE_X10,      /**< u8:  CUV anchor reserve (% × 10, e.g. 70 = 7.0%) */
    PARAM_SOC_SOFT_CORR_ERR_X10,    /**< u8:  Soft correction error band (% × 10, e.g. 50 = 5.0%) */
    PARAM_SOC_JUMP_DETECT_C_X10,    /**< u8:  PASSQ jump threshold (C-rate×10, 50=5.0C) */
    PARAM_SOC_TAPER_ENTRY_C_X100,   /**< u8:  Taper anchor entry current (C-rate×100, 33=0.33C) */
    PARAM_SOC_TAPER_EXIT_C_X100,    /**< u8:  Taper anchor exit current (C-rate×100, 53=0.53C) */

    /* === Balance Voltage Gate (84) === */
    PARAM_BAL_MIN_START_CELL_MV,    /**< u16: Only balance when vmax >= this (mV). LFP: 3350 */

    /* === SOC Top Soft Correction (85-87) === */
    PARAM_SOC_COV_RESERVE_X10,      /**< u16: COV ceiling (% × 10, e.g. 990 = 99.0%). Max SOC from top soft corr */
    PARAM_SOC_TOP_SOFT_CORR_ERR_X10, /**< u8: Top soft correction error band (% × 10, e.g. 50 = 5.0%) */
    PARAM_SOC_CELL_R_CHG_X10MOHM,  /**< u16: Cell IR for charge comp (×0.1 mΩ, e.g. 15 = 1.5mΩ) */

    /* === SOC anchoring v2 (88-) — see docs/soc_anchoring_redesign_plan.md === */
    PARAM_SOC_DISQ_LO_MV,           /**< u16: rest OCV band lower edge, no SOC info inside (mV) */
    PARAM_SOC_DISQ_HI_MV,           /**< u16: rest OCV band upper edge (mV) */
    PARAM_SOC_REST_C_X100,          /**< u8:  rest = |I| <= C × this/100 (5 = C/20), IR-corrected */
    PARAM_SOC_REST_MIN_MIN,         /**< u16: minimum rest before an anchor may fire (min) */
    PARAM_SOC_REST_MAX_MIN,         /**< u16: rest ceiling: qualify even if dV/dt never settles (min) */
    PARAM_SOC_REST_DVDT_X10UV_S,    /**< u8:  rest settled when |dV/dt| < this/10 µV/s (40 = 4 µV/s) */
    PARAM_SOC_TOP_REST_MV,          /**< u16: top rest anchor: highest cell OCV >= this (mV) */
    PARAM_SOC_BOT_REST_MV,          /**< u16: bottom rest anchor: lowest cell OCV <= this (mV) */
    PARAM_SOC_SPREAD_HIGH_MV,       /**< u8:  spread <= this → balanced, all offsets 0 (mV) */
    PARAM_SOC_SPREAD_MED_MV,        /**< u8:  taper anchor allowed only if spread <= this (mV) */
    PARAM_SOC_TAPER_CELL_MV,        /**< u16: taper anchor: all cells IR-corrected >= this (mV) */
    PARAM_SOC_TAPER_HOLD_S,         /**< u16: taper anchor hold time (s) */
    PARAM_SOC_CONV_REST_PCT_PER_MIN,   /**< u8: convergence rate at rest (%/min) */
    PARAM_SOC_CONV_LOAD_MIN_X10_PCT_PER_MIN, /**< u8: convergence floor under load (0.1 %/min) */
    PARAM_SOC_CONV_LOAD_MAX_PCT_PER_MIN,     /**< u8: convergence ceiling under load (%/min) */
    PARAM_SOC_LEARN_EN,             /**< u8:  capacity learning enable (0/1) */
    PARAM_SOC_LEARN_MIN_DSOC_PCT,   /**< u8:  minimum SOC span between qualified points (%) */
    PARAM_SOC_LEARN_TEMP_MIN_C,     /**< i16: learning allowed above this temp (°C) */
    PARAM_SOC_LEARN_TEMP_MAX_C,     /**< i16: learning allowed below this temp (°C) */
    PARAM_SOC_LEARN_STEP_DOWN_PCT,  /**< u8:  max capacity decrease per learn (% of nominal) */
    PARAM_SOC_LEARN_STEP_UP_PCT,    /**< u8:  max capacity increase per learn (% of nominal) */
    PARAM_SOC_SIGN_GUARD_MV,        /**< u8:  pack dV over 10 s that contradicts current sign (mV) */

    /* === Charge-termination anchor (T4) === */
    PARAM_SOC_TERM_C_X100,          /**< u8:  termination counts as "full" only at or below C x this/100 */
    PARAM_SOC_TERM_ERR_X10_PCT,     /**< u8:  largest tolerable termination error (0.1 % SOC) */
    PARAM_SOC_TERM_R_X10MOHM,       /**< u16: measured cell R, charge side (0.1 mOhm) */
    PARAM_SOC_TERM_R_DSG_X10MOHM,   /**< u16: cell R, discharge side, unmeasured (0.1 mOhm) */
    PARAM_SOC_LEARN_MAX_AGE_H,      /**< u16: a learning point older than this is not pairable (hours) */
    PARAM_SOC_LEARN_FIRST_W_PCT,    /**< u8:  weight given to the FIRST measurement against the nameplate */

    /* === Endpoint pull === */
    PARAM_SOC_PULL_Q_TAU_PCT,       /**< u8:  error decays 1/e per this much of capacity moved (%) */
    PARAM_SOC_BOT_PULL_ENTRY_MV,    /**< u16: corrected lowest cell, outer pull zone below this */
    PARAM_SOC_BOT_HARD_MV,          /**< u16: inner, current-blind zone below this */
    PARAM_SOC_TOP_PULL_ENTRY_MV,    /**< u16: corrected highest cell, outer pull zone above this */
    PARAM_SOC_TOP_HARD_MV,          /**< u16: inner, current-blind zone above this */
    PARAM_SOC_PULL_HARD_PCT_PER_MIN,/**< u8:  inner-zone correction rate (%/min) */

    /* === Sentinel (must be last) === */
    /* === Open-wire detection and wire-fault response === */
    PARAM_OW_ENABLE,                /**< u8:  run the cell open-wire classifier (0/1) */
    PARAM_WF_GRACE_MS,              /**< u32: limp-profile grace window (ms) */
    PARAM_OW_CELL_MIN_MV,           /**< i16: below this a cell reading is impossible */
    PARAM_OW_CELL_MAX_MV,           /**< i16: above this a cell reading is impossible */
    PARAM_OW_DRIFT_EPS_MV,          /**< i16: movement that counts as drift */
    PARAM_OW_PAIR_TOL_MV,           /**< i16: pair-sum match, unclamped case only */
    PARAM_OW_STACK_TOL_MV,          /**< i16: sum-of-cells vs stack match */
    PARAM_OW_CHECK_TIME,            /**< u8:  BQ CellOpenWireNORMALCheckTime (0x902B) */

    /* === MCU-side NTC safety (see ntc_bands.h) === */
    PARAM_NTC_SAFETY_EN,            /**< u8:  MCU temperature bands + NTC open-wire force-off (0/1) */
    PARAM_NTC_CHG_MIN_C,            /**< i16: CHG off below this (°C) */
    PARAM_NTC_CHG_PRECHG_END_C,     /**< i16: pre-charge band ends, normal charge begins (°C) */
    PARAM_NTC_CHG_NORMAL_MAX_C,     /**< i16: normal charge ends, extended begins (°C) */
    PARAM_NTC_CHG_MAX_C,            /**< i16: CHG off at or above this (°C) */
    PARAM_NTC_DSG_MIN_C,            /**< i16: DSG off below this (°C) */
    PARAM_NTC_DSG_NORMAL_MAX_C,     /**< i16: normal discharge ends, extended begins (°C) */
    PARAM_NTC_DSG_MAX_C,            /**< i16: DSG off at or above this (°C) */
    PARAM_NTC_FET_MIN_C,            /**< i16: CH4 lower limit, both FETs off below (°C) */
    PARAM_NTC_FET_MAX_C,            /**< i16: CH4 upper limit, both FETs off at or above (°C) */
    PARAM_NTC_HYST_C,               /**< u8:  band exit hysteresis (°C) */

    PARAM_COUNT

} param_id_t;

/* ============================================================================
 * Parameter Metadata Structure
 * ============================================================================ */

typedef struct {
    param_value_t min;              /**< Minimum valid value */
    param_value_t max;              /**< Maximum valid value */
    param_value_t default_value;    /**< Default/reset value */
    param_type_t type;              /**< Data type */
} param_metadata_t;

/* ============================================================================
 * PARAM_ADD Macro
 * ============================================================================ */

/**
 * @brief Macro for defining parameter metadata entries
 *
 * @param _ID       Parameter ID from param_id_t enum
 * @param _MIN      Minimum valid value
 * @param _MAX      Maximum valid value
 * @param _DEFAULT  Default value
 * @param _TYPE     Parameter type (PARAM_TYPE_xxx)
 * @param _MEMBER   Union member name (u8, i8, u16, i16, u32, i32)
 */
#define PARAM_ADD(_ID, _MIN, _MAX, _DEFAULT, _TYPE, _MEMBER) \
    [_ID] = {                                                  \
        .min._MEMBER = (_MIN),                                 \
        .max._MEMBER = (_MAX),                                 \
        .default_value._MEMBER = (_DEFAULT),                   \
        .type = (_TYPE),                                       \
    }

/* ============================================================================
 * External Declarations
 * ============================================================================ */

/**
 * @brief Parameter cache — in-memory copy of current parameter values
 * Size: PARAM_COUNT * 4 bytes
 */
extern volatile param_value_t param_cache[PARAM_COUNT];

/* ============================================================================
 * Function Prototypes
 * ============================================================================ */

/** Initialize parameter system (fills cache with defaults) */
void param_init(void);

/** Get parameter value from cache */
bool param_get(param_id_t id, param_value_t *value);

/** Set parameter value in cache (with bounds checking) */
bool param_set(param_id_t id, const param_value_t *value);

/** Get parameter type */
param_type_t param_get_type(param_id_t id);

/** Check if value is within bounds */
bool param_check_bounds(param_id_t id, const param_value_t *value);

/** Get default value for parameter */
bool param_get_default(param_id_t id, param_value_t *value);

/** Apply all default values to cache */
void param_apply_defaults_to_cache(void);

/** Get size of parameter data in bytes */
uint32_t param_get_data_size(void);

/**
 * @brief Apply all cached parameters to BQ76907 IC
 *
 * Enters CONFIG_UPDATE once, writes all protection/config registers
 * from param_cache[], then exits CONFIG_UPDATE.
 *
 * @return 0 on success, -1 on error
 */
int param_apply_to_bq(void);

#ifdef __cplusplus
}
#endif

#endif /* PARAMETER_H */
