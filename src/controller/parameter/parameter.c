/**
 * @file    parameter.c
 * @brief   BMS Parameter system implementation
 *
 * No flash storage — uses defaults on every boot.
 * param_apply_to_bq() writes cached values to BQ76907 via CONFIG_UPDATE.
 */

#include "parameter.h"
#include "bq_config_4s_lfp.h"
#include "../bq7690x/bq7690x_config.h"
#include "../bq7690x/bq7690x_protection.h"
#include "../bq7690x/bq7690x_status.h"
#include <string.h>

/* ============================================================================
 * Parameter Cache
 * ============================================================================ */

volatile param_value_t param_cache[PARAM_COUNT];

/* ============================================================================
 * Parameter Metadata Table
 * ============================================================================ */

static const param_metadata_t param_metadata[PARAM_COUNT] = {

    /* === System === */
    PARAM_ADD(PARAM_VERSION,               1, UINT32_MAX, 0x010000, PARAM_TYPE_UINT32, u32),

    /* === Cell Voltage Protection === */
    PARAM_ADD(PARAM_CELL_OV_THRESHOLD_MV,  2500, 4200, CFG_CELL_OV_THRESHOLD_MV, PARAM_TYPE_UINT16, u16),
    PARAM_ADD(PARAM_CELL_OV_DELAY_MS,      100,  CFG_VOLT_PROT_DELAY_MAX_MS, CFG_CELL_OV_DELAY_MS, PARAM_TYPE_UINT16, u16),
    PARAM_ADD(PARAM_CELL_OV_RECOVERY_MV,   50,   200,  CFG_CELL_OV_RECOVERY_MV,  PARAM_TYPE_UINT16, u16),
    PARAM_ADD(PARAM_CELL_UV_THRESHOLD_MV,  2000, 3500, CFG_CELL_UV_THRESHOLD_MV, PARAM_TYPE_UINT16, u16),
    PARAM_ADD(PARAM_CELL_UV_DELAY_MS,      100,  CFG_VOLT_PROT_DELAY_MAX_MS, CFG_CELL_UV_DELAY_MS, PARAM_TYPE_UINT16, u16),
    PARAM_ADD(PARAM_CELL_UV_RECOVERY_MV,   50,   200,  CFG_CELL_UV_RECOVERY_MV,  PARAM_TYPE_UINT16, u16),

    /* === Current Protection === */
    PARAM_ADD(PARAM_OCC_THRESHOLD_A,       1, 1000, CFG_OCC_THRESHOLD_A,           PARAM_TYPE_UINT16, u16),
    PARAM_ADD(PARAM_OCD1_THRESHOLD_A,      1, 2000, CFG_OCD1_THRESHOLD_A,          PARAM_TYPE_UINT16, u16),
    PARAM_ADD(PARAM_OCD2_THRESHOLD_A,      1, 2000, CFG_OCD2_THRESHOLD_A,          PARAM_TYPE_UINT16, u16),
    PARAM_ADD(PARAM_SCD_THRESHOLD_A,       5, 5000, CFG_SCD_THRESHOLD_A,           PARAM_TYPE_UINT16, u16),
    PARAM_ADD(PARAM_LATCH_LIMIT,           0, 7,   CFG_LATCH_LIMIT,              PARAM_TYPE_UINT8,  u8),
    PARAM_ADD(PARAM_CURRENT_RECOVERY_TIME_S, 1, 255, CFG_CURRENT_RECOVERY_TIME_S, PARAM_TYPE_UINT8,  u8),
    PARAM_ADD(PARAM_OCC_DELAY_REG,         0, 255, CFG_OCC_DELAY_REG,            PARAM_TYPE_UINT8,  u8),
    PARAM_ADD(PARAM_OCD1_DELAY_REG,        0, 255, CFG_OCD1_DELAY_REG,           PARAM_TYPE_UINT8,  u8),
    PARAM_ADD(PARAM_OCD2_DELAY_REG,        0, 255, CFG_OCD2_DELAY_REG,           PARAM_TYPE_UINT8,  u8),
    PARAM_ADD(PARAM_SCD_DELAY_REG,         0, 10,  CFG_SCD_DELAY_REG,            PARAM_TYPE_UINT8,  u8),

    /* === Temperature Protection === */
    PARAM_ADD(PARAM_OTC_THRESHOLD_C,       0,   100, CFG_OTC_THRESHOLD_C,         PARAM_TYPE_INT16,  i16),
    PARAM_ADD(PARAM_UTC_THRESHOLD_C,       -40, 30,  CFG_UTC_THRESHOLD_C,         PARAM_TYPE_INT16,  i16),
    PARAM_ADD(PARAM_OTD_THRESHOLD_C,       0,   100, CFG_OTD_THRESHOLD_C,         PARAM_TYPE_INT16,  i16),
    PARAM_ADD(PARAM_UTD_THRESHOLD_C,       -40, 10,  CFG_UTD_THRESHOLD_C,         PARAM_TYPE_INT16,  i16),
    PARAM_ADD(PARAM_OTINT_THRESHOLD_C,     50,  120, CFG_OTINT_THRESHOLD_C,       PARAM_TYPE_UINT8,  u8),
    PARAM_ADD(PARAM_OTC_DELAY_S,           1, 255, CFG_OTC_DELAY_MEAS,           PARAM_TYPE_UINT8,  u8),
    PARAM_ADD(PARAM_UTC_DELAY_S,           1, 255, CFG_UTC_DELAY_MEAS,           PARAM_TYPE_UINT8,  u8),
    PARAM_ADD(PARAM_OTD_DELAY_S,           1, 255, CFG_OTD_DELAY_MEAS,           PARAM_TYPE_UINT8,  u8),
    PARAM_ADD(PARAM_UTD_DELAY_S,           1, 255, CFG_UTD_DELAY_MEAS,           PARAM_TYPE_UINT8,  u8),
    PARAM_ADD(PARAM_OTINT_DELAY_S,         1, 255, CFG_OTINT_DELAY_MEAS,         PARAM_TYPE_UINT8,  u8),
    PARAM_ADD(PARAM_OTC_RECOVERY_C,        0,   100, CFG_OTC_RECOVERY_C,          PARAM_TYPE_INT16,  i16),
    PARAM_ADD(PARAM_UTC_RECOVERY_C,        -40, 30,  CFG_UTC_RECOVERY_C,          PARAM_TYPE_INT16,  i16),
    PARAM_ADD(PARAM_OTD_RECOVERY_C,        0,   100, CFG_OTD_RECOVERY_C,          PARAM_TYPE_INT16,  i16),
    PARAM_ADD(PARAM_UTD_RECOVERY_C,        -40, 10,  CFG_UTD_RECOVERY_C,          PARAM_TYPE_INT16,  i16),
    PARAM_ADD(PARAM_OTINT_RECOVERY_C,      50,  120, CFG_OTINT_RECOVERY_C,        PARAM_TYPE_UINT8,  u8),

    /* === Cell Balancing === */
    PARAM_ADD(PARAM_BAL_START_MV,          5, 100,  CFG_BAL_START_MV,             PARAM_TYPE_UINT16, u16),
    PARAM_ADD(PARAM_BAL_STOP_MV,           1, 50,   CFG_BAL_STOP_MV,             PARAM_TYPE_UINT16, u16),
    PARAM_ADD(PARAM_BAL_MIN_CELL_MV,       2000, 3500, CFG_BAL_MIN_CELL_MV,      PARAM_TYPE_UINT16, u16),
    PARAM_ADD(PARAM_BAL_MAX_TEMP_C,        20, 80,  CFG_BAL_MAX_TEMP_C,          PARAM_TYPE_UINT8,  u8),
    PARAM_ADD(PARAM_BAL_MAX_DISCHARGE_A,   0,  250, CFG_BAL_MAX_DISCHARGE_A,     PARAM_TYPE_UINT8,  u8),

    /* === IC: Sleep/Wake/Shutdown === */
    PARAM_ADD(PARAM_SLEEP_CURRENT_MA,      1,  500,   CFG_SLEEP_CURRENT_MA,       PARAM_TYPE_UINT16, u16),
    PARAM_ADD(PARAM_SLEEP_VOLTAGE_TIME_S,  1,  255,   CFG_SLEEP_VOLTAGE_TIME_S,   PARAM_TYPE_UINT8,  u8),
    PARAM_ADD(PARAM_WAKE_COMPARATOR_CURR,  1,  10,    CFG_WAKE_COMPARATOR_CURR,   PARAM_TYPE_UINT8,  u8),
    /* min bound 0 so "disabled" (TRM: 0 = shutdown path disabled) is expressible */
    PARAM_ADD(PARAM_SHUTDOWN_CELL_MV,      0, 3000, CFG_SHUTDOWN_CELL_MV,      PARAM_TYPE_INT16,  i16),
    PARAM_ADD(PARAM_SHUTDOWN_STACK_MV,     0, 16000, CFG_SHUTDOWN_STACK_MV,    PARAM_TYPE_UINT16, u16),
    PARAM_ADD(PARAM_SHUTDOWN_TEMP_C,       0, 120,    CFG_SHUTDOWN_TEMP_C,       PARAM_TYPE_UINT8,  u8),
    PARAM_ADD(PARAM_AUTO_SHUTDOWN_TIME_MIN, 0,  255,   CFG_AUTO_SHUTDOWN_TIME_MIN, PARAM_TYPE_UINT8,  u8),

    /* === IC: FET/Config === */
    PARAM_ADD(PARAM_FET_OPTIONS,           0,  255,    CFG_FET_OPTIONS,           PARAM_TYPE_UINT8,  u8),
    PARAM_ADD(PARAM_CHARGE_DETECTOR_TIME_100MS, 0, 255, CFG_CHARGE_DETECTOR_TIME_100MS, PARAM_TYPE_UINT8, u8),
    PARAM_ADD(PARAM_BODY_DIODE_THRESHOLD, 0, 32767, CFG_BODY_DIODE_THRESHOLD, PARAM_TYPE_UINT16, u16),

    /* === IC: Watchdog === */
    PARAM_ADD(PARAM_HOST_WATCHDOG_TIMEOUT_S, 0, 255,  CFG_HOST_WATCHDOG_TIMEOUT_S, PARAM_TYPE_UINT8, u8),

    /* === IC: Alarm/Protection Enable === */
    PARAM_ADD(PARAM_ALARM_ENABLE_MASK,     0, 0xFFFF, CFG_ALARM_ENABLE_MASK,     PARAM_TYPE_UINT16, u16),
    PARAM_ADD(PARAM_ENABLED_PROTECTIONS_A, 0, 0xFE,   CFG_ENABLED_PROTECTIONS_A, PARAM_TYPE_UINT8,  u8),
    PARAM_ADD(PARAM_ENABLED_PROTECTIONS_B, 0, 0x3F,    CFG_ENABLED_PROTECTIONS_B, PARAM_TYPE_UINT8,  u8),
    PARAM_ADD(PARAM_DSG_FET_PROTECTIONS_A, 0, 255,    CFG_DSG_FET_PROTECTIONS_A, PARAM_TYPE_UINT8,  u8),
    PARAM_ADD(PARAM_CHG_FET_PROTECTIONS_A, 0, 0xEF,   CFG_CHG_FET_PROTECTIONS_A, PARAM_TYPE_UINT8,  u8),
    PARAM_ADD(PARAM_BOTH_FET_PROTECTIONS_B, 0, 0x07,  CFG_BOTH_FET_PROTECTIONS_B, PARAM_TYPE_UINT8, u8),

    /* === IC: ADC/Coulomb Config === */
    PARAM_ADD(PARAM_DA_CONFIG,             0, 0x01FF, CFG_DA_CONFIG,             PARAM_TYPE_UINT16, u16),
    PARAM_ADD(PARAM_POWER_CONFIG,          0, 0xFF,   CFG_POWER_CONFIG,          PARAM_TYPE_UINT8,  u8),
    PARAM_ADD(PARAM_DEFAULT_ALARM_MASK,    0, 0xFFFF, CFG_DEFAULT_ALARM_MASK,    PARAM_TYPE_UINT16, u16),

    /* === SW: System Config === */
    PARAM_ADD(PARAM_RSENSE_MICRO_OHM,      100, 10000, CFG_RSENSE_MICRO_OHM,     PARAM_TYPE_UINT32, u32),
    PARAM_ADD(PARAM_CELL_COUNT,            3,  4,     BATTERY_CELLS,              PARAM_TYPE_UINT8,  u8),
    PARAM_ADD(PARAM_NOMINAL_CELL_MV,       2500, 3600, BATTERY_NOMINAL_VOLTAGE_MV, PARAM_TYPE_UINT16, u16),
    PARAM_ADD(PARAM_DESIGN_CAPACITY_MAH,   1000, 300000, BATTERY_CAPACITY_MAH,    PARAM_TYPE_UINT32, u32),
    PARAM_ADD(PARAM_BAL_ALLOW_ADJACENT,    0, 1,      CFG_BAL_ALLOW_ADJACENT,     PARAM_TYPE_UINT8,  u8),
    PARAM_ADD(PARAM_USE_RAW_CURRENT,       0, 1,      CFG_USE_RAW_CURRENT,        PARAM_TYPE_UINT8,  u8),
    PARAM_ADD(PARAM_MAX_CURRENT_A,         1, 2000,   CFG_MAX_CURRENT_A,          PARAM_TYPE_UINT16, u16),
    PARAM_ADD(PARAM_BATTERY_MAX_VOLTAGE_MV, 3000, 4200, BATTERY_MAX_VOLTAGE_MV,  PARAM_TYPE_UINT16, u16),
    PARAM_ADD(PARAM_BATTERY_MIN_VOLTAGE_MV, 2000, 3000, BATTERY_MIN_VOLTAGE_MV,  PARAM_TYPE_UINT16, u16),

    /* === SW: Overcurrent Protection === */
    PARAM_ADD(PARAM_SW_OC_DSG_THRESHOLD_A,  1, 2000, 130,  PARAM_TYPE_UINT16, u16),
    PARAM_ADD(PARAM_SW_OC_CHG_THRESHOLD_A,  1, 2000, 100,  PARAM_TYPE_UINT16, u16),
    PARAM_ADD(PARAM_SW_OC_DSG_TIME_S,       6, 36000, 300, PARAM_TYPE_UINT16, u16),
    PARAM_ADD(PARAM_SW_OC_CHG_TIME_S,       6, 36000, 1500, PARAM_TYPE_UINT16, u16),
    PARAM_ADD(PARAM_SW_OC_DECAY_RATE,       1, 50, 5,      PARAM_TYPE_UINT8,  u8),

    /* === SW: Fast Overcurrent === */
    PARAM_ADD(PARAM_SW_OC_FAST1_THRESHOLD_A, 1, 2000, 600, PARAM_TYPE_UINT16, u16),
    PARAM_ADD(PARAM_SW_OC_FAST1_TIME_S,      1, 3600, 10,  PARAM_TYPE_UINT16, u16),
    PARAM_ADD(PARAM_SW_OC_FAST1_DECAY_RATE,  1, 50, 5,     PARAM_TYPE_UINT8,  u8),
    PARAM_ADD(PARAM_SW_OC_FAST2_THRESHOLD_A, 1, 2000, 800, PARAM_TYPE_UINT16, u16),
    PARAM_ADD(PARAM_SW_OC_FAST2_TIME_S,      1, 3600, 5,   PARAM_TYPE_UINT16, u16),
    PARAM_ADD(PARAM_SW_OC_FAST2_DECAY_RATE,  1, 50, 5,     PARAM_TYPE_UINT8,  u8),
    PARAM_ADD(PARAM_BAL_CYCLE_TICKS,        3, 50, CFG_BAL_CYCLE_TICKS, PARAM_TYPE_UINT8, u8),
    PARAM_ADD(PARAM_BAL_ON_PCT,             20, 80, CFG_BAL_ON_PCT,     PARAM_TYPE_UINT8, u8),

    /* === SOC Estimation === */
    PARAM_ADD(PARAM_SOC_CUV_RESERVE_X10,    0, 250, 0,   PARAM_TYPE_UINT8, u8),
    PARAM_ADD(PARAM_SOC_SOFT_CORR_ERR_X10,  10, 200, 50, PARAM_TYPE_UINT8, u8),
    PARAM_ADD(PARAM_SOC_JUMP_DETECT_C_X10,  10, 100, 50, PARAM_TYPE_UINT8, u8),
    PARAM_ADD(PARAM_SOC_TAPER_ENTRY_C_X100, 1, 100, CFG_SOC_TAPER_ENTRY_C_X100, PARAM_TYPE_UINT8, u8),
    PARAM_ADD(PARAM_SOC_TAPER_EXIT_C_X100,  1, 200, CFG_SOC_TAPER_EXIT_C_X100,  PARAM_TYPE_UINT8, u8),

    /* === Balance Voltage Gate === */
    PARAM_ADD(PARAM_BAL_MIN_START_CELL_MV, 2800, 3600, CFG_BAL_MIN_START_CELL_MV, PARAM_TYPE_UINT16, u16),

    /* === SOC Top Soft Correction === */
    PARAM_ADD(PARAM_SOC_COV_RESERVE_X10,       900, 1000, 990, PARAM_TYPE_UINT16, u16),
    PARAM_ADD(PARAM_SOC_TOP_SOFT_CORR_ERR_X10,  10,  200,  50, PARAM_TYPE_UINT8,  u8),
    PARAM_ADD(PARAM_SOC_CELL_R_CHG_X10MOHM,    1, 1000, CFG_SOC_CELL_R_CHG_X10MOHM, PARAM_TYPE_UINT16, u16),

    /* === SOC anchoring v2 === */
    PARAM_ADD(PARAM_SOC_DISQ_LO_MV,          3000, 3400, CFG_SOC_DISQ_LO_MV,          PARAM_TYPE_UINT16, u16),
    PARAM_ADD(PARAM_SOC_DISQ_HI_MV,          3200, 3500, CFG_SOC_DISQ_HI_MV,          PARAM_TYPE_UINT16, u16),
    PARAM_ADD(PARAM_SOC_REST_C_X100,         1,    50,   CFG_SOC_REST_C_X100,         PARAM_TYPE_UINT8,  u8),
    PARAM_ADD(PARAM_SOC_REST_MIN_MIN,        1,    600,  CFG_SOC_REST_MIN_MIN,        PARAM_TYPE_UINT16, u16),
    PARAM_ADD(PARAM_SOC_REST_MAX_MIN,        5,    1440, CFG_SOC_REST_MAX_MIN,        PARAM_TYPE_UINT16, u16),
    PARAM_ADD(PARAM_SOC_REST_DVDT_X10UV_S,   1,    200,  CFG_SOC_REST_DVDT_X10UV_S,   PARAM_TYPE_UINT8,  u8),
    PARAM_ADD(PARAM_SOC_TOP_REST_MV,         3300, 3600, CFG_SOC_TOP_REST_MV,         PARAM_TYPE_UINT16, u16),
    PARAM_ADD(PARAM_SOC_BOT_REST_MV,         2500, 3200, CFG_SOC_BOT_REST_MV,         PARAM_TYPE_UINT16, u16),
    PARAM_ADD(PARAM_SOC_SPREAD_HIGH_MV,      5,    200,  CFG_SOC_SPREAD_HIGH_MV,      PARAM_TYPE_UINT8,  u8),
    PARAM_ADD(PARAM_SOC_SPREAD_MED_MV,       5,    250,  CFG_SOC_SPREAD_MED_MV,       PARAM_TYPE_UINT8,  u8),
    PARAM_ADD(PARAM_SOC_TAPER_CELL_MV,       3300, 3650, CFG_SOC_TAPER_CELL_MV,       PARAM_TYPE_UINT16, u16),
    PARAM_ADD(PARAM_SOC_TAPER_HOLD_S,        10,   3600, CFG_SOC_TAPER_HOLD_S,        PARAM_TYPE_UINT16, u16),
    PARAM_ADD(PARAM_SOC_CONV_REST_PCT_PER_MIN,         1, 255, CFG_SOC_CONV_REST_PCT_PER_MIN,         PARAM_TYPE_UINT8, u8),
    PARAM_ADD(PARAM_SOC_CONV_LOAD_MIN_X10_PCT_PER_MIN, 1, 100, CFG_SOC_CONV_LOAD_MIN_X10_PCT_PER_MIN, PARAM_TYPE_UINT8, u8),
    PARAM_ADD(PARAM_SOC_CONV_LOAD_MAX_PCT_PER_MIN,     1, 60,  CFG_SOC_CONV_LOAD_MAX_PCT_PER_MIN,     PARAM_TYPE_UINT8, u8),
    PARAM_ADD(PARAM_SOC_LEARN_EN,            0,    1,    CFG_SOC_CAPACITY_LEARNING_EN, PARAM_TYPE_UINT8, u8),
    PARAM_ADD(PARAM_SOC_LEARN_MIN_DSOC_PCT,  5,    95,   CFG_SOC_LEARN_MIN_DSOC_PCT,  PARAM_TYPE_UINT8,  u8),
    PARAM_ADD(PARAM_SOC_LEARN_TEMP_MIN_C,    -20,  40,   CFG_SOC_LEARN_TEMP_MIN_C,    PARAM_TYPE_INT16,  i16),
    PARAM_ADD(PARAM_SOC_LEARN_TEMP_MAX_C,    20,   80,   CFG_SOC_LEARN_TEMP_MAX_C,    PARAM_TYPE_INT16,  i16),
    PARAM_ADD(PARAM_SOC_LEARN_STEP_DOWN_PCT, 1,    20,   CFG_SOC_LEARN_STEP_DOWN_PCT, PARAM_TYPE_UINT8,  u8),
    PARAM_ADD(PARAM_SOC_LEARN_STEP_UP_PCT,   1,    20,   CFG_SOC_LEARN_STEP_UP_PCT,   PARAM_TYPE_UINT8,  u8),
    /* retired; the slot stays so parameter IDs do not shift */
    PARAM_ADD(PARAM_SOC_SIGN_GUARD_MV,       5,    250,  CFG_SOC_SIGN_GUARD_MV,       PARAM_TYPE_UINT8,  u8),

    /* === Charge-termination anchor (T4) === */
    PARAM_ADD(PARAM_SOC_TERM_C_X100,         1,    50,   CFG_SOC_TERM_C_X100,         PARAM_TYPE_UINT8,  u8),
    PARAM_ADD(PARAM_SOC_TERM_ERR_X10_PCT,    1,    200,  CFG_SOC_TERM_ERR_X10_PCT,    PARAM_TYPE_UINT8,  u8),
    PARAM_ADD(PARAM_SOC_TERM_R_X10MOHM,      1,    1000, CFG_SOC_TERM_R_X10MOHM,      PARAM_TYPE_UINT16, u16),
    PARAM_ADD(PARAM_SOC_TERM_R_DSG_X10MOHM,  1,    1000, CFG_SOC_TERM_R_DSG_X10MOHM,  PARAM_TYPE_UINT16, u16),
    PARAM_ADD(PARAM_SOC_LEARN_MAX_AGE_H,     1,    8760, CFG_SOC_LEARN_MAX_AGE_H,     PARAM_TYPE_UINT16, u16),
    PARAM_ADD(PARAM_SOC_LEARN_FIRST_W_PCT,   1,    100,  CFG_SOC_LEARN_FIRST_W_PCT,   PARAM_TYPE_UINT8,  u8),

    /* === Endpoint pull === */
    PARAM_ADD(PARAM_SOC_PULL_Q_TAU_PCT,      1,    50,  CFG_SOC_PULL_Q_TAU_PCT,    PARAM_TYPE_UINT8,  u8),
    PARAM_ADD(PARAM_SOC_BOT_PULL_ENTRY_MV, 2600, 3300, CFG_SOC_BOT_PULL_ENTRY_MV, PARAM_TYPE_UINT16, u16),
    PARAM_ADD(PARAM_SOC_BOT_HARD_MV,       2500, 3100, CFG_SOC_BOT_HARD_MV,       PARAM_TYPE_UINT16, u16),
    PARAM_ADD(PARAM_SOC_TOP_PULL_ENTRY_MV, 3300, 3600, CFG_SOC_TOP_PULL_ENTRY_MV, PARAM_TYPE_UINT16, u16),
    PARAM_ADD(PARAM_SOC_TOP_HARD_MV,       3350, 3650, CFG_SOC_TOP_HARD_MV,       PARAM_TYPE_UINT16, u16),
    PARAM_ADD(PARAM_SOC_PULL_HARD_PCT_PER_MIN, 1, 120, CFG_SOC_PULL_HARD_PCT_PER_MIN, PARAM_TYPE_UINT8, u8),
    /* === Open-wire detection === defaults mirror ow_config_defaults() so the
     * firmware and the host test cannot drift apart.  The grace window costs
     * autonomous undervoltage protection while it is open. */
    PARAM_ADD(PARAM_OW_ENABLE,             0, 1,        1,     PARAM_TYPE_UINT8,  u8),
    PARAM_ADD(PARAM_WF_GRACE_MS,           0, 3600000,  600000, PARAM_TYPE_UINT32, u32),
    PARAM_ADD(PARAM_OW_CELL_MIN_MV,        -2000, 1000, 0,     PARAM_TYPE_INT16,  i16),
    PARAM_ADD(PARAM_OW_CELL_MAX_MV,        3700, 7000,  4000,  PARAM_TYPE_INT16,  i16),
    PARAM_ADD(PARAM_OW_DRIFT_EPS_MV,       10, 500,     50,    PARAM_TYPE_INT16,  i16),
    PARAM_ADD(PARAM_OW_PAIR_TOL_MV,        10, 500,     150,   PARAM_TYPE_INT16,  i16),
    PARAM_ADD(PARAM_OW_STACK_TOL_MV,       50, 2000,    300,   PARAM_TYPE_INT16,  i16),
    PARAM_ADD(PARAM_OW_CHECK_TIME,         0, 255,      0,     PARAM_TYPE_UINT8,  u8),

    /* MCU-side NTC safety */
    PARAM_ADD(PARAM_NTC_SAFETY_EN,         0,   1,   CFG_NTC_SAFETY_EN,        PARAM_TYPE_UINT8, u8),
    PARAM_ADD(PARAM_NTC_CHG_MIN_C,        -40,  20,  CFG_NTC_CHG_MIN_C,        PARAM_TYPE_INT16, i16),
    PARAM_ADD(PARAM_NTC_CHG_PRECHG_END_C, -40,  30,  CFG_NTC_CHG_PRECHG_END_C, PARAM_TYPE_INT16, i16),
    PARAM_ADD(PARAM_NTC_CHG_NORMAL_MAX_C,  0,   120, CFG_NTC_CHG_NORMAL_MAX_C, PARAM_TYPE_INT16, i16),
    PARAM_ADD(PARAM_NTC_CHG_MAX_C,         0,   125, CFG_NTC_CHG_MAX_C,        PARAM_TYPE_INT16, i16),
    PARAM_ADD(PARAM_NTC_DSG_MIN_C,        -40,  20,  CFG_NTC_DSG_MIN_C,        PARAM_TYPE_INT16, i16),
    PARAM_ADD(PARAM_NTC_DSG_NORMAL_MAX_C,  0,   120, CFG_NTC_DSG_NORMAL_MAX_C, PARAM_TYPE_INT16, i16),
    PARAM_ADD(PARAM_NTC_DSG_MAX_C,         0,   125, CFG_NTC_DSG_MAX_C,        PARAM_TYPE_INT16, i16),
    PARAM_ADD(PARAM_NTC_FET_MIN_C,        -40,  20,  CFG_NTC_FET_MIN_C,        PARAM_TYPE_INT16, i16),
    PARAM_ADD(PARAM_NTC_FET_MAX_C,         0,   125, CFG_NTC_FET_MAX_C,        PARAM_TYPE_INT16, i16),
    PARAM_ADD(PARAM_NTC_HYST_C,            0,   20,  CFG_NTC_HYST_C,           PARAM_TYPE_UINT8, u8),
};

/* ============================================================================
 * Public Functions
 * ============================================================================ */

void param_init(void)
{
    param_apply_defaults_to_cache();
}

bool param_get(param_id_t id, param_value_t *value)
{
    if (id >= PARAM_COUNT || value == NULL) {
        return false;
    }
    *value = param_cache[id];
    return true;
}

bool param_set(param_id_t id, const param_value_t *value)
{
    if (id >= PARAM_COUNT || value == NULL) {
        return false;
    }
    if (!param_check_bounds(id, value)) {
        return false;
    }
    param_cache[id] = *value;
    return true;
}

param_type_t param_get_type(param_id_t id)
{
    if (id >= PARAM_COUNT) {
        return PARAM_TYPE_UINT32;
    }
    return param_metadata[id].type;
}

bool param_check_bounds(param_id_t id, const param_value_t *value)
{
    if (id >= PARAM_COUNT || value == NULL) {
        return false;
    }

    const param_metadata_t *meta = &param_metadata[id];

    switch (meta->type) {
    case PARAM_TYPE_UINT8:
        return (value->u8 >= meta->min.u8) && (value->u8 <= meta->max.u8);
    case PARAM_TYPE_INT8:
        return (value->i8 >= meta->min.i8) && (value->i8 <= meta->max.i8);
    case PARAM_TYPE_UINT16:
        return (value->u16 >= meta->min.u16) && (value->u16 <= meta->max.u16);
    case PARAM_TYPE_INT16:
        return (value->i16 >= meta->min.i16) && (value->i16 <= meta->max.i16);
    case PARAM_TYPE_UINT32:
        return (value->u32 >= meta->min.u32) && (value->u32 <= meta->max.u32);
    case PARAM_TYPE_INT32:
        return (value->i32 >= meta->min.i32) && (value->i32 <= meta->max.i32);
    case PARAM_TYPE_FLOAT:
        return (value->f32 >= meta->min.f32) && (value->f32 <= meta->max.f32);
    default:
        return false;
    }
}

bool param_get_default(param_id_t id, param_value_t *value)
{
    if (id >= PARAM_COUNT || value == NULL) {
        return false;
    }
    *value = param_metadata[id].default_value;
    return true;
}

void param_apply_defaults_to_cache(void)
{
    for (uint32_t i = 0; i < PARAM_COUNT; i++) {
        param_cache[i] = param_metadata[i].default_value;
    }
}

uint32_t param_get_data_size(void)
{
    return (uint32_t)(PARAM_COUNT * sizeof(param_value_t));
}
