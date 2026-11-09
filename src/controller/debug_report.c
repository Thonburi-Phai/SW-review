/**
 * @file debug_report.c
 * @brief Debug report output: CSV text for a serial plotter, or binary protocol binary
 *        packets for the app.
 *
 * @author Ming
 */

#include "link_bms_wire_fault.h"
#include "link_bms_ntc_fault.h"
#include "wire_fault.h"
#include "ntc_fault.h"
#include "parameter.h"
#include "cell_open_wire.h"
#include "debug_report.h"
#include "hal_uart.h"
#include "hal_uart1.h"
#include "hal_systime.h"
#include "bq_controller.h"
#include "bq7690x_status.h"
#include "external_temp_adc.h"
#include "battery_soc_soh.h"
#include "system_health.h"
#include "link_protocol.h"
#include "link_bms_mcu_status.h"
#include "link_bms_balance_debug.h"
#include "link_bms_soc_cell.h"
#include "sw_oc_protection.h"

/*---------------------------------------------------------------------------*/
/* Private Defines                                                           */
/*---------------------------------------------------------------------------*/

/** @brief CSV text format output type */
#define DEBUG_TYPE_CSV 0

/** @brief binary protocol binary format output type */
#define DEBUG_TYPE_BP 1

/*---------------------------------------------------------------------------*/
/* Private Variables                                                         */
/*---------------------------------------------------------------------------*/

/** @brief Debug output enable flag */
static bool debug_enabled = false;

/** @brief Current debug output type (CSV or binary protocol) */
static uint8_t debug_type = DEBUG_TYPE_BP;

/** @brief Debug report interval in milliseconds */
static uint32_t debug_interval_ms = DEBUG_REPORT_DEFAULT_INTERVAL_MS;

/** @brief Timestamp of last report output */
static uint32_t last_report_time = 0;

/** @brief Flag to track if CSV header has been printed */
static bool header_printed = false;

/** @brief Transmit buffer for binary protocol packets (44 bytes needed) */
static uint8_t tx_buffer[64];

/** @brief Broadcast stagger counter (rotates secondary packets) */
static uint8_t broadcast_cycle = 0;

/*---------------------------------------------------------------------------*/
/* Forward Declarations                                                      */
/*---------------------------------------------------------------------------*/

static void debug_report_send_energy(void);
static void debug_report_send_soc_soh(void);
static void debug_report_send_fault_detail(void);
static void debug_report_send_wire_fault(void);
static void debug_report_send_ntc_fault(void);
static void debug_report_send_mcu_status(void);
static void debug_report_send_balance_debug(void);
static void debug_report_send_ext_temp(void);
static void debug_report_send_soc_cell(void);

/*---------------------------------------------------------------------------*/
/* Private Functions                                                         */
/*---------------------------------------------------------------------------*/

/**
 * @brief Print CSV header row with column names.
 */
static void debug_report_print_header_csv(void)
{
    hal_uart_printf("time_ms,C1_mV,C2_mV,C3_mV,C4_mV,stack_mV,delta_mV,"
                      "current_mA,raw_current_mA,power_mW,"
                      "temp_int_C,temp_ts1_C,"
                      "ntc1_C,ntc2_C,ntc3_C,ntc4_C,"
                      "soc_x100,soh_x100,"
                      "passq_uAh,remain_mAh,"
                      "chg_fet,dsg_fet,balancing,"
                      "fault_a,fault_b,i2c_ok\r\n");
}

/**
 * @brief Print debug data in CSV format.
 */
static void debug_report_print_csv(void)
{
    const bq_battery_status_t *bq = bq_controller_get_status();
    const ext_temp_module_status_t *ext = ext_temp_adc_get_status();
    const battery_status_t *bat = battery_soc_soh_get_status();

    uint32_t timestamp = systime_milli();

    int16_t ntc1_temp = (ext != NULL) ? ext->sensors[0].temp_c : 0;
    int16_t ntc2_temp = (ext != NULL) ? ext->sensors[1].temp_c : 0;
    int16_t ntc3_temp = (ext != NULL) ? ext->sensors[2].temp_c : 0;
    int16_t ntc4_temp = (ext != NULL) ? ext->sensors[3].temp_c : 0;

    /* Get SOC/SOH as integer x100 (e.g., 85.25% -> 8525) */
    int32_t soc_x100 = (bat != NULL && bat->initialized) ? (int32_t)(bat->soc.soc_percent * 100) : 0;
    int32_t soh_x100 = (bat != NULL && bat->initialized) ? (int32_t)(bat->soh.soh_percent * 100) : 0;

    /* Extract FET status from battery_status register (bit2=DSG, bit3=CHG) */
    uint8_t chg_on = (bq->battery_status & BQ_BATT_STAT_CHG_FET) ? 1 : 0;
    uint8_t dsg_on = (bq->battery_status & BQ_BATT_STAT_DSG_FET) ? 1 : 0;

    int32_t passq_uah = bq->passed_charge_uah;
    int32_t remain_mah = (bat != NULL && bat->initialized) ? bat->soc.remaining_capacity_mah : 0;

    hal_uart_printf("%lu,%d,%d,%d,%d,%d,%d,"
                      "%ld,%ld,%ld,"
                      "%d,%d,%d,%d,%d,%d,"
                      "%ld,%ld,"
                      "%ld,%ld,"
                      "%u,%u,%u,"
                      "0x%02X,0x%02X,%u\r\n",
                      (unsigned long)timestamp,
                      bq->cell_voltage_mv[0],
                      bq->cell_voltage_mv[1],
                      bq->cell_voltage_mv[2],
                      bq->cell_voltage_mv[3],
                      bq->stack_voltage_mv,
                      bq->cell_delta_mv,
                      (long)bq->current_ma,
                      (long)bq->raw_current_ma,
                      (long)bq->power_mw,
                      bq->internal_temp_c,
                      bq->ts1_temp_c,
                      ntc1_temp,
                      ntc2_temp,
                      ntc3_temp,
                      ntc4_temp,
                      soc_x100,
                      soh_x100,
                      (long)passq_uah,
                      (long)remain_mah,
                      chg_on,
                      dsg_on,
                      bq->balance_active_cells,
                      bq->safety_status_a,
                      bq->safety_status_b,
                      bq->is_connected ? 1 : 0);
}

/**
 * @brief Send BMS_SW_OC_STATUS packet (ID=8) - software overcurrent protection state.
 */
static void debug_report_send_sw_oc_status(void)
{
    const sw_oc_state_t  *st  = sw_oc_protection_get_state();
    const sw_oc_config_t *cfg = sw_oc_protection_get_config();

    _link_bms_sw_oc_status_t pkt = {
        .timestamp_ms      = systime_milli(),
        .dsg_oc_counter    = st->dsg_counter,
        .chg_oc_counter    = st->chg_counter,
        .dsg_oc_trip_count = cfg->dsg_trip_count,
        .chg_oc_trip_count = cfg->chg_trip_count,
        .dsg_oc_tripped    = st->dsg_tripped,
        .chg_oc_tripped    = st->chg_tripped,
        .dsg_threshold_ma  = cfg->dsg_threshold_ma,
        .chg_threshold_ma  = cfg->chg_threshold_ma,
        .decay_rate        = cfg->decay_rate,
        ._pad0             = 0,
        .fast1_counter       = st->fast1_counter,
        .fast1_trip_count    = cfg->fast1_trip_count,
        .fast1_tripped       = st->fast1_tripped,
        .fast1_decay_rate    = cfg->fast1_decay_rate,
        .fast1_threshold_ma  = cfg->fast1_threshold_ma,
        .fast2_counter       = st->fast2_counter,
        .fast2_trip_count    = cfg->fast2_trip_count,
        .fast2_tripped       = st->fast2_tripped,
        .fast2_decay_rate    = cfg->fast2_decay_rate,
        .fast2_threshold_ma  = cfg->fast2_threshold_ma,
    };

    static uint8_t tx_buf_swoc[64];
    uint16_t len = link_encode_bms_sw_oc_status_pkt(tx_buf_swoc, LINK_ID_BMS, LINK_ID_APP, &pkt);
    hal_uart1_send(tx_buf_swoc, len);
}

/**
 * @brief Send BMS_EXT_TEMP packet (ID=10) - 4-channel NTC thermistor data.
 */
static void debug_report_send_ext_temp(void)
{
    const ext_temp_module_status_t *ext = ext_temp_adc_get_status();
    if (ext == NULL) return;

    const bq_battery_status_t *bq = bq_controller_get_status();
    uint8_t ow_mask = ext->open_wire_mask & 0x0F;
    if (bq->ts1_open_wire) {
        ow_mask |= (1u << 4);
    }

    _link_bms_ext_temp_t pkt = {
        .timestamp_ms = systime_milli(),
        .valid_mask = 0,
        .fault_mask = ext->fault_mask,
        .mosfet_disabled = ext->mosfet_disabled ? 1 : 0,
        .open_wire_mask = ow_mask,
    };

    for (int i = 0; i < 4; i++) {
        pkt.temp_c[i]  = (int8_t)ext->sensors[i].temp_c;
        pkt.adc_raw[i] = ext->sensors[i].adc_raw;
        if (ext->sensors[i].is_valid) {
            pkt.valid_mask |= (1u << i);
        }
    }

    static uint8_t tx_buf_et[64];
    uint16_t len = link_encode_bms_ext_temp_pkt(tx_buf_et, LINK_ID_BMS, LINK_ID_APP, &pkt);
    hal_uart1_send(tx_buf_et, len);
}

/**
 * @brief Print debug data in binary protocol binary format.
 */
static void debug_report_print_bp(void)
{
    const bq_battery_status_t *bq = bq_controller_get_status();
    const ext_temp_module_status_t *ext = ext_temp_adc_get_status();
    const battery_status_t *bat = battery_soc_soh_get_status();

    int8_t ext1_temp = (ext != NULL) ? (int8_t)ext->sensors[0].temp_c : 0;
    int8_t ext2_temp = (ext != NULL) ? (int8_t)ext->sensors[1].temp_c : 0;
    int8_t ext3_temp = (ext != NULL) ? (int8_t)ext->sensors[2].temp_c : 0;
    int8_t ext4_temp = (ext != NULL) ? (int8_t)ext->sensors[3].temp_c : 0;

    /* Get SOC/SOH as integer x100 (e.g., 85.25% -> 8525) */
    uint16_t soc_x100 = (bat != NULL && bat->initialized) ? (uint16_t)(bat->soc.soc_percent * 100) : 0;
    uint16_t soh_x100 = (bat != NULL && bat->initialized) ? (uint16_t)(bat->soh.soh_percent * 100) : 0;

    /* Extract FET status from battery_status register (bit2=DSG, bit3=CHG) */
    uint8_t chg_on = (bq->battery_status & BQ_BATT_STAT_CHG_FET) ? 1 : 0;
    uint8_t dsg_on = (bq->battery_status & BQ_BATT_STAT_DSG_FET) ? 1 : 0;

    _link_bms_status_t status = {
        .timestamp_ms = systime_milli(),
        .cell1_mv = bq->cell_voltage_mv[0],
        .cell2_mv = bq->cell_voltage_mv[1],
        .cell3_mv = bq->cell_voltage_mv[2],
        .cell4_mv = bq->cell_voltage_mv[3],
        .stack_mv = bq->stack_voltage_mv,
        .current_ma = bq->current_ma,
        .power_dw = bq->power_mw / 100,
        /* Clamp int16 -> int8: the TS1 open-wire sentinel (-999) must not
         * truncate to +25; clamped it reads -128 = clearly invalid. */
        .temp_internal = (int8_t)(bq->internal_temp_c > 127 ? 127 :
                                  (bq->internal_temp_c < -128 ? -128 : bq->internal_temp_c)),
        .temp_ts1 = (int8_t)(bq->ts1_temp_c > 127 ? 127 :
                             (bq->ts1_temp_c < -128 ? -128 : bq->ts1_temp_c)),
        .temp_ext1 = ext1_temp,
        .temp_ext2 = ext2_temp,
        .soc_x100 = soc_x100,
        .soh_x100 = soh_x100,
        .chg_fet_on = chg_on,
        .dsg_fet_on = dsg_on,
        .balance_active = bq->balance_active_cells,
        .status_flags = (uint8_t)(bq->battery_status & 0xFF),
        .safety_status_a = bq->safety_status_a,
        .safety_status_b = bq->safety_status_b,
        .temp_ext3 = ext3_temp,
        .temp_ext4 = ext4_temp,
        .raw_current_ma = bq->raw_current_ma};

    /* Encode packet and send via USART1 (binary protocol / BLE bridge) */
    uint16_t pkt_len = link_encode_bms_status_pkt(tx_buffer, LINK_ID_BMS, LINK_ID_PC, &status);
    hal_uart1_send(tx_buffer, pkt_len);

    /* All secondary packets go out every cycle; at the 600 ms interval there is
     * no UART burst concern. */
    debug_report_send_energy();
    debug_report_send_soc_soh();
    debug_report_send_fault_detail();
    debug_report_send_wire_fault();
    debug_report_send_mcu_status();
    debug_report_send_balance_debug();
    debug_report_send_sw_oc_status();
    debug_report_send_ext_temp();
    debug_report_send_ntc_fault();
    debug_report_send_soc_cell();
}

/**
 * @brief Send BMS_ENERGY packet (ID=4) - full-resolution current/power + coulomb counter.
 */
static void debug_report_send_energy(void)
{
    const bq_battery_status_t *bq = bq_controller_get_status();
    const battery_status_t *bat = battery_soc_soh_get_status();

    _link_bms_energy_t energy = {
        .timestamp_ms = systime_milli(),
        .current_ma = bq->current_ma,
        .power_mw = bq->power_mw,
        .passed_charge_uah = bq->passed_charge_uah,
        .dod0_uah = bq->dod0_uah,
        .remaining_capacity_mah = (bat != NULL && bat->initialized) ? bat->soc.remaining_capacity_mah : 0,
        .used_capacity_mah = (bat != NULL && bat->initialized) ? bat->soc.used_capacity_mah : 0,
        .passq_elapsed_time_sec = bq->passq_elapsed_time_sec};

    static uint8_t tx_buf_energy[64];
    uint16_t len = link_encode_bms_energy_pkt(tx_buf_energy, LINK_ID_BMS, LINK_ID_APP, &energy);
    hal_uart1_send(tx_buf_energy, len);
}

/**
 * @brief Send BMS_SOC_SOH packet (ID=5) - detailed SOC/SOH estimation state.
 */
static void debug_report_send_soc_soh(void)
{
    const battery_status_t *bat = battery_soc_soh_get_status();

    _link_bms_soc_soh_t soc_soh = {0};

    if (bat != NULL && bat->initialized) {
        soc_soh.soc_x100 = (uint16_t)(bat->soc.soc_percent * 100);
        soc_soh.soc_coulomb_x100 = (uint16_t)(bat->soc.soc_coulomb_percent * 100);
        soc_soh.soc_voltage_x100 = (uint16_t)(bat->soc.soc_voltage_percent * 100);
        soc_soh.soh_x100 = (uint16_t)(bat->soh.soh_percent * 100);
        soc_soh.actual_capacity_mah = bat->soh.actual_capacity_mah;
        soc_soh.nominal_capacity_mah = bat->soh.nominal_capacity_mah;
        soc_soh.capacity_fade_x100 = (uint16_t)(bat->soh.capacity_fade_percent * 100);
        soc_soh.coulombic_eff_x100 = (uint16_t)(bat->soh.coulombic_efficiency * 100);
        soc_soh.cycle_count = bat->soh.cycle_count;
        soc_soh.time_to_empty_min = bat->soc.time_to_empty_min;
        soc_soh.time_to_full_min = bat->soc.time_to_full_min;
        soc_soh.is_calibrated = bat->soc.is_calibrated ? 1 : 0;
        soc_soh.is_fully_charged = bat->soc.is_fully_charged ? 1 : 0;
        soc_soh.is_empty = bat->soc.is_empty ? 1 : 0;
        soc_soh.remaining_capacity_mah = bat->soc.remaining_capacity_mah;
        soc_soh.last_anchor_event = bat->soc.last_anchor_event;
        soc_soh.anchor_flags = bat->soc.anchor_flags;
        soc_soh.rest_duration_s = bat->soc.rest_duration_s;
        soc_soh.passq_jump_count = bat->soc.passq_jump_count;
        soc_soh.raw_coulomb_active = bat->soh.raw_coulomb_active ? 1 : 0;
        soc_soh.raw_coulomb_mah = bat->soh.raw_coulomb_mah;
        soc_soh.last_learned_capacity_mah = bat->soh.last_learned_capacity_mah;
    }

    static uint8_t tx_buf_soc[72];
    uint16_t len = link_encode_bms_soc_soh_pkt(tx_buf_soc, LINK_ID_BMS, LINK_ID_APP, &soc_soh);
    hal_uart1_send(tx_buf_soc, len);
}

/**
 * @brief Send BMS_SOC_CELL packet (ID=22) - SOC anchoring v2: per-cell SOC, grades, learning.
 */
static void debug_report_send_soc_cell(void)
{
    const battery_status_t *bat = battery_soc_soh_get_status();
    _link_bms_soc_cell_t p = {0};
    if (bat != NULL && bat->initialized) {
        p.timestamp_ms = systime_milli();
        for (int i = 0; i < 4; i++) {
            p.soc_cell_x100[i] = (uint16_t)(bat->soc.soc_cell_percent[i] * 100);
            p.cell_capacity_mah[i] = bat->soh.cell_capacity_mah[i];
            p.v_ocv_mv[i] = bat->soc.v_ocv_mv[i];
        }
        p.soc_pack_x100 = (uint16_t)(bat->soc.soc_percent * 100);
        p.soc_max_x100 = (uint16_t)(bat->soc.soc_max_percent * 100);
        float off_max = 0; for (int i = 0; i < 4; i++) if (bat->soc.cell_offset_pct[i] > off_max) off_max = bat->soc.cell_offset_pct[i];
        p.offset_max_x100 = (uint16_t)(off_max * 100);
        p.usable_capacity_mah = bat->soh.usable_capacity_mah;
        p.imbalance_loss_mah = bat->soh.imbalance_loss_mah;
        p.drift_at_anchor_x10 = (int16_t)(bat->soc.drift_at_anchor_pct * 10);
        p.reconcile_delta_x10 = (int16_t)(bat->soc.reconcile_delta_pct * 10);
        p.anchor_grade = bat->soc.anchor_grade;
        p.last_anchor_event = bat->soc.last_anchor_event;
        p.learn_count = bat->soh.learn_count;
        p.learn_refused_reason = bat->soh.learn_refused_reason;
        p.flags2 = bat->soc.anchor_flags2;
        uint32_t rm = bat->soc.rest_duration_s / 60u;
        p.rest_minutes = (uint8_t)(rm > 255 ? 255 : rm);
    }
    static uint8_t tx_buf_sc[80];
    uint16_t len = link_encode_bms_soc_cell_pkt(tx_buf_sc, LINK_ID_BMS, LINK_ID_APP, &p);
    hal_uart1_send(tx_buf_sc, len);
}

/**
 * @brief Send BMS_FAULT_DETAIL packet (ID=6) - full fault/alarm/health state.
 */
static void debug_report_send_fault_detail(void)
{
    const bq_battery_status_t *bq = bq_controller_get_status();
    const ext_temp_module_status_t *ext = ext_temp_adc_get_status();
    system_health_t health = system_health_check_all();

    /* Pack system_health_flags bitfield */
    uint8_t health_flags = 0;
    if (health.bq_healthy)         health_flags |= (1 << 0);
    if (health.ext_temp_healthy)   health_flags |= (1 << 1);
    if (health.soc_healthy)        health_flags |= (1 << 2);
    if (health.overall_healthy)    health_flags |= (1 << 3);
    if (health.has_voltage_fault)  health_flags |= (1 << 4);
    if (health.has_temp_fault)     health_flags |= (1 << 5);
    if (health.has_current_fault)  health_flags |= (1 << 6);
    if (health.has_critical_fault) health_flags |= (1 << 7);

    _link_bms_fault_detail_t fault = {
        .timestamp_ms = systime_milli(),
        .safety_alert_a = bq->safety_alert_a,
        .safety_status_a = bq->safety_status_a,
        .safety_alert_b = bq->safety_alert_b,
        .safety_status_b = bq->safety_status_b,
        .battery_status = bq->battery_status,
        .alarm_status = bq->alarm_status,
        .ext_temp_fault_mask = (ext != NULL) ? ext->fault_mask : 0,
        .system_health_flags = health_flags,
        .is_connected = bq->is_connected ? 1 : 0,
        .cuv_tripped = 0};

    static uint8_t tx_buf_fault[64];
    uint16_t len = link_encode_bms_fault_detail_pkt(tx_buf_fault, LINK_ID_BMS, LINK_ID_APP, &fault);
    hal_uart1_send(tx_buf_fault, len);
}

/**
 * @brief Send BMS_MCU_STATUS packet (ID=7) - MCU-side boolean flags.
 */
static void debug_report_send_mcu_status(void)
{
    const bq_battery_status_t *bq = bq_controller_get_status();
    uint32_t flags = 0;

    if (bq_controller_is_alert_active()) {
        flags |= MCU_FLAG_ALERT_ACTIVE;
    }

    _link_bms_mcu_status_t mcu_status = {
        .timestamp_ms = systime_milli(),
        .mcu_flags = flags,
        .config_verify_errors = bq->config_verify_errors,
    };

    static uint8_t tx_buf_mcu[64];
    uint16_t len = link_encode_bms_mcu_status_pkt(tx_buf_mcu, LINK_ID_BMS, LINK_ID_APP, &mcu_status);
    hal_uart1_send(tx_buf_mcu, len);
}

/**
 * @brief Send BMS_BALANCE_DEBUG packet (ID=21) - balancing state, block reasons, live config.
 */
static void debug_report_send_balance_debug(void)
{
    const bq_balance_debug_t *bd = bq_controller_get_balance_debug();

    _link_bms_balance_debug_t pkt = {
        .timestamp_ms        = systime_milli(),
        .enabled             = bd->enabled,
        .block_flags         = bd->block_flags,
        .cmd_mask            = bd->cmd_mask,
        .hw_active_mask      = bd->hw_active_mask,
        .alarm_cb            = bd->alarm_cb,
        .cfg_bal_config_reg  = bd->cfg_bal_config_reg,
        .tick_count          = bd->tick_count,
        .write_err_count     = bd->write_err_count,
        .last_write_status   = bd->last_write_status,
        .cfg_allow_adjacent  = bd->cfg_allow_adjacent,
        .vmin_mv             = bd->vmin_mv,
        .vmax_mv             = bd->vmax_mv,
        .delta_mv            = bd->delta_mv,
        .current_ma          = bd->current_ma,
        .internal_temp_c     = (int8_t)bd->internal_temp_c,
        .vmin_idx            = bd->vmin_idx,
        .vmax_idx            = bd->vmax_idx,
        .ncell               = bd->ncell,
        .cfg_start_mv        = bd->cfg_start_mv,
        .cfg_stop_mv         = bd->cfg_stop_mv,
        .cfg_min_cell_mv     = bd->cfg_min_cell_mv,
        .cfg_max_curr_ma     = bd->cfg_max_curr_ma,
        .cfg_max_temp_c      = bd->cfg_max_temp_c,
        .cfg_min_start_cell_mv = bd->cfg_min_start_cell_mv,
        .reserved            = {0}
    };

    static uint8_t tx_buf_bd[64];
    uint16_t len = link_encode_bms_balance_debug_pkt(tx_buf_bd, LINK_ID_BMS, LINK_ID_APP, &pkt);
    hal_uart1_send(tx_buf_bd, len);
}

/*---------------------------------------------------------------------------*/
/* Public Functions                                                          */
/*---------------------------------------------------------------------------*/

void debug_report_init(void)
{
    debug_enabled = true;
    debug_type = DEBUG_TYPE_BP;
    debug_interval_ms = DEBUG_REPORT_DEFAULT_INTERVAL_MS;
    last_report_time = 0;
    header_printed = false;
}

void debug_report_enable(void)
{
    debug_enabled = true;
    header_printed = false; /* Print header on next CSV report */
    last_report_time = systime_milli();
}

void debug_report_disable(void)
{
    debug_enabled = false;
    header_printed = false;
}

bool debug_report_toggle(void)
{
    if (debug_enabled)
    {
        debug_report_disable();
    }
    else
    {
        debug_report_enable();
    }
    return debug_enabled;
}

bool debug_report_is_enabled(void)
{
    return debug_enabled;
}

void debug_report_set_type(uint8_t type)
{
    if (type <= DEBUG_TYPE_BP)
    {
        debug_type = type;
        header_printed = false; /* Reset header flag when switching types */
    }
}

uint8_t debug_report_get_type(void)
{
    return debug_type;
}

int debug_report_set_interval(uint32_t interval_ms)
{
    if (interval_ms < DEBUG_REPORT_MIN_INTERVAL_MS ||
        interval_ms > DEBUG_REPORT_MAX_INTERVAL_MS)
    {
        return -1;
    }
    debug_interval_ms = interval_ms;
    return 0;
}

uint32_t debug_report_get_interval(void)
{
    return debug_interval_ms;
}

void debug_report_run(void)
{
    if (!debug_enabled)
    {
        return;
    }

    uint32_t now = systime_milli();
    if ((now - last_report_time) < debug_interval_ms)
    {
        return;
    }
    last_report_time = now;

    if (debug_type == DEBUG_TYPE_CSV && !header_printed)
    {
        debug_report_print_header_csv();
        header_printed = true;
    }

    debug_report_print_once();
}

void debug_report_print_once(void)
{
    /* Always send binary protocol (app needs telemetry) */
    debug_report_print_bp();

    /* CSV only in debug mode */
    if (debug_type == DEBUG_TYPE_CSV) {
        debug_report_print_csv();
    }
}

/*
 * Open-wire verdict and wire-fault response, packet 131. Sent every broadcast
 * cycle rather than only on a fault, so a capture has the healthy baseline to
 * compare against: sum-of-cells minus stack is only meaningful next to its normal
 * value.
 */
static void debug_report_send_wire_fault(void)
{
    static uint8_t tx_buf_wf[64];
    const bq_battery_status_t *bq = bq_controller_get_status();
    const wf_status_t *w = wire_fault_status();
    const ow_ctx_t *ow = bq_controller_open_wire_ctx();
    _link_bms_wire_fault_t pkt;
    uint16_t len;

    pkt.timestamp_ms        = systime_milli();
    pkt.ow_kind             = (uint8_t)ow->verdict.kind;
    pkt.ow_tap              = ow->verdict.tap;
    pkt.ow_cell             = ow->verdict.cell;
    pkt.ow_saturated        = ow->verdict.saturated ? 1u : 0u;
    pkt.ow_state            = (uint8_t)ow->state;
    pkt.ow_events           = ow->events;
    pkt.sum_minus_stack_mv  = ow->verdict.sum_minus_stack_mv;
    pkt.wf_state            = (uint8_t)w->state;
    pkt.wf_balancing_locked = w->balancing_locked ? 1u : 0u;
    pkt.wf_soc_suppressed   = w->soc_anchors_suppressed ? 1u : 0u;
    pkt.dsg_fet_map         = param_cache[PARAM_DSG_FET_PROTECTIONS_A].u8;
    pkt.wf_grace_left_ms    = w->grace_left_ms;
    pkt.wf_reapply_count    = w->reapply_count;
    pkt.wf_uv_sub_trips     = w->uv_substitute_trips;
    (void)bq;

    len = link_encode_bms_wire_fault_pkt(tx_buf_wf, LINK_ID_BMS, LINK_ID_APP, &pkt);
    hal_uart1_send(tx_buf_wf, len);
}

/*
 * MCU NTC band verdict and FET request, packet 132. Sent every broadcast cycle,
 * including while PARAM_NTC_SAFETY_EN is 0, so the bands can be watched against
 * real temperatures before the layer is allowed near the power path.
 */
static void debug_report_send_ntc_fault(void)
{
    static uint8_t tx_buf_nf[64];
    const ntc_fault_status_t *n = ntc_fault_status();
    const ext_temp_module_status_t *ext = ext_temp_adc_get_status();
    const bq_battery_status_t *bq = bq_controller_get_status();
    _link_bms_ntc_fault_t pkt;
    uint16_t len;

    pkt.timestamp_ms         = systime_milli();
    pkt.enabled              = n->enabled ? 1u : 0u;
    pkt.state                = (uint8_t)n->state;
    pkt.usable_battery_mask  = n->usable_battery_mask;
    pkt.open_wire_mask       = (uint8_t)((ext->open_wire_mask & 0x0Fu) |
                                         (bq->ts1_open_wire ? 0x10u : 0x00u));
    pkt.chg_band             = (uint8_t)n->chg_band;
    pkt.dsg_band             = (uint8_t)n->dsg_band;
    pkt.fet_band             = (uint8_t)n->fet_band;
    pkt.fet_temp_source      = !n->fet_temp_valid ? 0u : (n->ch4_open ? 2u : 1u);
    pkt.battery_min_c        = n->battery_min_c;
    pkt.battery_max_c        = n->battery_max_c;
    pkt.fet_temp_c           = n->fet_temp_c;
    pkt.want_fets_off        = n->want_fets_off;
    pkt.applied_fets_off     = n->applied_fets_off;
    pkt.fets_off_union       = bq_controller_fets_off_union();

    len = link_encode_bms_ntc_fault_pkt(tx_buf_nf, LINK_ID_BMS, LINK_ID_APP, &pkt);
    hal_uart1_send(tx_buf_nf, len);
}
