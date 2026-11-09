#include "sw_oc_protection.h"
#include "bq_controller.h"
#include "system_health.h"
#include "parameter.h"
#include "hal_uart.h"

static sw_oc_config_t g_cfg;

static sw_oc_state_t g_state;

void sw_oc_protection_init(void)
{
    g_state.dsg_counter = 0;
    g_state.chg_counter = 0;
    g_state.dsg_tripped = 0;
    g_state.chg_tripped = 0;
    g_state.fast1_counter = 0;
    g_state.fast1_tripped = 0;
    g_state.fast2_counter = 0;
    g_state.fast2_tripped = 0;
}

/* Helper: seconds → tick count using the compile-time tick interval */
static inline uint16_t secs_to_ticks(uint16_t time_s)
{
    return (uint16_t)((uint32_t)time_s * 1000 / SW_OC_TICK_MS);
}

void sw_oc_protection_update(int32_t raw_current_ma)
{
    /* Sync config from param_cache (live-adjustable via binary protocol) */
    g_cfg.dsg_threshold_ma  = (int32_t)param_cache[PARAM_SW_OC_DSG_THRESHOLD_A].u16 * 1000;
    g_cfg.chg_threshold_ma  = (int32_t)param_cache[PARAM_SW_OC_CHG_THRESHOLD_A].u16 * 1000;
    g_cfg.dsg_trip_count    = secs_to_ticks(param_cache[PARAM_SW_OC_DSG_TIME_S].u16);
    g_cfg.chg_trip_count    = secs_to_ticks(param_cache[PARAM_SW_OC_CHG_TIME_S].u16);
    g_cfg.decay_rate        = param_cache[PARAM_SW_OC_DECAY_RATE].u8;

    g_cfg.fast1_threshold_ma = (int32_t)param_cache[PARAM_SW_OC_FAST1_THRESHOLD_A].u16 * 1000;
    g_cfg.fast1_trip_count   = secs_to_ticks(param_cache[PARAM_SW_OC_FAST1_TIME_S].u16);
    g_cfg.fast1_decay_rate   = param_cache[PARAM_SW_OC_FAST1_DECAY_RATE].u8;

    g_cfg.fast2_threshold_ma = (int32_t)param_cache[PARAM_SW_OC_FAST2_THRESHOLD_A].u16 * 1000;
    g_cfg.fast2_trip_count   = secs_to_ticks(param_cache[PARAM_SW_OC_FAST2_TIME_S].u16);
    g_cfg.fast2_decay_rate   = param_cache[PARAM_SW_OC_FAST2_DECAY_RATE].u8;

    /* --- Slow: Discharge direction (current_ma is negative for discharge) --- */
    if (!g_state.dsg_tripped) {
        if (raw_current_ma < -(int32_t)g_cfg.dsg_threshold_ma) {
            g_state.dsg_counter++;
            if (g_state.dsg_counter >= g_cfg.dsg_trip_count) {
                g_state.dsg_counter = g_cfg.dsg_trip_count;
                g_state.dsg_tripped = 1;
                bq_controller_manual_shutdown();
                hal_uart_printf("[SW_OC] DSG overcurrent TRIP — FETs OFF\r\n");
            }
        } else {
            if (g_state.dsg_counter > g_cfg.decay_rate) {
                g_state.dsg_counter -= g_cfg.decay_rate;
            } else {
                g_state.dsg_counter = 0;
            }
        }
    }

    /* --- Slow: Charge direction (current_ma is positive for charge) --- */
    if (!g_state.chg_tripped) {
        if (raw_current_ma > (int32_t)g_cfg.chg_threshold_ma) {
            g_state.chg_counter++;
            if (g_state.chg_counter >= g_cfg.chg_trip_count) {
                g_state.chg_counter = g_cfg.chg_trip_count;
                g_state.chg_tripped = 1;
                bq_controller_manual_shutdown();
                hal_uart_printf("[SW_OC] CHG overcurrent TRIP — FETs OFF\r\n");
            }
        } else {
            if (g_state.chg_counter > g_cfg.decay_rate) {
                g_state.chg_counter -= g_cfg.decay_rate;
            } else {
                g_state.chg_counter = 0;
            }
        }
    }

    /* --- Fast1: DSG only --- */
    if (!g_state.fast1_tripped) {
        if (raw_current_ma < -(int32_t)g_cfg.fast1_threshold_ma) {
            g_state.fast1_counter++;
            if (g_state.fast1_counter >= g_cfg.fast1_trip_count) {
                g_state.fast1_counter = g_cfg.fast1_trip_count;
                g_state.fast1_tripped = 1;
                bq_controller_manual_shutdown();
                hal_uart_printf("[SW_OC] FAST1 DSG overcurrent TRIP — FETs OFF\r\n");
            }
        } else {
            if (g_state.fast1_counter > g_cfg.fast1_decay_rate) {
                g_state.fast1_counter -= g_cfg.fast1_decay_rate;
            } else {
                g_state.fast1_counter = 0;
            }
        }
    }

    /* --- Fast2: DSG only --- */
    if (!g_state.fast2_tripped) {
        if (raw_current_ma < -(int32_t)g_cfg.fast2_threshold_ma) {
            g_state.fast2_counter++;
            if (g_state.fast2_counter >= g_cfg.fast2_trip_count) {
                g_state.fast2_counter = g_cfg.fast2_trip_count;
                g_state.fast2_tripped = 1;
                bq_controller_manual_shutdown();
                hal_uart_printf("[SW_OC] FAST2 DSG overcurrent TRIP — FETs OFF\r\n");
            }
        } else {
            if (g_state.fast2_counter > g_cfg.fast2_decay_rate) {
                g_state.fast2_counter -= g_cfg.fast2_decay_rate;
            } else {
                g_state.fast2_counter = 0;
            }
        }
    }
}

const sw_oc_state_t* sw_oc_protection_get_state(void)
{
    return &g_state;
}

const sw_oc_config_t* sw_oc_protection_get_config(void)
{
    return &g_cfg;
}

void sw_oc_protection_clear(void)
{
    uint8_t was_tripped = g_state.dsg_tripped || g_state.chg_tripped
                       || g_state.fast1_tripped || g_state.fast2_tripped;
    g_state.dsg_counter = 0;
    g_state.chg_counter = 0;
    g_state.dsg_tripped = 0;
    g_state.chg_tripped = 0;
    g_state.fast1_counter = 0;
    g_state.fast1_tripped = 0;
    g_state.fast2_counter = 0;
    g_state.fast2_tripped = 0;
    if (was_tripped) {
        if (bq_controller_has_fault()) {
            hal_uart_printf("[SW_OC] Cleared but BQ fault active — FETs stay OFF\r\n");
        } else if (!system_health_can_recover()) {
            hal_uart_printf("[SW_OC] Cleared but conditions unsafe — FETs stay OFF\r\n");
        } else {
            bq_controller_restore_autonomous();
            hal_uart_printf("[SW_OC] Cleared — FETs restored\r\n");
        }
    }
}

bool sw_oc_protection_is_tripped(void)
{
    return g_state.dsg_tripped || g_state.chg_tripped
        || g_state.fast1_tripped || g_state.fast2_tripped;
}
