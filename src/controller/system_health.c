/**
 * @file system_health.c
 * @brief System health monitoring and recovery decisions.
 */

#include "system_health.h"
#include "bq_controller.h"
#include "external_temp_adc.h"
#include "battery_soc_soh.h"
#include "bq_config_4s_lfp.h"
#include "hal_uart.h"
#include <stdlib.h>  // For abs()

/* ========== Configuration ========== */

#define RECOVERY_VOLTAGE_MARGIN_MV  0    // Rely on IC's own recovery hysteresis
#define RECOVERY_TEMP_MARGIN_C      5    // Extra margin beyond recovery threshold
#define RECOVERY_CURRENT_LIMIT_MA   5000 // Max current during recovery (5A)

/* ========== Individual Condition Checks ========== */

bool system_health_is_voltage_safe(void)
{
    // COV and CUV recovery are handled entirely by the BQ IC autonomous
    // mechanism (threshold ± hysteresis). MCU must NOT send PROT_RECOVERY
    // for voltage faults — it conflicts with the IC's delay counter and
    // hysteresis logic, causing premature recovery.
    (void)0;
    return true;
}

bool system_health_is_temperature_safe(void)
{
    const bq_battery_status_t *bq_status = bq_controller_get_status();

    // 1. Check BQ TS1 temperature
    if (bq_status->ts1_temp_c != -999) {  // Valid sensor reading
        // Recovery values are absolute temperatures, not offsets.
        // Safe range: recovery temp + margin on each side.
        const int16_t utc_safe = CFG_UTC_RECOVERY_C + RECOVERY_TEMP_MARGIN_C;
        const int16_t otc_safe = CFG_OTC_RECOVERY_C - RECOVERY_TEMP_MARGIN_C;
        const int16_t utd_safe = CFG_UTD_RECOVERY_C + RECOVERY_TEMP_MARGIN_C;
        const int16_t otd_safe = CFG_OTD_RECOVERY_C - RECOVERY_TEMP_MARGIN_C;

        int16_t temp = bq_status->ts1_temp_c;

        // Must be within both charge and discharge safe ranges
        if (temp < utd_safe || temp > otd_safe) {
            return false;
        }
        if (temp < utc_safe || temp > otc_safe) {
            return false;
        }
    }

    // 2. External temp sensors - don't block BQ recovery
    // (ext_temp faults are handled separately in recovery)

    return true;
}

bool system_health_is_current_safe(void)
{
    const bq_battery_status_t *status = bq_controller_get_status();

    // Get absolute current value
    int32_t abs_current = (status->current_ma < 0) ?
                          -status->current_ma :
                          status->current_ma;

    // Current must be low during fault condition
    return (abs_current < RECOVERY_CURRENT_LIMIT_MA);
}

bool system_health_requires_manual_intervention(void)
{
    const bq_battery_status_t *status = bq_controller_get_status();

    // Critical hardware faults that should NOT auto-recover

    // REGOUT fault (regulator fault)
    if (status->safety_status_a & 0x01) {
        return true;
    }

    // VREF or VSS fault (reference voltage fault)
    if (status->safety_status_b & (0x04 | 0x08)) {
        return true;
    }

    // HWD fault (watchdog timeout)
    if (status->safety_status_b & 0x80) {
        return true;
    }

    return false;
}

/* ========== System Health Check ========== */

system_health_t system_health_check_all(void)
{
    system_health_t health = {0};

    // Check individual modules
    health.bq_healthy = !bq_controller_has_fault();
    health.ext_temp_healthy = !ext_temp_adc_has_fault();
    health.soc_healthy = true;  // SOC module doesn't have fault flags

    // Overall health (all modules must be healthy)
    health.overall_healthy = health.bq_healthy &&
                             health.ext_temp_healthy &&
                             health.soc_healthy;

    // Categorize faults for recovery decision
    health.has_voltage_fault = !system_health_is_voltage_safe();
    health.has_temp_fault = !system_health_is_temperature_safe();
    health.has_current_fault = !system_health_is_current_safe();
    health.has_critical_fault = system_health_requires_manual_intervention();

    return health;
}

bool system_health_can_recover(void)
{
    system_health_t health = system_health_check_all();

    // Cannot recover if any unsafe condition exists
    if (health.has_critical_fault) {
        hal_uart_printf("[RECOVERY] Critical fault - manual intervention required\r\n");
        return false;
    }

    if (health.has_voltage_fault) {
        hal_uart_printf("[RECOVERY] Voltage still unsafe\r\n");
        return false;
    }

    if (health.has_temp_fault) {
        hal_uart_printf("[RECOVERY] Temperature still unsafe\r\n");
        return false;
    }

    if (health.has_current_fault) {
        hal_uart_printf("[RECOVERY] Current too high\r\n");
        return false;
    }

    // All conditions safe for recovery
    return true;
}

int system_health_execute_recovery(void)
{
    hal_uart_printf("\r\n[SYSTEM RECOVERY] Checking conditions...\r\n");

    // Check if recovery is safe
    if (!system_health_can_recover()) {
        hal_uart_printf("[SYSTEM RECOVERY] Conditions not safe - aborting\r\n");
        return -1;
    }

    hal_uart_printf("[SYSTEM RECOVERY] Conditions safe - executing recovery...\r\n");

    int result = 0;

    // Clear BQ faults
    if (bq_controller_has_fault()) {
        hal_uart_printf("[SYSTEM RECOVERY] Clearing BQ faults...\r\n");
        if (bq_controller_clear_faults() != 0) {
            hal_uart_printf("[ERROR] Failed to clear BQ faults\r\n");
            result = -1;
        } else {
            hal_uart_printf("[OK] BQ faults cleared\r\n");
        }
    }

    // Clear external temp faults
    if (ext_temp_adc_has_fault()) {
        hal_uart_printf("[SYSTEM RECOVERY] Clearing external temp faults...\r\n");
        if (ext_temp_adc_clear_faults() != 0) {
            hal_uart_printf("[ERROR] Failed to clear external temp faults\r\n");
            result = -1;
        } else {
            hal_uart_printf("[OK] External temp faults cleared\r\n");
        }
    }

    if (result == 0) {
        hal_uart_printf("[SYSTEM RECOVERY] SUCCESS - All faults cleared\r\n\r\n");
    } else {
        hal_uart_printf("[SYSTEM RECOVERY] FAILED - Some faults remain\r\n\r\n");
    }

    return result;
}

/* ========== Print Functions ========== */

void system_health_print_status(void)
{
    system_health_t health = system_health_check_all();

    hal_uart_printf("\r\n");
    hal_uart_printf("========================================\r\n");
    hal_uart_printf("       SYSTEM HEALTH STATUS\r\n");
    hal_uart_printf("========================================\r\n");

    // Module health
    hal_uart_printf("Module Status:\r\n");
    hal_uart_printf("  BQ7690x:       %s\r\n", health.bq_healthy ? "OK" : "FAULT");
    hal_uart_printf("  External Temp: %s\r\n", health.ext_temp_healthy ? "OK" : "FAULT");
    hal_uart_printf("  SOC/SOH:       %s\r\n", health.soc_healthy ? "OK" : "FAULT");
    hal_uart_printf("  Overall:       %s\r\n", health.overall_healthy ? "HEALTHY" : "UNHEALTHY");
    hal_uart_printf("\r\n");

    // Recovery conditions
    hal_uart_printf("Recovery Conditions:\r\n");
    hal_uart_printf("  Voltage Safe:  %s\r\n", health.has_voltage_fault ? "NO" : "YES");
    hal_uart_printf("  Temp Safe:     %s\r\n", health.has_temp_fault ? "NO" : "YES");
    hal_uart_printf("  Current Safe:  %s\r\n", health.has_current_fault ? "NO" : "YES");
    hal_uart_printf("  Critical Flt:  %s\r\n", health.has_critical_fault ? "YES" : "NO");
    hal_uart_printf("\r\n");

    // Recovery decision
    bool can_recover = system_health_can_recover();
    hal_uart_printf("Can Auto-Recover: %s\r\n", can_recover ? "YES" : "NO");
    hal_uart_printf("========================================\r\n");
    hal_uart_printf("\r\n");
}
