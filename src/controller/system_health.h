/**
 * @file system_health.h
 * @brief System health monitoring and recovery decisions for all sensors:
 *        BQ7690x (voltages, current, internal temp, TS1), the external
 *        temperature ADC, and the SOC/SOH module.
 */

#pragma once
#include <stdint.h>
#include <stdbool.h>

/* ========== System Health Status ========== */

/**
 * @brief System health status structure
 */
typedef struct {
    // Module health
    bool bq_healthy;              // BQ7690x communication and no faults
    bool ext_temp_healthy;        // External temp sensors OK
    bool soc_healthy;             // SOC module OK (optional check)
    bool overall_healthy;         // All systems healthy

    // Fault categorization
    bool has_voltage_fault;       // Cell voltages outside safe range
    bool has_temp_fault;          // Temperature outside safe range (any sensor)
    bool has_current_fault;       // Current too high for recovery
    bool has_critical_fault;      // Hardware fault requiring manual intervention
} system_health_t;

/* ========== Health Check Functions ========== */

/**
 * @brief Check overall system health
 * @return System health status structure
 */
system_health_t system_health_check_all(void);

/**
 * @brief Check if system can auto-recover from faults
 * Verifies all conditions are safe for automatic recovery
 * @return true if safe to recover, false otherwise
 */
bool system_health_can_recover(void);

/**
 * @brief Execute system-wide recovery
 * Clears faults in all modules (BQ, external temp, etc.)
 * @return 0 on success, -1 on failure
 */
int system_health_execute_recovery(void);

/* ========== Individual Condition Checks ========== */

/**
 * @brief Check if cell voltages are safe for recovery
 * Uses margins from config: UV + recovery + 50mV, OV - recovery - 50mV
 * @return true if all cells within safe range
 */
bool system_health_is_voltage_safe(void);

/**
 * @brief Check if temperatures are safe for recovery
 * Checks BQ TS1 + External Temp sensors
 * Uses margins from config: UTC/OTC/UTD/OTD + recovery + 5°C
 * @return true if all temperatures within safe range
 */
bool system_health_is_temperature_safe(void);

/**
 * @brief Check if current is safe for recovery
 * Current should be low during fault condition
 * @return true if |current| < 5A
 */
bool system_health_is_current_safe(void);

/**
 * @brief Check if fault requires manual intervention
 * Critical faults: REGOUT, VREF, VSS, HWD (watchdog)
 * @return true if manual intervention required
 */
bool system_health_requires_manual_intervention(void);

/* ========== Print Functions ========== */

/**
 * @brief Print detailed system health report
 * Shows status of all modules and fault conditions
 */
void system_health_print_status(void);
