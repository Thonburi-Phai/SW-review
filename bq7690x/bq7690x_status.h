/**
 * @file bq7690x_status.h
 * @brief BQ7690x safety alert, battery status and alarm registers.
 */

#pragma once
#include "bq7690x.h"
#include "bq7690x_reg.h"

/* ========== SafetyAlertA Bitmasks (0x02) ========== */

/** SafetyAlertA (0x02) bits - SLUUCJ0 Table 6-2; bits 1-0 reserved */

#define BQ_SAFETY_ALERT_A_COV        (1u << 7)  // Cell Overvoltage Alert
#define BQ_SAFETY_ALERT_A_CUV        (1u << 6)  // Cell Undervoltage Alert
#define BQ_SAFETY_ALERT_A_SCD        (1u << 5)  // Short Circuit in Discharge Alert
#define BQ_SAFETY_ALERT_A_OCD1       (1u << 4)  // Overcurrent in Discharge 1 Alert
#define BQ_SAFETY_ALERT_A_OCD2       (1u << 3)  // Overcurrent in Discharge 2 Alert
#define BQ_SAFETY_ALERT_A_OCC        (1u << 2)  // Overcurrent in Charge Alert
// Bits 1-0: Reserved

/* ========== SafetyStatusA Bitmasks (0x03) ========== */

/** SafetyStatusA (0x03) bits - SLUUCJ0 Table 6-3 */

#define BQ_SAFETY_STATUS_A_COV        (1u << 7)  // Cell Overvoltage Fault
#define BQ_SAFETY_STATUS_A_CUV        (1u << 6)  // Cell Undervoltage Fault
#define BQ_SAFETY_STATUS_A_SCD        (1u << 5)  // Short Circuit in Discharge Fault
#define BQ_SAFETY_STATUS_A_OCD1       (1u << 4)  // Overcurrent in Discharge 1 Fault
#define BQ_SAFETY_STATUS_A_OCD2       (1u << 3)  // Overcurrent in Discharge 2 Fault
#define BQ_SAFETY_STATUS_A_OCC        (1u << 2)  // Overcurrent in Charge Fault
#define BQ_SAFETY_STATUS_A_CURLATCH   (1u << 1)  // Current Protection Latch Fault
#define BQ_SAFETY_STATUS_A_REGOUT     (1u << 0)  // REGOUT Fault


/* ========== SafetyAlertB Bitmasks (0x04) ========== */

/** SafetyAlertB (0x04) bits - SLUUCJ0 Table 6-4 */

#define BQ_SAFETY_ALERT_B_OTD        (1u << 7)  // Overtemperature in Discharge Alert
#define BQ_SAFETY_ALERT_B_OTC        (1u << 6)  // Overtemperature in Charge Alert
#define BQ_SAFETY_ALERT_B_UTD        (1u << 5)  // Undertemperature in Discharge Alert
#define BQ_SAFETY_ALERT_B_UTC        (1u << 4)  // Undertemperature in Charge Alert
#define BQ_SAFETY_ALERT_B_OTINT      (1u << 3)  // Internal Overtemperature Alert
#define BQ_SAFETY_ALERT_B_HWD        (1u << 2)  // Host Watchdog Alert
#define BQ_SAFETY_ALERT_B_VREF       (1u << 1)  // VREF Measurement Diagnostic Alert
#define BQ_SAFETY_ALERT_B_VSS        (1u << 0)  // VSS Measurement Diagnostic Alert

/* ========== SafetyStatusB Bitmasks (0x05) ========== */

/** SafetyStatusB (0x05) bits - SLUUCJ0 Table 6-5 */

#define BQ_SAFETY_STATUS_B_OTD        (1u << 7)  // Overtemperature in Discharge Fault
#define BQ_SAFETY_STATUS_B_OTC        (1u << 6)  // Overtemperature in Charge Fault
#define BQ_SAFETY_STATUS_B_UTD        (1u << 5)  // Undertemperature in Discharge Fault
#define BQ_SAFETY_STATUS_B_UTC        (1u << 4)  // Undertemperature in Charge Fault
#define BQ_SAFETY_STATUS_B_OTINT      (1u << 3)  // Internal Overtemperature Fault
#define BQ_SAFETY_STATUS_B_HWD        (1u << 2)  // Host Watchdog Fault
#define BQ_SAFETY_STATUS_B_VREF       (1u << 1)  // VREF Measurement Diagnostic Fault
#define BQ_SAFETY_STATUS_B_VSS        (1u << 0)  // VSS Measurement Diagnostic Fault


/* ========== BatteryStatus Bitmasks ========== */

/** BatteryStatus (0x12) bits, 16-bit - SLUUCJ0 Table 7-6 */

#define BQ_BATT_STAT_CHGDET       (1u << 1)   // Charge detector status
#define BQ_BATT_STAT_DSG_FET      (1u << 2)   // Discharge FET status (1=ON)
#define BQ_BATT_STAT_CHG_FET      (1u << 3)   // Charge FET status (1=ON)
#define BQ_BATT_STAT_ALERTPIN     (1u << 4)   // ALERT pin status
#define BQ_BATT_STAT_CFGUPDATE    (1u << 5)   // CONFIG_UPDATE mode (1=active)
#define BQ_BATT_STAT_SLEEP_EN     (1u << 6)   // Sleep mode enabled
#define BQ_BATT_STAT_POR          (1u << 7)   // Power-on reset occurred
#define BQ_BATT_STAT_FET_EN       (1u << 8)   // FET control enabled
#define BQ_BATT_STAT_SEC0         (1u << 10)  // Security mode bit 0
#define BQ_BATT_STAT_SEC1         (1u << 11)  // Security mode bit 1
#define BQ_BATT_STAT_SS           (1u << 12)  // Safety Status (any fault active)
#define BQ_BATT_STAT_SA           (1u << 13)  // Safety Alert (any alert active)
#define BQ_BATT_STAT_DEEPSLEEP    (1u << 14)  // Deep sleep mode
#define BQ_BATT_STAT_SLEEP        (1u << 15)  // Sleep mode (1=SLEEP)

// Security mode, bits 11:10 (SEC1:SEC0)
// TRM encoding: 01 = FULLACCESS, 11 = SEALED; no UNSEALED mode on BQ7690x
#define BQ_BATT_STAT_SEC_MASK       (0x3u << 10)  // Security mode mask
#define BQ_BATT_STAT_SEC_POR        (0x0u << 10)  // After POR (transitional)
#define BQ_BATT_STAT_SEC_FULLACCESS (0x1u << 10)  // Full Access mode (TI factory default)
#define BQ_BATT_STAT_SEC_SEALED     (0x3u << 10)  // Sealed mode


/* ========== AlarmStatus Bitmasks ========== */

/** AlarmStatus (0x62) latched bits - SLUUCJ0 Table 11-6 */

#define BQ_ALARM_SSA              (1u << 15)  // Safety Status A latched (any bit in SafetyStatusA)
#define BQ_ALARM_SSB              (1u << 14)  // Safety Status B latched (any bit in SafetyStatusB)
#define BQ_ALARM_SAA              (1u << 13)  // Safety Alert A latched (any bit in SafetyAlertA)
#define BQ_ALARM_SAB              (1u << 12)  // Safety Alert B latched (any bit in SafetyAlertB)
#define BQ_ALARM_XCHG             (1u << 11)  // CHG FET disabled
#define BQ_ALARM_XDSG             (1u << 10)  // DSG FET disabled
#define BQ_ALARM_SHUTV            (1u << 9)   // Shutdown voltage triggered
#define BQ_ALARM_CB               (1u << 8)   // Cell balancing active
#define BQ_ALARM_FULLSCAN         (1u << 7)   // Full scan complete
#define BQ_ALARM_ADSCAN           (1u << 6)   // ADC scan complete
#define BQ_ALARM_WAKE             (1u << 5)   // Wake from SLEEP mode
#define BQ_ALARM_SLEEP            (1u << 4)   // Enter SLEEP mode
#define BQ_ALARM_TIMER            (1u << 3)   // Timer alarm expired
#define BQ_ALARM_INITCOMP         (1u << 2)   // Initialization complete
#define BQ_ALARM_CDTOGGLE         (1u << 1)   // Charge detector toggle
#define BQ_ALARM_POR              (1u << 0)   // Power-on reset


/* ========== Safety Alert Functions ========== */

/**
 * @brief Read SafetyAlertA register
 * @param dev Device handle
 * @param alert Output: alert flags (BQ_SAFETY_ALERT_A_* / BQ_SAFETY_STATUS_A_* masks)
 * @return BQ_OK or error code
 */
bq_status_t bq7690x_read_safety_alert_a(bq7690x_t *dev, uint8_t *alert);

/**
 * @brief Read SafetyStatusA register
 * @param dev Device handle
 * @param status Output: status flags (BQ_SAFETY_ALERT_A_* / BQ_SAFETY_STATUS_A_* masks)
 * @return BQ_OK or error code
 */
bq_status_t bq7690x_read_safety_status_a(bq7690x_t *dev, uint8_t *status);

/**
 * @brief Read SafetyAlertB register
 * @param dev Device handle
 * @param alert Output: alert flags (BQ_SAFETY_ALERT_B_* / BQ_SAFETY_STATUS_B_* masks)
 * @return BQ_OK or error code
 */
bq_status_t bq7690x_read_safety_alert_b(bq7690x_t *dev, uint8_t *alert);

/**
 * @brief Read SafetyStatusB register
 * @param dev Device handle
 * @param status Output: status flags (BQ_SAFETY_ALERT_B_* / BQ_SAFETY_STATUS_B_* masks)
 * @return BQ_OK or error code
 */
bq_status_t bq7690x_read_safety_status_b(bq7690x_t *dev, uint8_t *status);

/* ========== Battery Status Functions ========== */

/**
 * @brief Read BatteryStatus register
 * @param dev Device handle
 * @param battery_status Output: battery status flags (BQ_BATT_STAT_* masks)
 * @return BQ_OK or error code
 */
bq_status_t bq7690x_read_battery_status(bq7690x_t *dev, uint16_t *battery_status);

/* ========== Alarm Status Functions ========== */

/**
 * @brief Read AlarmStatus register (latched)
 * @param dev Device handle
 * @param alarm_status Output: alarm status flags (BQ_ALARM_* masks)
 * @return BQ_OK or error code
 */
bq_status_t bq7690x_read_alarm_status(bq7690x_t *dev, uint16_t *alarm_status);

/**
 * @brief Write AlarmStatus register (clear latched alarms)
 * @param dev Device handle
 * @param alarm_status Alarm bits to write
 * @return BQ_OK or error code
 */
bq_status_t bq7690x_write_alarm_status(bq7690x_t *dev, uint16_t alarm_status);

/**
 * @brief Read AlarmRawStatus register (unlatched)
 * @param dev Device handle
 * @param alarm_raw_status Output: raw alarm status (BQ_ALARM_* masks)
 * @return BQ_OK or error code
 */
bq_status_t bq7690x_read_alarm_raw_status(bq7690x_t *dev, uint16_t *alarm_raw_status);

/**
 * @brief Read AlarmEnable register
 * @param dev Device handle
 * @param alarm_enable Output: alarm enable mask
 * @return BQ_OK or error code
 */
bq_status_t bq7690x_read_alarm_enable(bq7690x_t *dev, uint16_t *alarm_enable);

/**
 * @brief Write AlarmEnable register
 * @param dev Device handle
 * @param alarm_enable Alarm enable mask (BQ_ALARM_* masks)
 * @return BQ_OK or error code
 */
bq_status_t bq7690x_write_alarm_enable(bq7690x_t *dev, uint16_t alarm_enable);
