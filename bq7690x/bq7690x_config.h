/**
 * @file bq7690x_config.h
 * @brief BQ7690x Settings:Configuration data memory registers.
 */

#pragma once
#include "bq7690x.h"
#include "bq7690x_reg.h"

/* ========== Power Configuration (0x9014) ========== */

/**
 * Read PowerConfig register (H1)
 * @param dev Device handle
 * @param value Pointer to receive configuration byte
 */
bq_status_t bq7690x_read_power_config(bq7690x_t *dev, uint8_t *value);

/**
 * Write PowerConfig register (H1)
 * @param dev Device handle
 * @param value Configuration byte
 */
bq_status_t bq7690x_write_power_config(bq7690x_t *dev, uint8_t value);

/* ========== REGOUT Configuration (0x9015) ========== */

/**
 * Read REGOUTConfig register (H1)
 * @param dev Device handle
 * @param value Pointer to receive configuration byte
 */
bq_status_t bq7690x_read_regout_config(bq7690x_t *dev, uint8_t *value);

/**
 * Write REGOUTConfig register (H1)
 * @param dev Device handle
 * @param value Configuration byte
 */
bq_status_t bq7690x_write_regout_config(bq7690x_t *dev, uint8_t value);

/* ========== I2C Configuration (0x9016-0x9017) ========== */

/**
 * Read I2C address setting (H1)
 * @param dev Device handle
 * @param addr Pointer to receive 7-bit I2C address
 */
bq_status_t bq7690x_read_i2c_address(bq7690x_t *dev, uint8_t *addr);

/**
 * Write I2C address setting (H1); changes the device I2C address
 * @param dev Device handle
 * @param addr New 7-bit I2C address
 */
bq_status_t bq7690x_write_i2c_address(bq7690x_t *dev, uint8_t addr);

/**
 * Read I2C configuration register (H2)
 * @param dev Device handle
 * @param config Pointer to receive configuration word
 */
bq_status_t bq7690x_read_i2c_config(bq7690x_t *dev, uint16_t *config);

/**
 * Write I2C configuration register (H2)
 * @param dev Device handle
 * @param config Configuration word
 */
bq_status_t bq7690x_write_i2c_config(bq7690x_t *dev, uint16_t config);

/* ========== DA Configuration (0x9019) ========== */

/**
 * Read DAConfig register (H2)
 * @param dev Device handle
 * @param config Pointer to receive configuration word
 */
bq_status_t bq7690x_read_da_config(bq7690x_t *dev, uint16_t *config);

/**
 * Write DAConfig register (H2)
 * @param dev Device handle
 * @param config Configuration word
 */
bq_status_t bq7690x_write_da_config(bq7690x_t *dev, uint16_t config);

/* ========== Vcell Mode (0x901B) ========== */

/**
 * Read VcellMode - number of series cells (H1)
 * @param dev Device handle
 * @param cells Pointer to receive cell count (3-7)
 */
bq_status_t bq7690x_read_vcell_mode(bq7690x_t *dev, uint8_t *cells);

/**
 * Write VcellMode - set number of series cells (H1)
 * @param dev Device handle
 * @param cells Cell count (3-7 for BQ76905/907)
 */
bq_status_t bq7690x_write_vcell_mode(bq7690x_t *dev, uint8_t cells);

/* ========== Default Alarm Mask (0x901C) ========== */

/**
 * Read DefaultAlarmMask (H2)
 * @param dev Device handle
 * @param mask Pointer to receive alarm mask
 */
bq_status_t bq7690x_read_default_alarm_mask(bq7690x_t *dev, uint16_t *mask);

/**
 * Write DefaultAlarmMask (H2)
 * @param dev Device handle
 * @param mask Alarm mask configuration
 */
bq_status_t bq7690x_write_default_alarm_mask(bq7690x_t *dev, uint16_t mask);

/* ========== FET Options (0x901E) ========== */

/* Bit masks for FET Options (Table 12-8 in TRM) */
#define BQ_FETOPT_PROTRCVR    (1u << 0)  /* Allow PROT_RECOVERY() in SEALED mode */
#define BQ_FETOPT_PWM_EN      (1u << 1)  /* Enable PWM commands */
#define BQ_FETOPT_FET_EN      (1u << 2)  /* Autonomous FET enable default */
#define BQ_FETOPT_SFET        (1u << 3)  /* Series FET mode (body diode prot.) */
#define BQ_FETOPT_SLEEPCHG    (1u << 4)  /* Allow CHG in SLEEP */
#define BQ_FETOPT_HOST_FETON  (1u << 5)  /* Allow host force ON */
#define BQ_FETOPT_HOST_FETOFF (1u << 6)  /* Allow host force OFF */
#define BQ_FETOPT_CHGDETEN    (1u << 7)  /* Enable CHG detector block */

/**
 * Read FETOptions register (H1)
 * @param dev Device handle
 * @param options Pointer to receive FET options bitmask
 */
bq_status_t bq7690x_read_fet_options(bq7690x_t *dev, uint8_t *options);

/**
 * Write FETOptions register (H1)
 * @param dev Device handle
 * @param options FET options bitmask
 */
bq_status_t bq7690x_write_fet_options(bq7690x_t *dev, uint8_t options);

/* ========== Charge Detector Time (0x901F) ========== */

/**
 * Read ChargeDetectorTime (U1)
 * @param dev Device handle
 * @param time_100ms Pointer to receive the register value
 */
bq_status_t bq7690x_read_charge_detector_time(bq7690x_t *dev, uint8_t *time_100ms);

/**
 * Write ChargeDetectorTime (U1)
 * @param dev Device handle
 * @param time_100ms Register value
 */
bq_status_t bq7690x_write_charge_detector_time(bq7690x_t *dev, uint8_t time_100ms);

/* ========== Balancing Configuration (0x9020) ========== */

/**
 * Read BalancingConfiguration (H1)
 * Bit 0: CB_NO_CMD (balancing command mode)
 * @param dev Device handle
 * @param config Pointer to receive configuration byte
 */
bq_status_t bq7690x_read_balancing_configuration(bq7690x_t *dev, uint8_t *config);

/**
 * Write BalancingConfiguration (H1)
 * @param dev Device handle
 * @param config Configuration byte
 */
bq_status_t bq7690x_write_balancing_configuration(bq7690x_t *dev, uint8_t config);

/* ========== Balancing Temperature Thresholds (0x9021-0x9023) ========== */

/**
 * Read MinTempThreshold for cell balancing (U1)
 * @param dev Device handle
 * @param adc_code Pointer to receive the threshold register value
 */
bq_status_t bq7690x_read_min_temp_threshold(bq7690x_t *dev, uint8_t *adc_code);

/**
 * Write MinTempThreshold for cell balancing (U1)
 * @param dev Device handle
 * @param adc_code Threshold register value
 */
bq_status_t bq7690x_write_min_temp_threshold(bq7690x_t *dev, uint8_t adc_code);

/**
 * Read MaxTempThreshold for cell balancing (U1)
 * @param dev Device handle
 * @param adc_code Pointer to receive the threshold register value
 */
bq_status_t bq7690x_read_max_temp_threshold(bq7690x_t *dev, uint8_t *adc_code);

/**
 * Write MaxTempThreshold for cell balancing (U1)
 * @param dev Device handle
 * @param adc_code Threshold register value
 */
bq_status_t bq7690x_write_max_temp_threshold(bq7690x_t *dev, uint8_t adc_code);

/**
 * Read MaxInternalTemp for cell balancing (I1)
 * @param dev Device handle
 * @param temp_c Pointer to receive temperature in °C (signed)
 */
bq_status_t bq7690x_read_max_internal_temp(bq7690x_t *dev, int8_t *temp_c);

/**
 * Write MaxInternalTemp for cell balancing (I1)
 * @param dev Device handle
 * @param temp_c Temperature in °C (signed)
 */
bq_status_t bq7690x_write_max_internal_temp(bq7690x_t *dev, int8_t temp_c);
