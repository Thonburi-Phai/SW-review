/**
 * @file bq7690x_measurements.h
 * @brief BQ7690x voltage, current and temperature measurement registers.
 */

#pragma once
#include "bq7690x.h"
#include "bq7690x_reg.h"

/* ========== Voltage Measurements ========== */

/**
 * @brief Read individual cell voltage
 * @param dev Device handle
 * @param cell_index Cell number (1-7, where 1=bottom cell)
 * @param mV Output: voltage in millivolts (signed)
 * @return BQ_OK or error code
 */
bq_status_t bq7690x_read_cell_voltage(bq7690x_t *dev, uint8_t cell_index, int16_t *mV);

/**
 * @brief Read total stack voltage
 * @param dev Device handle
 * @param mV Output: total pack voltage in millivolts
 * @return BQ_OK or error code
 */
bq_status_t bq7690x_read_stack_voltage(bq7690x_t *dev, int16_t *mV);

/**
 * @brief Read internal 1.8V regulator voltage
 * @param dev Device handle
 * @param reg18_mV Output: REG18 voltage in millivolts
 * @return BQ_OK or error code
 */
bq_status_t bq7690x_read_reg18_voltage(bq7690x_t *dev, int16_t *reg18_mV);

/**
 * @brief Read VSS voltage (ADC diagnostic)
 * @param dev Device handle
 * @param vss_mV Output: VSS voltage in millivolts
 * @return BQ_OK or error code
 */
bq_status_t bq7690x_read_vss_voltage(bq7690x_t *dev, int16_t *vss_mV);

/* ========== Current Measurements ========== */

/**
 * @brief Read raw 32-bit current measurement
 * @param dev Device handle
 * @param raw Output: raw current value (32-bit)
 * @return BQ_OK or error code
 */
bq_status_t bq7690x_read_raw_current(bq7690x_t *dev, int32_t *raw);

/**
 * @brief Read CC2 current measurement
 * @param dev Device handle
 * @param mA Output: current in mA (signed, 16-bit)
 * @return BQ_OK or error code
 */
bq_status_t bq7690x_read_current(bq7690x_t *dev, int16_t *mA);

/**
 * @brief Read CC1 current measurement
 * @param dev Device handle
 * @param mA Output: current in mA (signed, 16-bit)
 * @return BQ_OK or error code
 */
bq_status_t bq7690x_read_cc1_current(bq7690x_t *dev, int16_t *mA);

/* ========== Temperature Measurements ========== */

/**
 * @brief Read internal die temperature
 * @param dev Device handle
 * @param temp Output: temperature in 0.1°C units
 * @return BQ_OK or error code
 */
bq_status_t bq7690x_read_internal_temperature(bq7690x_t *dev, int16_t *temp);

/**
 * @brief Read external thermistor (TS1) temperature
 * @param dev Device handle
 * @param temp Output: temperature in 0.1°C units
 * @return BQ_OK or error code
 */
bq_status_t bq7690x_read_ts1_temperature(bq7690x_t *dev, int16_t *temp);
