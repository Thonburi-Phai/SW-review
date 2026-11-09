/**
 * @file bq7690x_power.h
 * @brief BQ7690x sleep, shutdown and security setting registers.
 */

#pragma once
#include "bq7690x.h"
#include "bq7690x_reg.h"

/* ========== Sleep Configuration ========== */

bq_status_t bq7690x_read_sleep_current(bq7690x_t *dev, int16_t *current_ma);
bq_status_t bq7690x_write_sleep_current(bq7690x_t *dev, int16_t current_ma);

bq_status_t bq7690x_read_voltage_time(bq7690x_t *dev, uint8_t *time_sec);
bq_status_t bq7690x_write_voltage_time(bq7690x_t *dev, uint8_t time_sec);

bq_status_t bq7690x_read_wake_comparator_current(bq7690x_t *dev, uint8_t *current_index);
bq_status_t bq7690x_write_wake_comparator_current(bq7690x_t *dev, uint8_t current_index);

/* ========== Shutdown Configuration ========== */

bq_status_t bq7690x_read_shutdown_cell_voltage(bq7690x_t *dev, int16_t *voltage_mv);
bq_status_t bq7690x_write_shutdown_cell_voltage(bq7690x_t *dev, int16_t voltage_mv);

bq_status_t bq7690x_read_shutdown_stack_voltage(bq7690x_t *dev, uint16_t *voltage_mv);
bq_status_t bq7690x_write_shutdown_stack_voltage(bq7690x_t *dev, uint16_t voltage_mv);

bq_status_t bq7690x_read_shutdown_temperature(bq7690x_t *dev, uint8_t *temp_c);
bq_status_t bq7690x_write_shutdown_temperature(bq7690x_t *dev, uint8_t temp_c);

bq_status_t bq7690x_read_auto_shutdown_time(bq7690x_t *dev, uint8_t *time_hours);
bq_status_t bq7690x_write_auto_shutdown_time(bq7690x_t *dev, uint8_t time_hours);

/* ========== Security Settings ========== */

bq_status_t bq7690x_read_security_settings(bq7690x_t *dev, uint8_t *settings);
bq_status_t bq7690x_write_security_settings(bq7690x_t *dev, uint8_t settings);

bq_status_t bq7690x_read_full_access_key_step1(bq7690x_t *dev, uint16_t *key);
bq_status_t bq7690x_write_full_access_key_step1(bq7690x_t *dev, uint16_t key);

bq_status_t bq7690x_read_full_access_key_step2(bq7690x_t *dev, uint16_t *key);
bq_status_t bq7690x_write_full_access_key_step2(bq7690x_t *dev, uint16_t key);
