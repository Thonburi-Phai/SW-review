/**
 * @file bq7690x_power.c
 * @brief Sleep, shutdown and security settings via data memory.
 */

#include "bq7690x_power.h"

/* ========== Sleep Configuration ========== */

bq_status_t bq7690x_read_sleep_current(bq7690x_t *dev, int16_t *current_ma)
{
    return bq_dm_read_16_cfg(dev, SleepCurrent, (uint16_t *)current_ma);
}

bq_status_t bq7690x_write_sleep_current(bq7690x_t *dev, int16_t current_ma)
{
    return bq_dm_write_16_cfg(dev, SleepCurrent, (uint16_t)current_ma);
}

bq_status_t bq7690x_read_voltage_time(bq7690x_t *dev, uint8_t *time_sec)
{
    return bq_dm_read_8_cfg(dev, VoltageTime, time_sec);
}

bq_status_t bq7690x_write_voltage_time(bq7690x_t *dev, uint8_t time_sec)
{
    return bq_dm_write_8_cfg(dev, VoltageTime, time_sec);
}

bq_status_t bq7690x_read_wake_comparator_current(bq7690x_t *dev, uint8_t *current_index)
{
    return bq_dm_read_8_cfg(dev, WakeComparatorCurrent, current_index);
}

bq_status_t bq7690x_write_wake_comparator_current(bq7690x_t *dev, uint8_t current_index)
{
    return bq_dm_write_8_cfg(dev, WakeComparatorCurrent, current_index);
}

/* ========== Shutdown Configuration ========== */

bq_status_t bq7690x_read_shutdown_cell_voltage(bq7690x_t *dev, int16_t *voltage_mv)
{
    return bq_dm_read_16_cfg(dev, ShutdownCellVoltage, (uint16_t *)voltage_mv);
}

bq_status_t bq7690x_write_shutdown_cell_voltage(bq7690x_t *dev, int16_t voltage_mv)
{
    return bq_dm_write_16_cfg(dev, ShutdownCellVoltage, (uint16_t)voltage_mv);
}

bq_status_t bq7690x_read_shutdown_stack_voltage(bq7690x_t *dev, uint16_t *voltage_mv)
{
    return bq_dm_read_16_cfg(dev, ShutdownStackVoltage, voltage_mv);
}

bq_status_t bq7690x_write_shutdown_stack_voltage(bq7690x_t *dev, uint16_t voltage_mv)
{
    return bq_dm_write_16_cfg(dev, ShutdownStackVoltage, voltage_mv);
}

bq_status_t bq7690x_read_shutdown_temperature(bq7690x_t *dev, uint8_t *temp_c)
{
    return bq_dm_read_8_cfg(dev, ShutdownTemperature, temp_c);
}

bq_status_t bq7690x_write_shutdown_temperature(bq7690x_t *dev, uint8_t temp_c)
{
    return bq_dm_write_8_cfg(dev, ShutdownTemperature, temp_c);
}

bq_status_t bq7690x_read_auto_shutdown_time(bq7690x_t *dev, uint8_t *time_hours)
{
    return bq_dm_read_8_cfg(dev, AutoShutdownTime, time_hours);
}

bq_status_t bq7690x_write_auto_shutdown_time(bq7690x_t *dev, uint8_t time_hours)
{
    return bq_dm_write_8_cfg(dev, AutoShutdownTime, time_hours);
}

/* ========== Security Settings ========== */

bq_status_t bq7690x_read_security_settings(bq7690x_t *dev, uint8_t *settings)
{
    return bq_dm_read_8_cfg(dev, SecuritySettings, settings);
}

bq_status_t bq7690x_write_security_settings(bq7690x_t *dev, uint8_t settings)
{
    return bq_dm_write_8_cfg(dev, SecuritySettings, settings);
}

bq_status_t bq7690x_read_full_access_key_step1(bq7690x_t *dev, uint16_t *key)
{
    return bq_dm_read_16_cfg(dev, FullAccessKeyStep1, key);
}

bq_status_t bq7690x_write_full_access_key_step1(bq7690x_t *dev, uint16_t key)
{
    return bq_dm_write_16_cfg(dev, FullAccessKeyStep1, key);
}

bq_status_t bq7690x_read_full_access_key_step2(bq7690x_t *dev, uint16_t *key)
{
    return bq_dm_read_16_cfg(dev, FullAccessKeyStep2, key);
}

bq_status_t bq7690x_write_full_access_key_step2(bq7690x_t *dev, uint16_t key)
{
    return bq_dm_write_16_cfg(dev, FullAccessKeyStep2, key);
}
