/**
 * @file bq7690x_config.c
 * @brief BQ7690x Settings:Configuration Implementation
 */

#include "bq7690x_config.h"

/* ========== Power Configuration ========== */

bq_status_t bq7690x_read_power_config(bq7690x_t *dev, uint8_t *value)
{
    return bq_dm_read_8_cfg(dev, PowerConfig, value);
}

bq_status_t bq7690x_write_power_config(bq7690x_t *dev, uint8_t value)
{
    return bq_dm_write_8_cfg(dev, PowerConfig, value);
}

/* ========== REGOUT Configuration ========== */

bq_status_t bq7690x_read_regout_config(bq7690x_t *dev, uint8_t *value)
{
    return bq_dm_read_8_cfg(dev, REGOUTConfig, value);
}

bq_status_t bq7690x_write_regout_config(bq7690x_t *dev, uint8_t value)
{
    return bq_dm_write_8_cfg(dev, REGOUTConfig, value);
}

/* ========== I2C Configuration ========== */

bq_status_t bq7690x_read_i2c_address(bq7690x_t *dev, uint8_t *addr)
{
    return bq_dm_read_8_cfg(dev, I2CAddress, addr);
}

bq_status_t bq7690x_write_i2c_address(bq7690x_t *dev, uint8_t addr)
{
    return bq_dm_write_8_cfg(dev, I2CAddress, addr);
}

bq_status_t bq7690x_read_i2c_config(bq7690x_t *dev, uint16_t *config)
{
    return bq_dm_read_16_cfg(dev, I2CConfig, config);
}

bq_status_t bq7690x_write_i2c_config(bq7690x_t *dev, uint16_t config)
{
    return bq_dm_write_16_cfg(dev, I2CConfig, config);
}

/* ========== DA Configuration ========== */

bq_status_t bq7690x_read_da_config(bq7690x_t *dev, uint16_t *config)
{
    return bq_dm_read_16_cfg(dev, DAConfig, config);
}

bq_status_t bq7690x_write_da_config(bq7690x_t *dev, uint16_t config)
{
    return bq_dm_write_16_cfg(dev, DAConfig, config);
}

/* ========== Vcell Mode ========== */

bq_status_t bq7690x_read_vcell_mode(bq7690x_t *dev, uint8_t *cells)
{
    return bq_dm_read_8_cfg(dev, VcellMode, cells);
}

bq_status_t bq7690x_write_vcell_mode(bq7690x_t *dev, uint8_t cells)
{
    if (!dev) return BQ_ERR_PARAM;
    if (cells < 3 || cells > 7) return BQ_ERR_PARAM;

    bq_status_t st = bq_dm_write_8_cfg(dev, VcellMode, cells);

    /* Update device structure if successful */
    if (st == BQ_OK)
        dev->cells = cells;

    return st;
}

/* ========== Default Alarm Mask ========== */

bq_status_t bq7690x_read_default_alarm_mask(bq7690x_t *dev, uint16_t *mask)
{
    return bq_dm_read_16_cfg(dev, DefaultAlarmMask, mask);
}

bq_status_t bq7690x_write_default_alarm_mask(bq7690x_t *dev, uint16_t mask)
{
    return bq_dm_write_16_cfg(dev, DefaultAlarmMask, mask);
}

/* ========== FET Options ========== */

bq_status_t bq7690x_read_fet_options(bq7690x_t *dev, uint8_t *options)
{
    return bq_dm_read_8_cfg(dev, FETOptions, options);
}

bq_status_t bq7690x_write_fet_options(bq7690x_t *dev, uint8_t options)
{
    return bq_dm_write_8_cfg(dev, FETOptions, options);
}

/* ========== Charge Detector Time ========== */

bq_status_t bq7690x_read_charge_detector_time(bq7690x_t *dev, uint8_t *time_sec)
{
    return bq_dm_read_8_cfg(dev, ChargeDetectorTime, time_sec);
}

bq_status_t bq7690x_write_charge_detector_time(bq7690x_t *dev, uint8_t time_sec)
{
    return bq_dm_write_8_cfg(dev, ChargeDetectorTime, time_sec);
}

/* ========== Balancing Configuration ========== */

bq_status_t bq7690x_read_balancing_configuration(bq7690x_t *dev, uint8_t *config)
{
    return bq_dm_read_8_cfg(dev, BalancingConfiguration, config);
}

bq_status_t bq7690x_write_balancing_configuration(bq7690x_t *dev, uint8_t config)
{
    return bq_dm_write_8_cfg(dev, BalancingConfiguration, config);
}

/* ========== Balancing Temperature Thresholds ========== */

bq_status_t bq7690x_read_min_temp_threshold(bq7690x_t *dev, uint8_t *temp_c)
{
    return bq_dm_read_8_cfg(dev, MinTempThreshold, temp_c);
}

bq_status_t bq7690x_write_min_temp_threshold(bq7690x_t *dev, uint8_t temp_c)
{
    return bq_dm_write_8_cfg(dev, MinTempThreshold, temp_c);
}

bq_status_t bq7690x_read_max_temp_threshold(bq7690x_t *dev, uint8_t *temp_c)
{
    return bq_dm_read_8_cfg(dev, MaxTempThreshold, temp_c);
}

bq_status_t bq7690x_write_max_temp_threshold(bq7690x_t *dev, uint8_t temp_c)
{
    return bq_dm_write_8_cfg(dev, MaxTempThreshold, temp_c);
}

bq_status_t bq7690x_read_max_internal_temp(bq7690x_t *dev, int8_t *temp_c)
{
    return bq_dm_read_8_cfg(dev, MaxInternalTemp, (uint8_t *)temp_c);
}

bq_status_t bq7690x_write_max_internal_temp(bq7690x_t *dev, int8_t temp_c)
{
    return bq_dm_write_8_cfg(dev, MaxInternalTemp, (uint8_t)temp_c);
}
