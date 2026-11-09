/**
 * @file bq7690x_status.c
 * @brief BQ7690x Status and Alert Functions Implementation
 */

#include "bq7690x_status.h"

/* ========== Safety Alert Functions ========== */

bq_status_t bq7690x_read_safety_alert_a(bq7690x_t *dev, uint8_t *alert)
{
    return bq_read_reg8(dev, SafetyAlertA, alert);
}

bq_status_t bq7690x_read_safety_status_a(bq7690x_t *dev, uint8_t *status)
{
    return bq_read_reg8(dev, SafetyStatusA, status);
}

bq_status_t bq7690x_read_safety_alert_b(bq7690x_t *dev, uint8_t *alert)
{
    return bq_read_reg8(dev, SafetyAlertB, alert);
}

bq_status_t bq7690x_read_safety_status_b(bq7690x_t *dev, uint8_t *status)
{
    return bq_read_reg8(dev, SafetyStatusB, status);
}

/* ========== Battery Status Functions ========== */

bq_status_t bq7690x_read_battery_status(bq7690x_t *dev, uint16_t *battery_status)
{
    return bq_read_reg16(dev, BatteryStatus, battery_status);
}

/* ========== Alarm Status Functions ========== */

bq_status_t bq7690x_read_alarm_status(bq7690x_t *dev, uint16_t *alarm_status)
{
    return bq_read_reg16(dev, AlarmStatus, alarm_status);
}

bq_status_t bq7690x_write_alarm_status(bq7690x_t *dev, uint16_t alarm_status)
{
    return bq_write_reg16(dev, AlarmStatus, alarm_status);
}

bq_status_t bq7690x_read_alarm_raw_status(bq7690x_t *dev, uint16_t *alarm_raw_status)
{
    return bq_read_reg16(dev, AlarmRawStatus, alarm_raw_status);
}

bq_status_t bq7690x_read_alarm_enable(bq7690x_t *dev, uint16_t *alarm_enable)
{
    return bq_read_reg16(dev, AlarmEnable, alarm_enable);
}

bq_status_t bq7690x_write_alarm_enable(bq7690x_t *dev, uint16_t alarm_enable)
{
    return bq_write_reg16(dev, AlarmEnable, alarm_enable);
}
