/**
 * @file bq7690x_measurements.c
 * @brief BQ7690x Measurement Functions Implementation
 */

#include "bq7690x_measurements.h"

/* ========== Voltage Measurements ========== */

bq_status_t bq7690x_read_cell_voltage(bq7690x_t *dev, uint8_t cell_index, int16_t *mV)
{
    if (cell_index < 1 || cell_index > 7) return BQ_ERR_PARAM;

    uint8_t reg = Cell1Voltage + (cell_index - 1) * 2;
    return bq_read_reg16(dev, reg, (uint16_t*)mV);
}

bq_status_t bq7690x_read_stack_voltage(bq7690x_t *dev, int16_t *mV)
{
    return bq_read_reg16(dev, StackVoltage, (uint16_t*)mV);
}

bq_status_t bq7690x_read_reg18_voltage(bq7690x_t *dev, int16_t *reg18_mV)
{
    return bq_read_reg16(dev, REG18Voltage, (uint16_t*)reg18_mV);
}

bq_status_t bq7690x_read_vss_voltage(bq7690x_t *dev, int16_t *vss_mV)
{
    return bq_read_reg16(dev, VSSVoltage, (uint16_t*)vss_mV);
}

/* ========== Current Measurements ========== */

bq_status_t bq7690x_read_raw_current(bq7690x_t *dev, int32_t *raw)
{
    return bq_read_reg32(dev, RawCurrent, (uint32_t*)raw);
}

bq_status_t bq7690x_read_current(bq7690x_t *dev, int16_t *mA)
{
    return bq_read_reg16(dev, Current, (uint16_t*)mA);
}

bq_status_t bq7690x_read_cc1_current(bq7690x_t *dev, int16_t *mA)
{
    return bq_read_reg16(dev, CC1Current, (uint16_t*)mA);
}

/* ========== Temperature Measurements ========== */

bq_status_t bq7690x_read_internal_temperature(bq7690x_t *dev, int16_t *temp)
{
    return bq_read_reg16(dev, IntTemperature, (uint16_t*)temp);
}

bq_status_t bq7690x_read_ts1_temperature(bq7690x_t *dev, int16_t *temp)
{
    return bq_read_reg16(dev, TS1Temperature, (uint16_t*)temp);
}
