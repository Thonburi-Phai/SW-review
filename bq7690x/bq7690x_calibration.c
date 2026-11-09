/**
 * @file bq7690x_calibration.c
 * @brief BQ7690x Calibration Register Implementation
 */

#include "bq7690x_calibration.h"

/* ========== Voltage Calibration ========== */

bq_status_t bq7690x_read_cell_gain(bq7690x_t *dev, uint16_t *gain)
{
    return bq_dm_read_16(dev, CellGain, gain);
}

bq_status_t bq7690x_write_cell_gain(bq7690x_t *dev, uint16_t gain)
{
    return bq_dm_write_16_cfg(dev, CellGain, gain);
}

bq_status_t bq7690x_read_stack_gain(bq7690x_t *dev, uint16_t *gain)
{
    return bq_dm_read_16(dev, StackGain, gain);
}

bq_status_t bq7690x_write_stack_gain(bq7690x_t *dev, uint16_t gain)
{
    return bq_dm_write_16_cfg(dev, StackGain, gain);
}

bq_status_t bq7690x_read_cell2_offset(bq7690x_t *dev, int8_t *offset)
{
    return bq_dm_read_8(dev, Cell2Offset, (uint8_t *)offset);
}

bq_status_t bq7690x_write_cell2_offset(bq7690x_t *dev, int8_t offset)
{
    return bq_dm_write_8_cfg(dev, Cell2Offset, (uint8_t)offset);
}

bq_status_t bq7690x_read_cell3_offset(bq7690x_t *dev, int8_t *offset)
{
    return bq_dm_read_8(dev, Cell3Offset, (uint8_t *)offset);
}

bq_status_t bq7690x_write_cell3_offset(bq7690x_t *dev, int8_t offset)
{
    return bq_dm_write_8_cfg(dev, Cell3Offset, (uint8_t)offset);
}

bq_status_t bq7690x_read_cell45_offset(bq7690x_t *dev, int8_t *offset)
{
    return bq_dm_read_8(dev, Cell45Offset, (uint8_t *)offset);
}

bq_status_t bq7690x_write_cell45_offset(bq7690x_t *dev, int8_t offset)
{
    return bq_dm_write_8_cfg(dev, Cell45Offset, (uint8_t)offset);
}

bq_status_t bq7690x_read_cell67_offset(bq7690x_t *dev, int8_t *offset)
{
    return bq_dm_read_8(dev, Cell67Offset, (uint8_t *)offset);
}

bq_status_t bq7690x_write_cell67_offset(bq7690x_t *dev, int8_t offset)
{
    return bq_dm_write_8_cfg(dev, Cell67Offset, (uint8_t)offset);
}

/* ========== Current Calibration ========== */

bq_status_t bq7690x_read_curr_gain(bq7690x_t *dev, uint16_t *gain)
{
    return bq_dm_read_16(dev, CurrGain, gain);
}

bq_status_t bq7690x_write_curr_gain(bq7690x_t *dev, uint16_t gain)
{
    return bq_dm_write_16_cfg(dev, CurrGain, gain);
}

bq_status_t bq7690x_read_curr_offset(bq7690x_t *dev, int16_t *offset)
{
    return bq_dm_read_16(dev, CurrOffset, (uint16_t *)offset);
}

bq_status_t bq7690x_write_curr_offset(bq7690x_t *dev, int16_t offset)
{
    return bq_dm_write_16_cfg(dev, CurrOffset, (uint16_t)offset);
}

bq_status_t bq7690x_read_cc1_gain(bq7690x_t *dev, uint16_t *gain)
{
    return bq_dm_read_16(dev, CC1Gain, gain);
}

bq_status_t bq7690x_write_cc1_gain(bq7690x_t *dev, uint16_t gain)
{
    return bq_dm_write_16_cfg(dev, CC1Gain, gain);
}

bq_status_t bq7690x_read_cc1_offset(bq7690x_t *dev, int16_t *offset)
{
    return bq_dm_read_16(dev, CC1Offset, (uint16_t *)offset);
}

bq_status_t bq7690x_write_cc1_offset(bq7690x_t *dev, int16_t offset)
{
    return bq_dm_write_16_cfg(dev, CC1Offset, (uint16_t)offset);
}

/* ========== Temperature Calibration ========== */

bq_status_t bq7690x_read_ts_offset(bq7690x_t *dev, int16_t *offset)
{
    return bq_dm_read_16(dev, TSOffset, (uint16_t *)offset);
}

bq_status_t bq7690x_write_ts_offset(bq7690x_t *dev, int16_t offset)
{
    return bq_dm_write_16_cfg(dev, TSOffset, (uint16_t)offset);
}

bq_status_t bq7690x_read_int_temp_gain(bq7690x_t *dev, uint16_t *gain)
{
    return bq_dm_read_16(dev, IntTempGain, gain);
}

bq_status_t bq7690x_write_int_temp_gain(bq7690x_t *dev, uint16_t gain)
{
    return bq_dm_write_16_cfg(dev, IntTempGain, gain);
}

bq_status_t bq7690x_read_int_temp_offset(bq7690x_t *dev, int16_t *offset)
{
    return bq_dm_read_16(dev, IntTempOffset, (uint16_t *)offset);
}

bq_status_t bq7690x_write_int_temp_offset(bq7690x_t *dev, int16_t offset)
{
    return bq_dm_write_16_cfg(dev, IntTempOffset, (uint16_t)offset);
}
