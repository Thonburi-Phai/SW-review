/**
 * @file bq7690x_fet.c
 * @brief BQ7690x FET Control Implementation
 */

#include "bq7690x_fet.h"

/* Read FETControl (0x68) */
bq_status_t bq7690x_read_fet_control(bq7690x_t *dev, uint8_t *value)
{
    return bq_read_reg8(dev, FETControl, value);
}

/* Write FETControl (0x68) */
bq_status_t bq7690x_write_fet_control(bq7690x_t *dev, uint8_t value)
{
    return bq_write_reg8(dev, FETControl, value);
}

/* Set FET control with verification */
bq_status_t bq7690x_set_fet_control(bq7690x_t *dev, uint8_t onoff_mask, uint8_t *verify_out)
{
    if (!dev) return BQ_ERR_PARAM;

    bq_status_t st = bq_write_reg8(dev, FETControl, onoff_mask);
    if (st != BQ_OK) return st;

    dev->delay_ms(2);

    uint8_t rb = 0;
    st = bq_read_reg8(dev, FETControl, &rb);
    if (st != BQ_OK) return st;

    if (verify_out) *verify_out = rb;

    if (rb != onoff_mask)
        return BQ_ERR_VERIFY_MISMATCH;

    return BQ_OK;
}

/* Read REGOUTControl (0x69) */
bq_status_t bq7690x_read_regout_control_reg(bq7690x_t *dev, uint8_t *value)
{
    return bq_read_reg8(dev, REGOUTControl, value);
}

/* Write REGOUTControl (0x69) */
bq_status_t bq7690x_write_regout_control_reg(bq7690x_t *dev, uint8_t value)
{
    return bq_write_reg8(dev, REGOUTControl, value);
}

/* Read DSGFETDriverPWMControl (0x6A) */
bq_status_t bq7690x_read_dsg_pwm_control(bq7690x_t *dev, uint16_t *value)
{
    return bq_read_reg16(dev, DSGFETDriverPWMControl, value);
}

/* Write DSGFETDriverPWMControl (0x6A) */
bq_status_t bq7690x_write_dsg_pwm_control(bq7690x_t *dev, uint16_t value)
{
    return bq_write_reg16(dev, DSGFETDriverPWMControl, value);
}

/* Read CHGFETDriverPWMControl (0x6C) */
bq_status_t bq7690x_read_chg_pwm_control(bq7690x_t *dev, uint16_t *value)
{
    return bq_read_reg16(dev, CHGFETDriverPWMControl, value);
}

/* Write CHGFETDriverPWMControl (0x6C) */
bq_status_t bq7690x_write_chg_pwm_control(bq7690x_t *dev, uint16_t value)
{
    return bq_write_reg16(dev, CHGFETDriverPWMControl, value);
}
