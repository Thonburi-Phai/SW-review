/**
 * @file bq7690x_fet.h
 * @brief CHG/DSG FET control; requires HOST_FETON and HOST_FETOFF set in FETOptions.
 */

#pragma once
#include "bq7690x.h"
#include "bq7690x_reg.h"

/* FET Control Bitmasks */
#define BQ_FET_DSG_ON   (1u << 0)  /* Turn ON discharge FET */
#define BQ_FET_CHG_ON   (1u << 1)  /* Turn ON charge FET */
#define BQ_FET_DSG_OFF  (1u << 2)  /* Turn OFF discharge FET */
#define BQ_FET_CHG_OFF  (1u << 3)  /* Turn OFF charge FET */

/**
 * Read current FET control register (0x68)
 * @param dev Device handle
 * @param value Pointer to receive FET control byte
 * @return BQ_OK or error code
 */
bq_status_t bq7690x_read_fet_control(bq7690x_t *dev, uint8_t *value);

/**
 * Write FET control register (0x68)
 * Use BQ_FET_* bitmasks to control FETs
 * @param dev Device handle
 * @param value FET control bitmask
 * @return BQ_OK or error code
 */
bq_status_t bq7690x_write_fet_control(bq7690x_t *dev, uint8_t value);

/**
 * Set FET control, with read-back verification
 * @param dev Device handle
 * @param onoff_mask FET control bitmask (BQ_FET_DSG_ON | BQ_FET_CHG_ON, etc.)
 * @param verify_out Optional pointer to receive read-back value (can be NULL)
 * @return BQ_OK, or BQ_ERR_VERIFY_MISMATCH if read-back differs
 */
bq_status_t bq7690x_set_fet_control(bq7690x_t *dev, uint8_t onoff_mask, uint8_t *verify_out);

/**
 * Read REGOUT control register (0x69)
 * @param dev Device handle
 * @param value Pointer to receive REGOUT control byte
 * @return BQ_OK or error code
 */
bq_status_t bq7690x_read_regout_control_reg(bq7690x_t *dev, uint8_t *value);

/**
 * Write REGOUT control register (0x69)
 * @param dev Device handle
 * @param value REGOUT control byte
 * @return BQ_OK or error code
 */
bq_status_t bq7690x_write_regout_control_reg(bq7690x_t *dev, uint8_t value);

/**
 * Read DSG FET driver PWM control (0x6A)
 * @param dev Device handle
 * @param value Pointer to receive PWM control word
 * @return BQ_OK or error code
 */
bq_status_t bq7690x_read_dsg_pwm_control(bq7690x_t *dev, uint16_t *value);

/**
 * Write DSG FET driver PWM control (0x6A)
 * @param dev Device handle
 * @param value PWM control word
 * @return BQ_OK or error code
 */
bq_status_t bq7690x_write_dsg_pwm_control(bq7690x_t *dev, uint16_t value);

/**
 * Read CHG FET driver PWM control (0x6C)
 * @param dev Device handle
 * @param value Pointer to receive PWM control word
 * @return BQ_OK or error code
 */
bq_status_t bq7690x_read_chg_pwm_control(bq7690x_t *dev, uint16_t *value);

/**
 * Write CHG FET driver PWM control (0x6C)
 * @param dev Device handle
 * @param value PWM control word
 * @return BQ_OK or error code
 */
bq_status_t bq7690x_write_chg_pwm_control(bq7690x_t *dev, uint16_t value);
