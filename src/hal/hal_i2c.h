/*
 * hal_i2c.h -- LPI2C0 master wrapper for S32K116 BMS
 *
 * PTA2 = SDA (ALT3), PTA3 = SCL (ALT3)
 * SIRC 8 MHz, PRESCALE=2, ~77 kHz standard mode
 *
 * Function signatures match BQ7690x driver callback interface:
 *   write: int (*)(uint8_t addr7, const uint8_t *data, uint16_t len)
 *   read:  int (*)(uint8_t addr7, uint8_t reg, uint8_t *data, uint16_t len)
 */

#ifndef HAL_I2C_H
#define HAL_I2C_H

#include <stdint.h>

/* Initialize LPI2C0 master (clocks, pins, peripheral) */
void hal_i2c_init(void);

/*
 * I2C write: data[0] = register address, data[1..len-1] = payload
 * Matches BQ7690x write callback signature.
 * Returns 0 on success, -1 on error.
 */
int hal_i2c_write(uint8_t addr7, const uint8_t *data, uint16_t len);

/*
 * I2C register read: write reg addr, repeated start, read data.
 * Matches BQ7690x read callback signature.
 * Returns 0 on success, -1 on error.
 */
int hal_i2c_read_reg(uint8_t addr7, uint8_t reg, uint8_t *data, uint16_t len);

/*
 * Check if device responds on I2C bus.
 * Returns 0 if device ACKs, -1 if NACK.
 */
int hal_i2c_probe(uint8_t addr7);

#endif /* HAL_I2C_H */
