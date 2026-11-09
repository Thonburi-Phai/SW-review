/*
 * hal_crc.h -- STM32F4-compatible hardware CRC-32 for S32K116
 *
 * Poly 0x04C11DB7, seed 0xFFFFFFFF, no bit reversal, no final XOR,
 * 32-bit word feed MSB-first.
 */

#ifndef HAL_CRC_H
#define HAL_CRC_H

#include <stdint.h>

/* Initialize CRC peripheral clock */
void hal_crc_init(void);

/* Compute the CRC-32 over a data buffer. */
uint32_t hal_crc_stm32_compute(const uint8_t *data, uint32_t size);

#endif /* HAL_CRC_H */
