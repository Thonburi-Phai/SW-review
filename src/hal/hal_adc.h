/*
 * hal_adc.h -- ADC0 wrapper for S32K116 BMS
 *
 * 12-bit, 32x hardware averaging, SIRC 8 MHz
 * External NTC channels: SE0 (PTA0), SE1 (PTA1), SE3 (PTA7), SE9 (PTC1)
 * Internal channels: Ch26 (die temp), Ch27 (bandgap), Ch29 (VDDA)
 */

#ifndef HAL_ADC_H
#define HAL_ADC_H

#include <stdint.h>

/* Initialize ADC0 peripheral and run self-calibration */
void hal_adc_init(void);

/* Read a single ADC channel (blocking). Returns 12-bit result. */
uint16_t hal_adc_read_channel(uint8_t ch);

/* Convert 12-bit raw ADC to millivolts (assuming VREF=3.3V) */
uint16_t hal_adc_to_mv(uint16_t raw);

#endif /* HAL_ADC_H */
