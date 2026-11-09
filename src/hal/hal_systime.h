/*
 * hal_systime.h -- LPIT-based millisecond timer for S32K116
 *
 * LPIT0 Channel 0: 1ms periodic interrupt (SIRC 8 MHz / 8000 = 1 kHz)
 * Provides systime_milli() and delay_ms() for the BMS firmware.
 */

#ifndef HAL_SYSTIME_H
#define HAL_SYSTIME_H

#include <stdint.h>

/* Initialize LPIT0 CH0 for 1ms tick + NVIC */
void hal_systime_init(void);

/* Get current millisecond tick count (wraps at 2^32, i.e. every 49.7 days) */
uint32_t systime_milli(void);

/* Seconds since boot; wraps at 2^32 s (136 years). */
uint32_t systime_seconds(void);

/* Milliseconds counted by the 1 ms tick alone, excluding systime_advance() credits.
 * Used by the LPTMR wake handler to credit only the part of a sleep period the tick
 * did not count. */
uint32_t systime_lpit_ms(void);

/* Add ms to the tick counter (used by LPTMR to compensate VLPS time) */
void systime_advance(uint32_t ms);

/* Blocking delay in milliseconds */
void delay_ms(uint32_t ms);

#endif /* HAL_SYSTIME_H */
