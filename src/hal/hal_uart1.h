/*
 * hal_uart1.h -- LPUART1 binary protocol port for S32K116 BMS
 *
 * Pins come from board_pins.h: V1 TX PTC7 / RX PTC6, V2 TX PTC9 / RX PTC8.
 * Through an ADuM1201 isolator to the J2300 connector.
 * Carries the binary protocol binary protocol; 8N1. See hal_uart1.c for the baud divisors.
 */

#ifndef HAL_UART1_H
#define HAL_UART1_H

#include <stdint.h>

/* Initialize clocks, pins, and LPUART1 peripheral (TX + RX + RX IRQ) */
void hal_uart1_init(void);

/* Send raw bytes (binary protocol binary packets) */
void hal_uart1_send(const uint8_t *data, uint16_t len);

/* RX interrupt buffer access */
uint8_t hal_uart1_rx_available(void);
uint8_t hal_uart1_rx_getc(void);

/* VLPS wake: set by ISR on RX active edge, cleared by main loop */
extern volatile uint8_t g_uart1_rx_wakeup;

/* Debug: internal loopback (TX feeds back to RX) */
void hal_uart1_loopback_enable(void);
void hal_uart1_loopback_disable(void);

#endif /* HAL_UART1_H */
