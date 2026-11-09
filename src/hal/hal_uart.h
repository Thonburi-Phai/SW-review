/*
 * hal_uart.h -- LPUART0 debug port for S32K116 BMS
 *
 * TX: PTB1 (ALT2), RX: PTB0 (ALT2), 8N1. See hal_uart.c for the baud divisors.
 * Low-level TX plus printf-like formatting; RX interrupt feeds the triple command
 * receiver.
 */

#ifndef HAL_UART_H
#define HAL_UART_H

#include <stdint.h>

/* Initialize clocks, pins, and LPUART0 peripheral (TX + RX + RX IRQ) */
void hal_uart_init(void);

/* Blocking polled output */
void hal_uart_putc(char c);
void hal_uart_puts(const char *s);
void hal_uart_put_hex8(uint8_t val);
void hal_uart_put_hex16(uint16_t val);
void hal_uart_put_hex32(uint32_t val);
void hal_uart_put_dec16(uint16_t val);
void hal_uart_put_dec32(uint32_t val);
void hal_uart_put_sdec16(int16_t val);
void hal_uart_put_sdec32(int32_t val);

/* Send raw bytes (for binary protocol binary packets) */
void hal_uart_send(const uint8_t *data, uint16_t len);

/*
 * Console gate -- on V2 the debug console and binary protocol share one UART, and the gate
 * selects which owns the line. binary protocol is the default once init has finished; boot
 * banners print before that.
 *
 * Both sinks are blocking polled writes from the single-threaded main loop and no ISR
 * writes the UART, so console text lands between frames, never inside one.
 *
 * On V1 the console owns LPUART0, the gate is compiled out, and every call here is a
 * no-op that reports the console as enabled.
 *
 * hold_ms on enable is a deadline after which console mode reverts to binary protocol by
 * itself; 0 holds it indefinitely.
 */
void hal_uart_console_enable(uint32_t hold_ms);
void hal_uart_console_disable(void);
uint8_t hal_uart_console_is_enabled(void);

/* Printf-like formatted output (uses vsnprintf internally) */
void hal_uart_printf(const char *fmt, ...) __attribute__((format(printf, 1, 2)));

/* RX interrupt buffer access */
uint8_t hal_uart_rx_available(void);
uint8_t hal_uart_rx_getc(void);

#endif /* HAL_UART_H */
