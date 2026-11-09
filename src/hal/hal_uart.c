/*
 * hal_uart.c -- console sink for the S32K116 BMS.
 *
 * The board has one usable UART, so console text shares the binary protocol
 * line on LPUART1. Every print funnels through hal_uart_putc() and is gated on
 * console_allowed(), so unframed text goes out only while the console holds the
 * wire. There is no console RX.
 */

#include "hal_uart.h"
#include "device_registers.h"
#include "board_pins.h"
#include "hal_uart1.h"
#include "hal_systime.h"
#include <stdarg.h>
#include <stdio.h>

/*
 * The three-identical-character command detector cannot be used on a shared
 * line, because binary protocol payloads contain runs of identical bytes;
 * switching between console and binary protocol is a CRC-protected command.
 */
void hal_uart_init(void)
{
    hal_uart1_init();          /* idempotent */
}

uint8_t hal_uart_rx_available(void) { return 0u; }
uint8_t hal_uart_rx_getc(void)      { return 0u; }

/* One wire, two owners. */
static uint8_t  s_console_on     = 1u;   /* boot banners print before the mode arms */
static uint32_t s_console_until  = 0u;   /* 0 = no deadline */

void hal_uart_console_enable(uint32_t hold_ms)
{
    s_console_on    = 1u;
    s_console_until = (hold_ms != 0u) ? (systime_milli() + hold_ms) : 0u;
}

void hal_uart_console_disable(void)
{
    s_console_on    = 0u;
    s_console_until = 0u;
}

/*
 * The hold deadline is checked on each print rather than from a main-loop timer, so it
 * also applies in code paths that never reach the main loop.
 */
static uint8_t console_allowed(void)
{
    if (s_console_on == 0u) {
        return 0u;
    }
    if (s_console_until != 0u) {
        /* Signed difference so the comparison survives the millisecond counter wrap. */
        if ((int32_t)(systime_milli() - s_console_until) >= 0) {
            s_console_on    = 0u;
            s_console_until = 0u;
            return 0u;
        }
    }
    return 1u;
}

uint8_t hal_uart_console_is_enabled(void) { return console_allowed(); }

void hal_uart_putc(char c)
{
    if (console_allowed() == 0u) {
        return;
    }
    uint8_t b = (uint8_t)c;
    hal_uart1_send(&b, 1u);
}

void hal_uart_puts(const char *s)
{
    while (*s != '\0') {
        hal_uart_putc(*s);
        s++;
    }
}

void hal_uart_put_hex8(uint8_t val)
{
    static const char hex[] = "0123456789ABCDEF";
    hal_uart_putc(hex[(val >> 4u) & 0x0Fu]);
    hal_uart_putc(hex[val & 0x0Fu]);
}

void hal_uart_put_hex16(uint16_t val)
{
    hal_uart_put_hex8((uint8_t)(val >> 8u));
    hal_uart_put_hex8((uint8_t)(val & 0xFFu));
}

void hal_uart_put_hex32(uint32_t val)
{
    hal_uart_put_hex16((uint16_t)(val >> 16u));
    hal_uart_put_hex16((uint16_t)(val & 0xFFFFu));
}

void hal_uart_put_dec16(uint16_t val)
{
    char buf[6];
    int i = 0;
    if (val == 0u) { hal_uart_putc('0'); return; }
    while (val > 0u) {
        buf[i++] = (char)('0' + (char)(val % 10u));
        val /= 10u;
    }
    while (i > 0) { hal_uart_putc(buf[--i]); }
}

void hal_uart_put_dec32(uint32_t val)
{
    char buf[11];
    int i = 0;
    if (val == 0u) { hal_uart_putc('0'); return; }
    while (val > 0u) {
        buf[i++] = (char)('0' + (char)(val % 10u));
        val /= 10u;
    }
    while (i > 0) { hal_uart_putc(buf[--i]); }
}

void hal_uart_put_sdec16(int16_t val)
{
    if (val < 0) { hal_uart_putc('-'); val = -val; }
    hal_uart_put_dec16((uint16_t)val);
}

void hal_uart_put_sdec32(int32_t val)
{
    if (val < 0) { hal_uart_putc('-'); val = -val; }
    hal_uart_put_dec32((uint32_t)val);
}

void hal_uart_send(const uint8_t *data, uint16_t len)
{
    /* Gated like every other console sink; this goes out the binary protocol wire. */
    if (console_allowed() == 0u) {
        return;
    }
    /* LPUART0 is unclocked and its pins belong to CAN0, so writing it would hang
     * on a dead TDRE; use the console's actual port, as hal_uart_putc() does. */
    hal_uart1_send(data, len);
}

void hal_uart_printf(const char *fmt, ...)
{
    char buf[256];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (n > 0) {
        hal_uart_puts(buf);
    }
}

