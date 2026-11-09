#pragma once
#include "hal_uart.h"
#include "hal_uart1.h"

/*
 * Map STM32F4 USART names to the S32K116 HAL. Both are 115200 8N1; pins come from
 * board_pins.h.
 *   USART6 = Debug console -> hal_uart   (own LPUART0 on V1; shares LPUART1 on V2)
 *   USART1 = binary protocol link  -> hal_uart1  (LPUART1)
 */
#define USART6_DMA_printf   hal_uart_printf
#define USART6_DMA_send_burst(data, len) hal_uart_send((const uint8_t*)(data), (len))
#define USART6_DMA_send_text hal_uart_puts
#define USART6_enabler()    ((void)0)

#define USART1_DMA_printf   hal_uart_printf
#define USART1_DMA_send_burst(data, len) hal_uart1_send((const uint8_t*)(data), (len))
#define USART1_enabler()    ((void)0)
