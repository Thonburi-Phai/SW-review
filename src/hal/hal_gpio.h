/*
 * hal_gpio.h -- GPIO abstraction for the S32K116 BMS TAAP board.
 *
 * Pin numbers live in board_pins.h; this is the behaviour each one carries.
 *   LED_1, LED_2   status outputs, each with an external mirror pin
 *   BQ_ALERT       input, active LOW, pull-up
 *   TS_WAKE_MCU    output, wakes the BQ76907 from SHUTDOWN
 *   EN_Temperature output, NTC bias rail
 */

#ifndef HAL_GPIO_H
#define HAL_GPIO_H

#include <stdint.h>

/* Initialize all GPIO pins */
void hal_gpio_init(void);

/* LED control */
void hal_gpio_led1_on(void);
void hal_gpio_led1_off(void);
void hal_gpio_led1_toggle(void);
void hal_gpio_led2_on(void);
void hal_gpio_led2_off(void);
void hal_gpio_led2_toggle(void);

/* BQ76907 ALERT pin, active LOW */
uint8_t hal_gpio_read_alert(void);

/* BQ76907 TS_WAKE pin */
void hal_gpio_ts_wake_high(void);
void hal_gpio_ts_wake_low(void);

/* NTC temperature bias rail */
void hal_gpio_set_en_temperature(uint8_t on);

#endif /* HAL_GPIO_H */
