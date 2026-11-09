/**
 * @file main_controller.h
 * @brief Main application controller for the BMS system.
 *
 * Coordinates system initialization, BQ7690x battery management, UART
 * communication, status monitoring and ALERT pin handling.
 */

#ifndef MAIN_CONTROLLER_H
#define MAIN_CONTROLLER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/*---------------------------------------------------------------------------*/
/* Controller Functions                                                      */
/*---------------------------------------------------------------------------*/

/**
 * @brief Initialize main controller and its subsystems.
 *
 * Call once at startup before entering the main loop.
 */
void controller_init(void);

/**
 * @brief Process all pending UART RX data and execute commands.
 *
 * Feeds bytes from LPUART0 (debug) and LPUART1 (binary protocol) into their
 * respective parsers, then dispatches any complete commands.
 */
void controller_poll_uart(void);

void controller_run(void);

#ifdef __cplusplus
}
#endif

#endif /* MAIN_CONTROLLER_H */
