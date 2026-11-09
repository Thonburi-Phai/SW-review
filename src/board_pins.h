/*
 * board_pins.h -- pin assignments for the S32K116 BMS TAAP board.
 */

#ifndef BOARD_PINS_H
#define BOARD_PINS_H

#define BRD_NAME                "V2"

/* LPUART0's pins (PTB1/PTB0) are CAN0 on this board, so the console has no port
 * of its own: LPUART1 owns the peripheral and the console shares the binary
 * protocol line. */
#define BRD_BP_PORT             PORTC
#define BRD_BP_PORT_PCC         PCC_PORTC_INDEX
#define BRD_BP_TX_PIN           9
#define BRD_BP_RX_PIN           8
#define BRD_BP_MUX              2

/* Four LEDs: an internal pair and an external pair. LED index 1 and 2 each drive
 * their internal and external pin together. */
#define BRD_LED1_PORT           PORTD
#define BRD_LED1_GPIO           PTD
#define BRD_LED1_PIN            16
#define BRD_LED2_PORT           PORTD
#define BRD_LED2_GPIO           PTD
#define BRD_LED2_PIN            15
#define BRD_LED1_EXT_PORT       PORTE
#define BRD_LED1_EXT_GPIO       PTE
#define BRD_LED1_EXT_PIN        8
#define BRD_LED2_EXT_PORT       PORTE
#define BRD_LED2_EXT_GPIO       PTE
#define BRD_LED2_EXT_PIN        9

#define BRD_ALERT_PORT          PORTA
#define BRD_ALERT_GPIO          PTA
#define BRD_ALERT_PIN           12

#define BRD_TS_WAKE_PORT        PORTD
#define BRD_TS_WAKE_GPIO        PTD
#define BRD_TS_WAKE_PIN         2

#define BRD_EN_TEMP_PORT        PORTD
#define BRD_EN_TEMP_GPIO        PTD
#define BRD_EN_TEMP_PIN         3

/* NTC_CH1..4 -> ADC0 channel numbers. The driver addresses channels, not pins:
 * SE12 is PTC14, SE3 is PTA7, SE1 is PTA1, SE0 is PTA0. */
#define BRD_ADC_NTC_CHANNELS    { 12, 3, 1, 0 }

#endif /* BOARD_PINS_H */
