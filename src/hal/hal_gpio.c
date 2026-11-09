/*
 * hal_gpio.c -- GPIO abstraction for S32K116 BMS TAAP board
 */

#include "hal_gpio.h"
#include "device_registers.h"
#include "board_pins.h"

void hal_gpio_init(void)
{
    /* Enable port clocks. PORTE carries the external LED pair. */
    PCC->PCCn[PCC_PORTA_INDEX] |= PCC_PCCn_CGC_MASK;
    PCC->PCCn[PCC_PORTB_INDEX] |= PCC_PCCn_CGC_MASK;
    PCC->PCCn[PCC_PORTC_INDEX] |= PCC_PCCn_CGC_MASK;
    PCC->PCCn[PCC_PORTD_INDEX] |= PCC_PCCn_CGC_MASK;
    PCC->PCCn[PCC_PORTE_INDEX] |= PCC_PCCn_CGC_MASK;

    /* Status LEDs, off at start. */
    BRD_LED1_PORT->PCR[BRD_LED1_PIN] = PORT_PCR_MUX(1);
    BRD_LED1_GPIO->PDDR |= (1u << BRD_LED1_PIN);
    BRD_LED1_GPIO->PCOR  = (1u << BRD_LED1_PIN);

    BRD_LED2_PORT->PCR[BRD_LED2_PIN] = PORT_PCR_MUX(1);
    BRD_LED2_GPIO->PDDR |= (1u << BRD_LED2_PIN);
    BRD_LED2_GPIO->PCOR  = (1u << BRD_LED2_PIN);

    /* The external pair mirrors the internal one. */
    BRD_LED1_EXT_PORT->PCR[BRD_LED1_EXT_PIN] = PORT_PCR_MUX(1);
    BRD_LED1_EXT_GPIO->PDDR |= (1u << BRD_LED1_EXT_PIN);
    BRD_LED1_EXT_GPIO->PCOR  = (1u << BRD_LED1_EXT_PIN);

    BRD_LED2_EXT_PORT->PCR[BRD_LED2_EXT_PIN] = PORT_PCR_MUX(1);
    BRD_LED2_EXT_GPIO->PDDR |= (1u << BRD_LED2_EXT_PIN);
    BRD_LED2_EXT_GPIO->PCOR  = (1u << BRD_LED2_EXT_PIN);

    /* BQ76907 ALERT: input, pull-up, active LOW. */
    BRD_ALERT_PORT->PCR[BRD_ALERT_PIN] =
        PORT_PCR_MUX(1) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;
    BRD_ALERT_GPIO->PDDR &= ~(1u << BRD_ALERT_PIN);

    /* TS_WAKE_MCU: output, start LOW. */
    BRD_TS_WAKE_PORT->PCR[BRD_TS_WAKE_PIN] = PORT_PCR_MUX(1);
    BRD_TS_WAKE_GPIO->PDDR |= (1u << BRD_TS_WAKE_PIN);
    BRD_TS_WAKE_GPIO->PCOR  = (1u << BRD_TS_WAKE_PIN);

    /* EN_Temperature: output, start HIGH = thermistor bias on. */
    BRD_EN_TEMP_PORT->PCR[BRD_EN_TEMP_PIN] = PORT_PCR_MUX(1);
    BRD_EN_TEMP_GPIO->PDDR |= (1u << BRD_EN_TEMP_PIN);
    BRD_EN_TEMP_GPIO->PSOR  = (1u << BRD_EN_TEMP_PIN);
}

/* Each LED index drives its internal pin and its external mirror together. */
#define LED1_SET()    do { BRD_LED1_GPIO->PSOR = (1u << BRD_LED1_PIN); \
                           BRD_LED1_EXT_GPIO->PSOR = (1u << BRD_LED1_EXT_PIN); } while (0)
#define LED1_CLR()    do { BRD_LED1_GPIO->PCOR = (1u << BRD_LED1_PIN); \
                           BRD_LED1_EXT_GPIO->PCOR = (1u << BRD_LED1_EXT_PIN); } while (0)
#define LED1_TGL()    do { BRD_LED1_GPIO->PTOR = (1u << BRD_LED1_PIN); \
                           BRD_LED1_EXT_GPIO->PTOR = (1u << BRD_LED1_EXT_PIN); } while (0)
#define LED2_SET()    do { BRD_LED2_GPIO->PSOR = (1u << BRD_LED2_PIN); \
                           BRD_LED2_EXT_GPIO->PSOR = (1u << BRD_LED2_EXT_PIN); } while (0)
#define LED2_CLR()    do { BRD_LED2_GPIO->PCOR = (1u << BRD_LED2_PIN); \
                           BRD_LED2_EXT_GPIO->PCOR = (1u << BRD_LED2_EXT_PIN); } while (0)
#define LED2_TGL()    do { BRD_LED2_GPIO->PTOR = (1u << BRD_LED2_PIN); \
                           BRD_LED2_EXT_GPIO->PTOR = (1u << BRD_LED2_EXT_PIN); } while (0)

void hal_gpio_led1_on(void)     { LED1_SET(); }
void hal_gpio_led1_off(void)    { LED1_CLR(); }
void hal_gpio_led1_toggle(void) { LED1_TGL(); }
void hal_gpio_led2_on(void)     { LED2_SET(); }
void hal_gpio_led2_off(void)    { LED2_CLR(); }
void hal_gpio_led2_toggle(void) { LED2_TGL(); }

uint8_t hal_gpio_read_alert(void)
{
    /* ALERT is active LOW: return 1 if active (pin LOW) */
    return ((BRD_ALERT_GPIO->PDIR >> BRD_ALERT_PIN) & 1u) == 0u ? 1u : 0u;
}

void hal_gpio_ts_wake_high(void) { BRD_TS_WAKE_GPIO->PSOR = (1u << BRD_TS_WAKE_PIN); }
void hal_gpio_ts_wake_low(void)  { BRD_TS_WAKE_GPIO->PCOR = (1u << BRD_TS_WAKE_PIN); }

void hal_gpio_set_en_temperature(uint8_t on)
{
    if (on) { BRD_EN_TEMP_GPIO->PSOR = (1u << BRD_EN_TEMP_PIN); }
    else    { BRD_EN_TEMP_GPIO->PCOR = (1u << BRD_EN_TEMP_PIN); }
}

