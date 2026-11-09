/*
 * hal_adc.c -- ADC0 wrapper for S32K116 BMS
 *
 * 12-bit resolution, 32x hardware averaging, SIRC 8 MHz clock.
 */

#include "hal_adc.h"
#include "device_registers.h"

void hal_adc_init(void)
{
    /* PORTA clock (ADC pins PTA0, PTA1, PTA7 already analog by default) */
    PCC->PCCn[PCC_PORTA_INDEX] |= PCC_PCCn_CGC_MASK;

    /* PORTC clock for ADC0_SE9 on PTC1 (analog by default at MUX=0) */
    PCC->PCCn[PCC_PORTC_INDEX] |= PCC_PCCn_CGC_MASK;

    /* ADC0 clock: SIRC 8 MHz */
    PCC->PCCn[PCC_ADC0_INDEX] &= ~PCC_PCCn_CGC_MASK;
    PCC->PCCn[PCC_ADC0_INDEX]  = PCC_PCCn_PCS(2);
    PCC->PCCn[PCC_ADC0_INDEX] |= PCC_PCCn_CGC_MASK;

    /* Disable (no channel selected) */
    ADC0->SC1[0] = ADC_SC1_ADCH(0x1Fu);

    /* 12-bit, no prescaler */
    ADC0->CFG1 = ADC_CFG1_ADIV(0) | ADC_CFG1_MODE(1);

    /* 13 ADC clock sample time */
    ADC0->CFG2 = ADC_CFG2_SMPLTS(12);

    /* SW trigger, VREFH/VREFL */
    ADC0->SC2 = 0;

    /* Enable HW averaging, 32 samples */
    ADC0->SC3 = ADC_SC3_AVGE_MASK | ADC_SC3_AVGS(3);

    /* Run self-calibration */
    ADC0->SC3 |= ADC_SC3_CAL_MASK;
    while ((ADC0->SC1[0] & ADC_SC1_COCO_MASK) == 0u) {}
}

uint16_t hal_adc_read_channel(uint8_t ch)
{
    ADC0->SC1[0] = ADC_SC1_ADCH(ch);
    while ((ADC0->SC1[0] & ADC_SC1_COCO_MASK) == 0u) {}
    return (uint16_t)(ADC0->R[0] & 0xFFFu);
}

uint16_t hal_adc_to_mv(uint16_t raw)
{
    return (uint16_t)((3300u * (uint32_t)raw) / 4095u);
}
