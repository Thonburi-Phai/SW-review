/*
 * hal_systime.c -- LPIT-based millisecond timer for S32K116
 *
 * LPIT0 Channel 0: SIRCDIV2 4 MHz / 4000 = 1 kHz (1 ms tick)
 * NVIC IRQ 20 = LPIT0_IRQHandler
 *
 * PCS=2 selects SIRCDIV2, which main() programs as divide-by-2 of the 8 MHz SIRC,
 * so the peripheral clock here is 4 MHz.
 */

#include "hal_systime.h"
#include "device_registers.h"

/* ISR prototype (weak symbol in startup_S32K116.S) */
void LPIT0_IRQHandler(void);

static volatile uint32_t g_systime_ms;
static volatile uint32_t g_systime_sec;
static volatile uint16_t g_ms_frac;      /* ISR-owned; never touched from thread context */

/* Milliseconds counted by this tick alone, never added to by systime_advance(), so
 * the LPTMR wake handler can credit only the part of a sleep period it missed. */
static volatile uint32_t g_lpit_ms;

/* Sub-second carry for systime_advance(), kept apart from the tick ISR's g_ms_frac
 * so each has exactly one writer. */
static volatile uint16_t g_adv_frac;

void LPIT0_IRQHandler(void)
{
    if ((LPIT0->MSR & LPIT_MSR_TIF0_MASK) != 0u) {
        LPIT0->MSR = LPIT_MSR_TIF0_MASK;   /* Clear flag (W1C) */
        g_systime_ms++;
        g_lpit_ms++;
        if (++g_ms_frac >= 1000u) { g_ms_frac = 0u; g_systime_sec++; }
    }
}

void hal_systime_init(void)
{
    /* LPIT0 clock: SIRCDIV2 = 4 MHz via PCS=2 */
    PCC->PCCn[PCC_LPIT_INDEX] &= ~PCC_PCCn_CGC_MASK;
    PCC->PCCn[PCC_LPIT_INDEX]  = PCC_PCCn_PCS(2);
    PCC->PCCn[PCC_LPIT_INDEX] |= PCC_PCCn_CGC_MASK;

    /* Enable module; keep running in doze/wait mode (needed for WFI in VLPR) */
    LPIT0->MCR |= LPIT_MCR_M_CEN_MASK | LPIT_MCR_DOZE_EN_MASK;

    /* Channel 0: 1 ms = 4 MHz / 4000 */
    LPIT0->TMR[0].TVAL = 4000u - 1u;

    /* Enable CH0 interrupt */
    LPIT0->MIER |= LPIT_MIER_TIE0_MASK;

    /* Start CH0 */
    LPIT0->TMR[0].TCTRL |= LPIT_TMR_TCTRL_T_EN_MASK;

    /* NVIC: LPIT0 IRQ = 20 */
    S32_NVIC->ICPR[0] = (1u << 20);
    S32_NVIC->ISER[0] = (1u << 20);
}

uint32_t systime_milli(void)
{
    return g_systime_ms;
}

uint32_t systime_seconds(void)
{
    return g_systime_sec;
}

uint32_t systime_lpit_ms(void)
{
    return g_lpit_ms;
}

void systime_advance(uint32_t ms)
{
    g_systime_ms += ms;

    /* Carry the sub-second remainder in g_adv_frac rather than discarding it; a
     * credit is one LPTMR period (around 83 ms), so ms / 1000 alone is usually zero.
     * g_adv_frac is separate from the tick ISR's accumulator, so each has a single
     * writer. g_systime_sec is incremented from both handlers, which run at the same
     * priority and cannot preempt one another. */
    g_adv_frac += (uint16_t)(ms % 1000u);

    uint32_t whole = ms / 1000u;
    if (g_adv_frac >= 1000u) {
        g_adv_frac = (uint16_t)(g_adv_frac - 1000u);
        whole++;
    }
    g_systime_sec += whole;
}

void delay_ms(uint32_t ms)
{
    uint32_t start = g_systime_ms;
    while ((g_systime_ms - start) < ms) {
        __asm volatile("nop");
    }
}
