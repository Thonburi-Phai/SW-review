/*
 * main.c -- S32K116 BMS TAAP 12V entry point.
 *
 * Initializes the HAL peripherals and enters the controller_init() /
 * controller_run() main loop. WDOG is already disabled by SystemInit().
 */

#include "device_registers.h"
#include "hal_uart.h"
#include "hal_uart1.h"
#include "hal_gpio.h"
#include "hal_systime.h"
#include "hal_i2c.h"
#include "hal_crc.h"
#include "hal_adc.h"
#include "main_controller.h"

/* The LPTMR fires every LPTMR_WAKE_TICKS counts of the untrimmed 1 kHz LPO, so a tick
 * is only nominally 1 ms. LPTMR_WAKE_US is an estimate; the real duration of a wake
 * period is measured at boot by lptmr_calibrate(). */
#define LPTMR_WAKE_TICKS 100u
#define LPTMR_WAKE_US    (LPTMR_WAKE_TICKS * 1000u)

/* Periods timed during calibration: about 2 s at the real LPO rate, which puts the tick
 * counter's +-1 ms quantisation at 0.05 %. The core busy-waits for this window. */
#define LPTMR_CAL_PERIODS 24u
#define LPTMR_CAL_TIMEOUT_MS 5000u

static volatile uint32_t s_lptmr_periods;            /* LPTMR interrupts, for calibration */
static volatile uint32_t s_wake_us = LPTMR_WAKE_US;  /* measured duration of one period */
static volatile uint32_t s_credit_us_frac;           /* sub-millisecond carry */
static volatile uint32_t s_last_lpit_ms;             /* tick count at the previous wake */
static volatile uint8_t  s_cal_in_progress;

/*
 * Credit the part of the sleep the 1 ms tick could not count, and only that part.
 *
 * The tick keeps running in wait mode and while the core is awake working, so crediting
 * a whole period would count the awake part twice. Instead: missed = calibrated period
 * - microseconds the tick counted since the previous wake, with the sub-millisecond
 * remainder carried in s_credit_us_frac. Wait mode, deep sleep and a long busy stretch
 * all come out right without the handler knowing which happened.
 */
void LPTMR0_IRQHandler(void)
{
    LPTMR0->CSR |= LPTMR_CSR_TCF_MASK;

    s_lptmr_periods++;

    uint32_t lpit_now   = systime_lpit_ms();
    uint32_t counted_us = (lpit_now - s_last_lpit_ms) * 1000u;
    s_last_lpit_ms = lpit_now;

    if (s_cal_in_progress != 0u) {
        return;        /* measuring the period; do not credit with a stale estimate */
    }

    uint32_t period_us = s_wake_us;
    if (period_us > counted_us) {
        uint32_t missed = (period_us - counted_us) + s_credit_us_frac;
        uint32_t ms     = missed / 1000u;

        s_credit_us_frac = missed - (ms * 1000u);
        if (ms != 0u) {
            systime_advance(ms);
        }
    }
}

/*
 * Measure how long an LPTMR period really is, in microseconds, against the 1 ms tick.
 *
 * Must run with the core awake and both clocks going, hence the call inside the boot
 * debug window rather than from the sleep loop. The window is bounded by LPTMR periods
 * rather than by milliseconds, so there is no partial-period error: exactly
 * LPTMR_CAL_PERIODS periods are timed and the only uncertainty is the tick counter's
 * own +-1 ms over the window. The result inherits SIRC's error, since the tick is
 * derived from SIRC.
 */
static void lptmr_calibrate(void)
{
    s_cal_in_progress = 1u;

    /* Both waits are bounded, so an LPTMR that never fires cannot hang the boot.
     * LPTMR_CAL_PERIODS periods take about 2 s against the 5 s bound. */
    uint32_t guard_ms = systime_lpit_ms();
    uint32_t p = s_lptmr_periods;
    while (s_lptmr_periods == p) {
        if ((systime_lpit_ms() - guard_ms) > LPTMR_CAL_TIMEOUT_MS) goto bail;
    }

    uint32_t p0 = s_lptmr_periods;
    uint32_t m0 = systime_lpit_ms();
    while ((s_lptmr_periods - p0) < LPTMR_CAL_PERIODS) {
        if ((systime_lpit_ms() - m0) > LPTMR_CAL_TIMEOUT_MS) goto bail;
    }
    uint32_t m1 = systime_lpit_ms();

    uint32_t elapsed_ms = m1 - m0;
    if (elapsed_ms >= LPTMR_CAL_PERIODS) {    /* sanity: a period cannot be under 1 ms */
        s_wake_us = (elapsed_ms * 1000u) / LPTMR_CAL_PERIODS;
    }

    s_last_lpit_ms   = systime_lpit_ms();
    s_credit_us_frac = 0u;
    s_cal_in_progress = 0u;

    /* Report the measured period against the nominal one. */
    hal_uart_printf("[PWR] LPTMR period %lu us (nominal %lu) over %lu ms, %lu periods\r\n",
                    (unsigned long)s_wake_us,
                    (unsigned long)LPTMR_WAKE_US,
                    (unsigned long)elapsed_ms,
                    (unsigned long)LPTMR_CAL_PERIODS);
    return;

bail:
    /* Keep the nominal estimate and report it: time runs off by the LPO's error, but
     * the board boots. */
    s_last_lpit_ms    = systime_lpit_ms();
    s_credit_us_frac  = 0u;
    s_cal_in_progress = 0u;
    hal_uart_printf("[PWR] LPTMR calibration TIMED OUT, using nominal %lu us\r\n",
                    (unsigned long)s_wake_us);
}

static void lptmr_init(void)
{
    PCC->PCCn[PCC_LPTMR0_INDEX] |= PCC_PCCn_CGC_MASK;
    LPTMR0->CSR = 0u;
    LPTMR0->PSR = LPTMR_PSR_PCS(1)
                | LPTMR_PSR_PBYP_MASK;
    LPTMR0->CMR = LPTMR_WAKE_TICKS - 1u;
    LPTMR0->CSR = LPTMR_CSR_TIE_MASK;
    LPTMR0->CSR |= LPTMR_CSR_TEN_MASK;

    S32_NVIC->ICPR[0] = (1u << 8);
    S32_NVIC->ISER[0] = (1u << 8);
}

int main(void)
{
    /* Allow VLP modes (write-once after reset, must be set before VLPR entry) */
    SMC->PMPROT = SMC_PMPROT_AVLP_MASK;

    /* SIRC dividers. The field value is a power-of-two code, not the divisor:
     * DIV1=1 -> /1 = 8 MHz, DIV2=2 -> /2 = 4 MHz. SIRCDIV2 clocks the LPIT, so
     * 4 MHz is the number hal_systime.c divides down for the 1 ms tick. */
    SCG->SIRCDIV = SCG_SIRCDIV_SIRCDIV1(1) | SCG_SIRCDIV_SIRCDIV2(2);

    /* Enable SIRC in low-power modes (VLPR/VLPS) */
    SCG->SIRCCSR |= SCG_SIRCCSR_SIRCLPEN_MASK;

    /* --- HAL init (order matters: uart first for debug output) --- */
    hal_uart_init();
    hal_uart1_init();
    hal_gpio_init();
    hal_systime_init();
    hal_i2c_init();
    hal_crc_init();
    hal_adc_init();

    hal_uart_puts("\r\n=== NXP BMS TAAP 12V v1.0 ===\r\n");
    hal_uart_puts("S32K116 + BQ76907 | 4S LFP\r\n\r\n");

    hal_uart_puts("[PWR] Waiting 5s for debug attach...\r\n");
    delay_ms(5000);

    /* The debug probe is on LPUART1/PTC9. */

    /* --- Enter VLPR mode (SIRC 8 MHz, core 4 MHz) --- */
    SCG->VCCR = SCG_VCCR_SCS(2)
              | SCG_VCCR_DIVCORE(1)
              | SCG_VCCR_DIVBUS(0)
              | SCG_VCCR_DIVSLOW(3);
    SMC->PMCTRL = SMC_PMCTRL_RUNM(2);
    while ((SMC->PMSTAT & 0x04u) == 0u) {}
    SCG->FIRCCSR &= ~SCG_FIRCCSR_FIRCEN_MASK;

    hal_uart_puts("[PWR] VLPR mode active, FIRC off, core 4 MHz\r\n");

    /* --- Application --- */
    controller_init();
    lptmr_init();
    lptmr_calibrate();

    /* VLPS default: LPUART1 RXEDGIE wakes the CPU on an RX start bit and the ISR
     * clears SLEEPDEEP, so subsequent WFIs enter VLPW (UART works). The main loop
     * re-enables SLEEPDEEP after the UART idle timeout. */
    S32_SCB->SCR |= S32_SCB_SCR_SLEEPDEEP_MASK;
    hal_uart_puts("[PWR] VLPS+RXEDGE wake, LPTMR 100ms\r\n");

    uint32_t vlpw_deadline = 0;

    for (;;) {
        controller_run();

        /* RXEDGE ISR clears SLEEPDEEP on any UART start bit. Extend the VLPW window
         * on every RX activity to stay awake through a param-read session; re-arm
         * VLPS after 5 s of silence. */
        if (hal_uart1_rx_available() || g_uart1_rx_wakeup) {
            vlpw_deadline = systime_milli() + 5000u;
            g_uart1_rx_wakeup = 0;
        }
        if (!(S32_SCB->SCR & S32_SCB_SCR_SLEEPDEEP_MASK)) {
            if ((int32_t)(systime_milli() - vlpw_deadline) >= 0) {
                S32_SCB->SCR |= S32_SCB_SCR_SLEEPDEEP_MASK;
            }
        }

        __asm volatile("wfi");
    }

    return 0;
}
