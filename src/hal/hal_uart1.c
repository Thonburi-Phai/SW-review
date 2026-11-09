/*
 * hal_uart1.c -- LPUART1 binary protocol port for S32K116 BMS
 *
 * Pins come from board_pins.h: V1 TX PTC7 / RX PTC6, V2 TX PTC9 / RX PTC8.
 * NVIC IRQ 30 = LPUART1_RxTx_IRQHandler
 * Interrupt-driven RX with a circular buffer; polled TX.
 */

#include "hal_uart1.h"
#include "device_registers.h"
#include "board_pins.h"

/* ISR prototype (weak symbol in startup_S32K116.S) */
void LPUART1_RxTx_IRQHandler(void);

/* RX circular buffer */
#define RX1_BUF_SIZE 256u
static volatile uint8_t rx1_buf[RX1_BUF_SIZE];
static volatile uint16_t rx1_head;
static volatile uint16_t rx1_tail;

/* Debug counters, readable from main context */
volatile uint32_t g_uart1_isr_count = 0;
volatile uint32_t g_uart1_isr_rx_count = 0;
volatile uint32_t g_uart1_isr_err_count = 0;

/* Set by the ISR when RXEDGIF fires during VLPS; tells the main loop to stay in VLPW */
volatile uint8_t g_uart1_rx_wakeup = 0;

void LPUART1_RxTx_IRQHandler(void)
{
    uint32_t stat = LPUART1->STAT;
    g_uart1_isr_count++;

    /* RX active edge: wakes CPU from VLPS. Clear flag and drop SLEEPDEEP
     * so next WFI enters VLPW instead of VLPS. */
    if (stat & LPUART_STAT_RXEDGIF_MASK) {
        LPUART1->STAT = (stat & 0x3FE00000u) | LPUART_STAT_RXEDGIF_MASK;
        S32_SCB->SCR &= ~S32_SCB_SCR_SLEEPDEEP_MASK;
        g_uart1_rx_wakeup = 1;
    }

    uint32_t err = stat & (LPUART_STAT_OR_MASK | LPUART_STAT_NF_MASK |
                           LPUART_STAT_FE_MASK | LPUART_STAT_PF_MASK);
    if (err) {
        LPUART1->STAT = (stat & 0x3FE00000u) | err;
        g_uart1_isr_err_count++;
    }

    if ((stat & LPUART_STAT_RDRF_MASK) != 0u) {
        uint8_t c = (uint8_t)(LPUART1->DATA & 0xFFu);
        uint16_t next = (rx1_head + 1u) & (RX1_BUF_SIZE - 1u);
        if (next != rx1_tail) {
            rx1_buf[rx1_head] = c;
            rx1_head = next;
        }
        g_uart1_isr_rx_count++;
    }
}

void hal_uart1_init(void)
{
    /* PORTC clock */
    PCC->PCCn[BRD_BP_PORT_PCC] |= PCC_PCCn_CGC_MASK;

    /* LPUART1 clock: SIRC 8 MHz (PCS=2) */
    PCC->PCCn[PCC_LPUART1_INDEX] &= ~PCC_PCCn_CGC_MASK;
    PCC->PCCn[PCC_LPUART1_INDEX]  = PCC_PCCn_PCS(2);
    PCC->PCCn[PCC_LPUART1_INDEX] |= PCC_PCCn_CGC_MASK;

    /* RX (pull-up) and TX, per board_pins.h: V1 PTC6/PTC7, V2 PTC8/PTC9 */
    BRD_BP_PORT->PCR[BRD_BP_RX_PIN] =
        PORT_PCR_MUX(BRD_BP_MUX) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;
    BRD_BP_PORT->PCR[BRD_BP_TX_PIN] = PORT_PCR_MUX(BRD_BP_MUX);

    /* 115200 baud: SIRC/DIV2 4MHz / (2 * 17) = 117647 (2.1% error)
     * RXEDGIE: RX active edge interrupt, wakes the CPU from VLPS */
    LPUART1->BAUD = LPUART_BAUD_SBR(2) | LPUART_BAUD_OSR(16)
                  | LPUART_BAUD_RXEDGIE_MASK;

    /* Enable TX + RX + RX interrupt (DOZEEN=0: stays active in VLPS) */
    LPUART1->CTRL = LPUART_CTRL_TE_MASK | LPUART_CTRL_RE_MASK | LPUART_CTRL_RIE_MASK;

    /* NVIC: LPUART1 IRQ = 30 */
    S32_NVIC->ICPR[0] = (1u << 30);
    S32_NVIC->ISER[0] = (1u << 30);
}

void hal_uart1_send(const uint8_t *data, uint16_t len)
{
    for (uint16_t i = 0u; i < len; i++) {
        while ((LPUART1->STAT & LPUART_STAT_TDRE_MASK) == 0u) {}
        LPUART1->DATA = data[i];
    }
}

uint8_t hal_uart1_rx_available(void)
{
    return (rx1_head != rx1_tail) ? 1u : 0u;
}

uint8_t hal_uart1_rx_getc(void)
{
    if (rx1_head == rx1_tail) return 0u;
    uint8_t c = rx1_buf[rx1_tail];
    rx1_tail = (rx1_tail + 1u) & (RX1_BUF_SIZE - 1u);
    return c;
}

void hal_uart1_loopback_enable(void)
{
    LPUART1->CTRL &= ~(LPUART_CTRL_TE_MASK | LPUART_CTRL_RE_MASK);
    LPUART1->CTRL |= LPUART_CTRL_LOOPS_MASK;
    LPUART1->CTRL |= LPUART_CTRL_TE_MASK | LPUART_CTRL_RE_MASK;
}

void hal_uart1_loopback_disable(void)
{
    LPUART1->CTRL &= ~(LPUART_CTRL_TE_MASK | LPUART_CTRL_RE_MASK);
    LPUART1->CTRL &= ~LPUART_CTRL_LOOPS_MASK;
    LPUART1->CTRL |= LPUART_CTRL_TE_MASK | LPUART_CTRL_RE_MASK;
}
