/*
 * hw_probe.c -- V2 hardware discovery firmware. NOT part of the application.
 *
 * Reports: die and package (SIM_SDID); crystal presence, range and frequency on
 * PTB7/PTB6 and the derived SIRC error; whether SOSC stays valid in VLPR; the LPO
 * rate used as the LPTMR wake timebase; the SPI flash on LPSPI0 by JEDEC ID and
 * SFDP; whether a CAN transceiver is fitted and powered on PTB1/PTB0; and which
 * pins have anything attached.
 *
 * Constraints:
 *   - The only pins driven as outputs are the three LPSPI0 signals, the flash chip
 *     select and EN_TEMP. Every other pin is read as an input with the internal pull.
 *   - Only read-only flash opcodes are issued; nothing is erased or programmed.
 *   - The FlexCAN transmit pin is never muxed to CAN0_TX, so nothing reaches the bus.
 *   - SWD (PTC4/PTA4), RESET (PTA5) and the console (PTC9/PTC8) are excluded from
 *     the pin scan.
 */

#include "device_registers.h"
#include "board_pins.h"
#include "hal_uart.h"
#include "hal_uart1.h"
#include "hal_adc.h"
#include <stdint.h>

/* ------------------------------------------------------------------ timebase */

/* LPIT channel 0 free-runs as a 32-bit down-counter off SIRCDIV2. Every delay and
 * measurement window in this file is referenced to it. LPIT_HZ is the nominal rate. */
#define LPIT_HZ 4000000u

static void lpit_init(void)
{
    PCC->PCCn[PCC_LPIT_INDEX] &= ~PCC_PCCn_CGC_MASK;
    PCC->PCCn[PCC_LPIT_INDEX]  = PCC_PCCn_PCS(2);      /* SIRCDIV2 = 4 MHz */
    PCC->PCCn[PCC_LPIT_INDEX] |= PCC_PCCn_CGC_MASK;

    LPIT0->MCR = LPIT_MCR_M_CEN_MASK;
    LPIT0->TMR[0].TVAL  = 0xFFFFFFFFu;
    LPIT0->TMR[0].TCTRL = LPIT_TMR_TCTRL_T_EN_MASK;     /* 32-bit periodic */
}

static inline uint32_t lpit_cval(void) { return LPIT0->TMR[0].CVAL; }

/* The counter runs down and reloads; unsigned subtraction gives elapsed ticks
 * across the wrap. */
static inline uint32_t lpit_elapsed(uint32_t start) { return start - lpit_cval(); }

static void delay_us(uint32_t us)
{
    uint32_t start = lpit_cval();
    uint32_t want  = us * (LPIT_HZ / 1000000u);
    while (lpit_elapsed(start) < want) { }
}

static void delay_ms(uint32_t ms)
{
    while (ms-- != 0u) { delay_us(1000u); }
}

/* ------------------------------------------------- counting an unknown clock */

/*
 * LPTMR0 as a pulse counter for a clock of unknown rate, gated by an LPIT-timed
 * window.
 *
 * LPTMR_PSR[PCS] on this part: 0 = SIRCDIV2, 1 = LPO 1 kHz, 2 = RTC,
 * 3 = whatever PCC[LPTMR0][PCS] selects (1 = SOSCDIV2).
 */
static uint32_t lptmr_count(uint32_t lptmr_pcs, uint32_t pcc_pcs,
                            uint32_t bypass, uint32_t prescale,
                            uint32_t window_us)
{
    PCC->PCCn[PCC_LPTMR0_INDEX] &= ~PCC_PCCn_CGC_MASK;
    if (pcc_pcs != 0u) {
        PCC->PCCn[PCC_LPTMR0_INDEX] = PCC_PCCn_PCS(pcc_pcs);
    }
    PCC->PCCn[PCC_LPTMR0_INDEX] |= PCC_PCCn_CGC_MASK;

    LPTMR0->CSR = 0u;                   /* disabled while reconfiguring */
    LPTMR0->PSR = LPTMR_PSR_PCS(lptmr_pcs)
                | (bypass ? LPTMR_PSR_PBYP_MASK : 0u)
                | LPTMR_PSR_PRESCALE(prescale);
    LPTMR0->CMR = 0xFFFFu;              /* free-run the full 16 bits */
    LPTMR0->CSR = LPTMR_CSR_TEN_MASK;

    /* Reading CNR requires a dummy write first to latch the counter. */
    LPTMR0->CNR = 0u;
    uint32_t c0 = LPTMR0->CNR & 0xFFFFu;

    uint32_t start = lpit_cval();
    uint32_t want  = window_us * (LPIT_HZ / 1000000u);
    while (lpit_elapsed(start) < want) { }
    uint32_t actual_us = lpit_elapsed(start) / (LPIT_HZ / 1000000u);

    LPTMR0->CNR = 0u;
    uint32_t c1 = LPTMR0->CNR & 0xFFFFu;

    LPTMR0->CSR = 0u;

    uint32_t delta = (c1 - c0) & 0xFFFFu;
    (void)actual_us;
    return delta;
}

/* Hz from a tick count over a window, staged to stay inside 64 bits. */
static uint32_t ticks_to_hz(uint32_t ticks, uint32_t window_us, uint32_t mult)
{
    uint64_t n = (uint64_t)ticks * mult * 1000000u;
    return (uint32_t)(n / window_us);
}

/*
 * Measure an unknown clock, never using a count that could have wrapped.
 *
 * Start at the largest prescaler, /65536, where even 40 MHz over the window cannot
 * reach 65536 counts, then step down only while the predicted next count -- double
 * the current one -- stays clear of MEAS_CEILING. A slow clock ends up at /1 with
 * few counts, so the window is widened and the measurement repeated.
 */
#define MEAS_CEILING 30000u

static uint32_t measure_at(uint32_t lptmr_pcs, uint32_t pcc_pcs,
                           uint32_t window_us, uint32_t *out_step)
{
    uint32_t step  = 16u;
    uint32_t ticks = 0u;

    for (;;) {
        uint32_t bypass = (step == 0u) ? 1u : 0u;
        uint32_t presc  = (step == 0u) ? 0u : (step - 1u);
        ticks = lptmr_count(lptmr_pcs, pcc_pcs, bypass, presc, window_us);

        if (step == 0u)              { break; }   /* cannot divide less than /1 */
        if ((ticks * 2u) > MEAS_CEILING) { break; }   /* stepping down would risk a wrap */
        step--;
    }

    if (out_step != 0) { *out_step = step; }
    return ticks_to_hz(ticks, window_us, (uint32_t)1u << step);
}

static uint32_t measure_hz(uint32_t lptmr_pcs, uint32_t pcc_pcs, uint32_t *out_window)
{
    uint32_t window_us = 50000u;
    uint32_t step = 0u;
    uint32_t hz = measure_at(lptmr_pcs, pcc_pcs, window_us, &step);

    /* At /1 with a thin count: widen the window for resolution. */
    if (step == 0u && hz != 0u && hz < 100000u) {
        window_us = 500000u;
        hz = measure_at(lptmr_pcs, pcc_pcs, window_us, &step);
    }

    if (out_window != 0) { *out_window = window_us; }
    return hz;
}

static void print_hz(const char *what, uint32_t hz)
{
    hal_uart_printf("  %-22s %lu Hz (%lu.%03lu MHz)\r\n", what,
                    (unsigned long)hz,
                    (unsigned long)(hz / 1000000u),
                    (unsigned long)((hz % 1000000u) / 1000u));
}

/* ------------------------------------------------------------------- T1: die */

static void t1_identity(void)
{
    hal_uart_puts("\r\n[T1] Die and reset identity\r\n");

    uint32_t sdid = SIM->SDID;
    hal_uart_printf("  SIM_SDID               0x%08lX\r\n", (unsigned long)sdid);
    hal_uart_printf("    PACKAGE              %lu\r\n",
                    (unsigned long)((sdid & SIM_SDID_PACKAGE_MASK) >> SIM_SDID_PACKAGE_SHIFT));
    hal_uart_printf("    REVID                %lu\r\n",
                    (unsigned long)((sdid >> 12) & 0xFu));

    hal_uart_printf("  RCM_SRS                0x%08lX  ", (unsigned long)RCM->SRS);
    uint32_t srs = RCM->SRS;
    if (srs & RCM_SRS_POR_MASK)    hal_uart_puts("POR ");
    if (srs & RCM_SRS_PIN_MASK)    hal_uart_puts("PIN ");
    if (srs & RCM_SRS_SW_MASK)     hal_uart_puts("SW ");
    if (srs & RCM_SRS_WDOG_MASK)   hal_uart_puts("WDOG ");
    if (srs & RCM_SRS_LOCKUP_MASK) hal_uart_puts("LOCKUP ");
    if (srs & RCM_SRS_LVD_MASK)    hal_uart_puts("LVD ");
    if (srs & RCM_SRS_LOC_MASK)    hal_uart_puts("LOC ");
    if (srs & RCM_SRS_JTAG_MASK)   hal_uart_puts("JTAG ");
    hal_uart_puts("\r\n");

    /* Flash security and protection: FSEC 0xFE is unsecured, FPROT all-ones is
     * unprotected. */
    hal_uart_printf("  FTFC FSEC              0x%02X\r\n", (unsigned)FTFC->FSEC);
    hal_uart_printf("  FTFC FOPT              0x%02X\r\n", (unsigned)FTFC->FOPT);
    hal_uart_puts("  FTFC FPROT            ");
    for (uint32_t i = 0u; i < FTFC_FPROT_COUNT; i++) {
        hal_uart_printf(" %02X", (unsigned)FTFC->FPROT[i]);
    }
    hal_uart_puts("  (FF = unprotected)\r\n");

    hal_uart_printf("  SCG RCCR               0x%08lX\r\n", (unsigned long)SCG->RCCR);
    hal_uart_printf("  SCG CSR  (live clock)  0x%08lX\r\n", (unsigned long)SCG->CSR);
}

/* ------------------------------------------------------------ T2: crystal */

static const char *sosc_range_name(uint32_t r)
{
    switch (r) {
    case 1u:  return "low (32-40 kHz)";
    case 2u:  return "medium (1-8 MHz)";
    case 3u:  return "high (8-40 MHz)";
    default:  return "reserved";
    }
}

/* Try one oscillator configuration and wait for SOSCVLD. Returns 1 if it locked. */
static int sosc_try(uint32_t range, uint32_t erefs, uint32_t hgo)
{
    SCG->SOSCCSR = 0u;                  /* disable; also clears a latched error */
    delay_ms(1u);

    SCG->SOSCCFG = SCG_SOSCCFG_RANGE(range)
                 | (erefs ? SCG_SOSCCFG_EREFS_MASK : 0u)
                 | (hgo   ? SCG_SOSCCFG_HGO_MASK   : 0u);
    SCG->SOSCDIV = SCG_SOSCDIV_SOSCDIV1(1) | SCG_SOSCDIV_SOSCDIV2(1);  /* /1, /1 */
    SCG->SOSCCSR = SCG_SOSCCSR_SOSCEN_MASK;

    /* A crystal in the low range can take hundreds of ms to start. */
    for (uint32_t i = 0u; i < 1000u; i++) {
        uint32_t csr = SCG->SOSCCSR;
        if (csr & SCG_SOSCCSR_SOSCERR_MASK) { return 0; }
        if (csr & SCG_SOSCCSR_SOSCVLD_MASK) { return 1; }
        delay_ms(1u);
    }
    return 0;
}

static uint32_t s_sosc_hz;              /* 0 if no crystal was found */
static uint32_t s_sosc_range;

static void t2_crystal(void)
{
    hal_uart_puts("\r\n[T2] Crystal on PTB7/PTB6 (EXTAL/XTAL)\r\n");

    /* Two self-checks of the measurement path, each of which must read ~4 MHz:
     * (a) direct: LPTMR PCS=0 is SIRCDIV2, counted against an LPIT also clocked
     *     from SIRCDIV2.
     * (b) via the PCC: LPTMR PCS=3 with PCC PCS=2 points the crystal's own path at
     *     SIRCDIV2. */
    uint32_t win = 0u;
    uint32_t self_direct = measure_hz(0u, 0u, &win);
    print_hz("direct: PCS=0 SIRCDIV2", self_direct);
    uint32_t self_pcc = measure_hz(3u, 2u, &win);
    print_hz("via PCC: PCS=3 PCC=2", self_pcc);

    uint32_t lo = LPIT_HZ - (LPIT_HZ / 20u), hi = LPIT_HZ + (LPIT_HZ / 20u);
    int path_ok = (self_direct > lo && self_direct < hi &&
                   self_pcc    > lo && self_pcc    < hi);
    hal_uart_printf("    both must read ~4 MHz: %s\r\n",
                    path_ok ? "PASS" : "*** FAIL -- crystal reading below is invalid ***");

    static const struct { uint32_t range, erefs, hgo; } attempts[] = {
        { 3u, 1u, 1u }, { 3u, 1u, 0u },
        { 2u, 1u, 1u }, { 2u, 1u, 0u },
        { 1u, 1u, 0u },
        { 3u, 0u, 0u }, { 2u, 0u, 0u },   /* EREFS=0: external square wave, not a xtal */
    };

    for (uint32_t i = 0u; i < sizeof(attempts)/sizeof(attempts[0]); i++) {
        hal_uart_printf("  try RANGE=%lu EREFS=%lu HGO=%lu  %-18s ... ",
                        (unsigned long)attempts[i].range,
                        (unsigned long)attempts[i].erefs,
                        (unsigned long)attempts[i].hgo,
                        sosc_range_name(attempts[i].range));
        if (sosc_try(attempts[i].range, attempts[i].erefs, attempts[i].hgo)) {
            hal_uart_puts("VALID\r\n");
            uint32_t hz = measure_hz(3u, 1u, &win);   /* PCS=3 -> PCC -> SOSCDIV2 */
            print_hz("SOSCDIV2 measured", hz);
            s_sosc_hz    = hz;
            s_sosc_range = attempts[i].range;
            return;
        }
        hal_uart_printf("no (CSR=0x%08lX)\r\n", (unsigned long)SCG->SOSCCSR);
    }

    SCG->SOSCCSR = 0u;
    hal_uart_puts("  RESULT: no oscillator detected in any configuration.\r\n");
}

/* ------------------------------------------------- T3: SIRC error vs crystal */

static void t3_sirc_error(void)
{
    hal_uart_puts("\r\n[T3] SIRC error, referred to the crystal\r\n");

    if (s_sosc_hz == 0u) {
        hal_uart_puts("  skipped: no crystal, nothing better than SIRC to measure against\r\n");
        return;
    }

    /* Snap the SIRC-referenced measurement to the nearest standard crystal value,
     * which turns it into a statement about SIRC's error. */
    static const uint32_t nominals[] = {
        32768u, 1000000u, 4000000u, 8000000u, 12000000u, 16000000u,
        20000000u, 24000000u, 25000000u, 32000000u, 40000000u
    };
    uint32_t best = 0u, best_err = 0xFFFFFFFFu;
    for (uint32_t i = 0u; i < sizeof(nominals)/sizeof(nominals[0]); i++) {
        uint32_t d = (s_sosc_hz > nominals[i]) ? (s_sosc_hz - nominals[i])
                                               : (nominals[i] - s_sosc_hz);
        if (d < best_err) { best_err = d; best = nominals[i]; }
    }

    /* More than 4% (best/25) from every standard value means the measurement path
     * is at fault; no SIRC error is derived. */
    if (best_err > (best / 25u)) {
        hal_uart_printf("  measured %lu Hz is %lu Hz from the nearest standard value\r\n",
                        (unsigned long)s_sosc_hz, (unsigned long)best_err);
        hal_uart_puts("  REFUSING to derive a SIRC error: no crystal is that far off,\r\n");
        hal_uart_puts("  so the measurement path is at fault. Check the T2 self-checks.\r\n");
        return;
    }

    hal_uart_printf("  nearest nominal        %lu Hz\r\n", (unsigned long)best);
    hal_uart_printf("  measured               %lu Hz\r\n", (unsigned long)s_sosc_hz);

    /* measured/nominal inverts to SIRC's error: a crystal reading high means the
     * SIRC-derived timebase runs slow. */
    int32_t ppm = (int32_t)(((int64_t)s_sosc_hz - (int64_t)best) * 1000000 / (int64_t)best);
    hal_uart_printf("  crystal reads           %ld ppm off nominal\r\n", (long)ppm);
    hal_uart_printf("  => SIRC is              %ld ppm (%ld.%02ld%%) off, opposite sign\r\n",
                    (long)(-ppm), (long)(-ppm / 10000), (long)((-ppm < 0 ? -(-ppm) : (-ppm)) % 10000 / 100));
    hal_uart_puts("  (a positive SIRC error means the SIRC-derived tick runs fast)\r\n");
}

/* ------------------------------------------------------------- T4: LPO 1 kHz */

static void t4_lpo(void)
{
    hal_uart_puts("\r\n[T4] LPO, the LPTMR wake timebase\r\n");
    uint32_t win = 0u;
    uint32_t hz = measure_hz(1u, 0u, &win);
    hal_uart_printf("  LPO measured           %lu Hz (nominal 1000)\r\n", (unsigned long)hz);
    if (hz != 0u) {
        int32_t err = (int32_t)hz - 1000;
        hal_uart_printf("  error vs nominal       %ld Hz (%ld.%ld%%)\r\n",
                        (long)err, (long)(err / 10), (long)((err < 0 ? -err : err) % 10));
    }
    hal_uart_puts("  NOTE: this is SIRC-referenced, so it carries SIRC's error too.\r\n");
}

/* ------------------------------------------------------- T5: SOSC in VLPR */

static void t5_sosc_in_vlpr(void)
{
    hal_uart_puts("\r\n[T5] Does SOSC survive VLPR?\r\n");

    if (s_sosc_hz == 0u) {
        hal_uart_puts("  skipped: no crystal to keep running\r\n");
        return;
    }

    hal_uart_printf("  SOSCCSR in RUN         0x%08lX  VLD=%lu\r\n",
                    (unsigned long)SCG->SOSCCSR,
                    (unsigned long)((SCG->SOSCCSR & SCG_SOSCCSR_SOSCVLD_MASK) ? 1u : 0u));

    /* Enter VLPR as the application does: SIRC as system clock, core 4 MHz. The
     * UART is clocked from SIRCDIV2, which VLPR leaves alone. */
    SMC->PMPROT = SMC_PMPROT_AVLP_MASK;
    SCG->SIRCCSR |= SCG_SIRCCSR_SIRCLPEN_MASK;
    SCG->VCCR = SCG_VCCR_SCS(2) | SCG_VCCR_DIVCORE(1)
              | SCG_VCCR_DIVBUS(0) | SCG_VCCR_DIVSLOW(3);
    SMC->PMCTRL = SMC_PMCTRL_RUNM(2);
    while ((SMC->PMSTAT & SMC_PMSTAT_PMSTAT_MASK) != 0x04u) { }

    delay_ms(10u);
    uint32_t csr_vlpr = SCG->SOSCCSR;
    uint32_t scg_csr  = SCG->CSR;

    /* Back to RUN before printing, so the rest of the probe is unaffected. */
    SMC->PMCTRL = SMC_PMCTRL_RUNM(0);
    while ((SMC->PMSTAT & SMC_PMSTAT_PMSTAT_MASK) != 0x01u) { }
    delay_ms(5u);

    hal_uart_printf("  SOSCCSR in VLPR        0x%08lX  VLD=%lu  ERR=%lu\r\n",
                    (unsigned long)csr_vlpr,
                    (unsigned long)((csr_vlpr & SCG_SOSCCSR_SOSCVLD_MASK) ? 1u : 0u),
                    (unsigned long)((csr_vlpr & SCG_SOSCCSR_SOSCERR_MASK) ? 1u : 0u));
    hal_uart_printf("  SCG_CSR in VLPR        0x%08lX (SCS=%lu, 2=SIRC 1=SOSC)\r\n",
                    (unsigned long)scg_csr, (unsigned long)((scg_csr >> 24) & 0xFu));

    if ((csr_vlpr & SCG_SOSCCSR_SOSCVLD_MASK) == 0u) {
        hal_uart_puts("  RESULT: SOSC STOPS in VLPR. The crystal cannot carry the\r\n");
        hal_uart_puts("          timebase; it can only calibrate SIRC in RUN windows.\r\n");
    } else {
        hal_uart_puts("  RESULT: SOSC still valid in VLPR -- the header-based claim in\r\n");
        hal_uart_puts("          the board review was WRONG; revisit using it directly.\r\n");
    }
}

/* ------------------------------------------------------------- T6: SPI flash */

#define FLASH_CS_GPIO   PTB
#define FLASH_CS_PORT   PORTB
#define FLASH_CS_PIN    5u

static void spi_init(void)
{
    PCC->PCCn[PCC_PORTB_INDEX] |= PCC_PCCn_CGC_MASK;

    /* SCK/SOUT/SIN on PTB2/PTB4/PTB3; LPSPI0 is ALT3 on all three. */
    PORTB->PCR[2] = PORT_PCR_MUX(3);
    PORTB->PCR[4] = PORT_PCR_MUX(3);
    PORTB->PCR[3] = PORT_PCR_MUX(3);

    /* Chip select is a plain GPIO. */
    FLASH_CS_PORT->PCR[FLASH_CS_PIN] = PORT_PCR_MUX(1);
    FLASH_CS_GPIO->PSOR = (1u << FLASH_CS_PIN);      /* idle high before output */
    FLASH_CS_GPIO->PDDR |= (1u << FLASH_CS_PIN);

    PCC->PCCn[PCC_LPSPI0_INDEX] &= ~PCC_PCCn_CGC_MASK;
    PCC->PCCn[PCC_LPSPI0_INDEX]  = PCC_PCCn_PCS(2);  /* SIRCDIV2 = 4 MHz */
    PCC->PCCn[PCC_LPSPI0_INDEX] |= PCC_PCCn_CGC_MASK;

    LPSPI0->CR    = 0u;
    LPSPI0->CFGR1 = 1u;                 /* MASTER */
    /* SCK = 4 MHz / (SCKDIV + 2). SCKDIV=6 -> 500 kHz. */
    LPSPI0->CCR   = (4u << 24) | (4u << 16) | (4u << 8) | 6u;
    LPSPI0->FCR   = 0u;
    LPSPI0->CR    = 1u;                 /* MEN */
}

static uint8_t spi_xfer(uint8_t out)
{
    /* 8-bit frame, mode 0, PCS unused (chip select is the GPIO above). */
    LPSPI0->TCR = (0u << 27) | (0u << 24) | (7u << 0);
    LPSPI0->TDR = out;
    while ((LPSPI0->RSR & 0x2u) != 0u) { }      /* RSR[RXEMPTY] */
    return (uint8_t)LPSPI0->RDR;
}

static void cs_low(void)  { FLASH_CS_GPIO->PCOR = (1u << FLASH_CS_PIN); delay_us(2u); }
static void cs_high(void) { delay_us(2u); FLASH_CS_GPIO->PSOR = (1u << FLASH_CS_PIN); delay_us(5u); }

static const char *jedec_vendor(uint8_t mf)
{
    switch (mf) {
    case 0xEFu: return "Winbond";
    case 0xC2u: return "Macronix";
    case 0x20u: return "Micron/ST";
    case 0x1Fu: return "Adesto/Atmel";
    case 0xBFu: return "SST/Microchip";
    case 0x01u: return "Cypress/Spansion";
    case 0x9Du: return "ISSI";
    case 0xC8u: return "GigaDevice";
    case 0x0Bu: return "XTX";
    case 0x85u: return "Puya";
    case 0xA1u: return "Fudan";
    default:    return "unknown";
    }
}

static void t6_spi_flash(void)
{
    hal_uart_puts("\r\n[T6] SPI flash on LPSPI0 (PTB2/3/4), CS PTB5\r\n");
    hal_uart_puts("     read-only opcodes only; nothing is erased or programmed\r\n");

    spi_init();
    hal_uart_printf("  LPSPI0 VERID           0x%08lX  PARAM 0x%08lX\r\n",
                    (unsigned long)LPSPI0->VERID, (unsigned long)LPSPI0->PARAM);

    /* 0x9F JEDEC READ ID: manufacturer, memory type, capacity. */
    cs_low();
    (void)spi_xfer(0x9Fu);
    uint8_t mf  = spi_xfer(0x00u);
    uint8_t typ = spi_xfer(0x00u);
    uint8_t cap = spi_xfer(0x00u);
    cs_high();

    hal_uart_printf("  JEDEC ID (0x9F)        %02X %02X %02X   vendor=%s\r\n",
                    mf, typ, cap, jedec_vendor(mf));

    if ((mf == 0x00u && typ == 0x00u && cap == 0x00u) ||
        (mf == 0xFFu && typ == 0xFFu && cap == 0xFFu)) {
        hal_uart_puts("  RESULT: no response. Either no device, unpowered, or the\r\n");
        hal_uart_puts("          chip select / mux assumption is wrong.\r\n");
        return;
    }

    /* On almost every SPI NOR the capacity byte is log2(bytes). */
    if (cap >= 0x10u && cap <= 0x1Cu) {
        uint32_t kbytes = (1u << (cap - 10u));
        hal_uart_printf("  capacity byte 0x%02X     => %lu KiB (%lu Mbit)\r\n",
                        cap, (unsigned long)kbytes, (unsigned long)(kbytes * 8u / 1024u));
    }

    /* 0x05 read status register 1: bit 0 WIP, bit 1 WEL. */
    cs_low();
    (void)spi_xfer(0x05u);
    uint8_t sr1 = spi_xfer(0x00u);
    cs_high();
    hal_uart_printf("  status reg 1 (0x05)    0x%02X  (WIP=%u WEL=%u)\r\n",
                    sr1, (unsigned)(sr1 & 1u), (unsigned)((sr1 >> 1) & 1u));

    /* 0x5A SFDP: 3 address bytes plus one dummy byte, then the 8-byte header. The
     * "SFDP" signature marks a JEDEC SPI NOR parameter table. */
    cs_low();
    (void)spi_xfer(0x5Au);
    (void)spi_xfer(0x00u); (void)spi_xfer(0x00u); (void)spi_xfer(0x00u);
    (void)spi_xfer(0x00u);                       /* one dummy byte */
    uint8_t sig[8];
    for (uint32_t i = 0u; i < 8u; i++) { sig[i] = spi_xfer(0x00u); }
    cs_high();

    hal_uart_printf("  SFDP header (0x5A)     %02X %02X %02X %02X %02X %02X %02X %02X  '%c%c%c%c'\r\n",
                    sig[0], sig[1], sig[2], sig[3], sig[4], sig[5], sig[6], sig[7],
                    (sig[0] >= 32 && sig[0] < 127) ? sig[0] : '.',
                    (sig[1] >= 32 && sig[1] < 127) ? sig[1] : '.',
                    (sig[2] >= 32 && sig[2] < 127) ? sig[2] : '.',
                    (sig[3] >= 32 && sig[3] < 127) ? sig[3] : '.');
    if (sig[0] == 'S' && sig[1] == 'F' && sig[2] == 'D' && sig[3] == 'P') {
        hal_uart_printf("  SFDP rev %u.%u, %u parameter header(s)\r\n",
                        (unsigned)sig[5], (unsigned)sig[4], (unsigned)(sig[6] + 1u));
    } else {
        hal_uart_puts("  no SFDP signature -- older or non-JEDEC part; use the ID above\r\n");
    }

    /* 0x03 read data, 3 address bytes: first 16 bytes of the array. All 0xFF means
     * erased. */
    cs_low();
    (void)spi_xfer(0x03u);
    (void)spi_xfer(0x00u); (void)spi_xfer(0x00u); (void)spi_xfer(0x00u);
    hal_uart_puts("  first 16 bytes (0x03)  ");
    for (uint32_t i = 0u; i < 16u; i++) {
        hal_uart_printf("%02X ", spi_xfer(0x00u));
    }
    cs_high();
    hal_uart_puts("\r\n");
}

/* ----------------------------------------------------------------- T7: CAN */

static void t7_can(void)
{
    hal_uart_puts("\r\n[T7] CAN0 on PTB1/PTB0\r\n");
    hal_uart_puts("     TX is never muxed to CAN0_TX, so nothing reaches the bus\r\n");

    PCC->PCCn[PCC_PORTB_INDEX] |= PCC_PCCn_CGC_MASK;

    /* A powered transceiver drives RXD high while the bus is idle (recessive), so
     * PTB0 holding high against the internal pull-down means one is fitted; a pin
     * that follows the pull is floating. */
    PORTB->PCR[0] = PORT_PCR_MUX(1) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;  /* pull-up */
    PTB->PDDR &= ~(1u << 0);
    delay_ms(2u);
    uint32_t rx_up = (PTB->PDIR >> 0) & 1u;

    PORTB->PCR[0] = PORT_PCR_MUX(1) | PORT_PCR_PE_MASK;                     /* pull-down */
    delay_ms(2u);
    uint32_t rx_dn = (PTB->PDIR >> 0) & 1u;

    hal_uart_printf("  CAN0_RX (PTB0)         pull-up=%lu pull-down=%lu  -> ",
                    (unsigned long)rx_up, (unsigned long)rx_dn);
    if (rx_up == 1u && rx_dn == 1u) {
        hal_uart_puts("DRIVEN HIGH: transceiver fitted and powered\r\n");
    } else if (rx_up == 0u && rx_dn == 0u) {
        hal_uart_puts("DRIVEN LOW: transceiver present but bus dominant/faulted\r\n");
    } else {
        hal_uart_puts("FLOATING: no transceiver, or it is unpowered\r\n");
    }
    PORTB->PCR[0] = 0u;     /* leave it disabled, not pulled */

    /* Does the FlexCAN module clock and respond? Entering freeze mode touches no
     * pin. */
    PCC->PCCn[PCC_FlexCAN0_INDEX] |= PCC_PCCn_CGC_MASK;
    CAN0->MCR &= ~CAN_MCR_MDIS_MASK;                 /* leave disable mode */
    uint32_t spin = 0u;
    while ((CAN0->MCR & CAN_MCR_LPMACK_MASK) && ++spin < 100000u) { }
    CAN0->MCR |= CAN_MCR_FRZ_MASK | CAN_MCR_HALT_MASK;
    spin = 0u;
    while (!(CAN0->MCR & CAN_MCR_FRZACK_MASK) && ++spin < 100000u) { }

    hal_uart_printf("  FlexCAN0 MCR           0x%08lX  FRZACK=%lu %s\r\n",
                    (unsigned long)CAN0->MCR,
                    (unsigned long)((CAN0->MCR & CAN_MCR_FRZACK_MASK) ? 1u : 0u),
                    (CAN0->MCR & CAN_MCR_FRZACK_MASK) ? "(module alive)" : "(NO RESPONSE)");
    hal_uart_printf("  FlexCAN0 message bufs  %lu\r\n",
                    (unsigned long)((CAN0->MCR & CAN_MCR_MAXMB_MASK) + 1u));

    /* Back to disable mode so it draws nothing. */
    CAN0->MCR |= CAN_MCR_MDIS_MASK;
    PCC->PCCn[PCC_FlexCAN0_INDEX] &= ~PCC_PCCn_CGC_MASK;
}

/* ------------------------------------------------------------- T8: pin scan */

/*
 * For each permitted GPIO: enable the internal pull-up, read, then the internal
 * pull-down, read. A pin that follows the pull is floating; one that holds its level
 * against the opposing pull is externally driven. PDDR is never set, so no pin here
 * is an output.
 */
static const char *classify(uint32_t up, uint32_t dn)
{
    if (up == 1u && dn == 1u) return "DRIVEN HIGH";
    if (up == 0u && dn == 0u) return "DRIVEN LOW ";
    if (up == 1u && dn == 0u) return "floating   ";
    return "unstable   ";
}

static int pin_excluded(char port, uint32_t pin)
{
    /* Touching any of these loses the debugger or the console mid-probe. */
    if (port == 'C' && (pin == 4u))               return 1;   /* SWD_CLK  */
    if (port == 'A' && (pin == 4u || pin == 5u))  return 1;   /* SWD_DIO, RESET */
    if (port == 'C' && (pin == 8u || pin == 9u))  return 1;   /* console UART */
    return 0;
}

static void scan_port(char name, PORT_Type *port, GPIO_Type *gpio, uint32_t npins)
{
    for (uint32_t p = 0u; p < npins; p++) {
        if (pin_excluded(name, p)) {
            hal_uart_printf("  PT%c%-2lu  skipped (SWD/console)\r\n", name, (unsigned long)p);
            continue;
        }

        uint32_t saved = port->PCR[p];

        gpio->PDDR &= ~(1u << p);           /* input, always */
        port->PCR[p] = PORT_PCR_MUX(1) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;
        delay_ms(2u);
        uint32_t up = (gpio->PDIR >> p) & 1u;

        port->PCR[p] = PORT_PCR_MUX(1) | PORT_PCR_PE_MASK;
        delay_ms(2u);
        uint32_t dn = (gpio->PDIR >> p) & 1u;

        port->PCR[p] = saved;

        /* Print only the externally driven pins. */
        if (!(up == 1u && dn == 0u)) {
            hal_uart_printf("  PT%c%-2lu  %s\r\n", name, (unsigned long)p, classify(up, dn));
        }
    }
}

static void t8_pin_scan(void)
{
    hal_uart_puts("\r\n[T8] Pin scan -- only non-floating pins are listed\r\n");
    hal_uart_puts("     inputs with internal pulls only; nothing is driven\r\n");

    PCC->PCCn[PCC_PORTA_INDEX] |= PCC_PCCn_CGC_MASK;
    PCC->PCCn[PCC_PORTB_INDEX] |= PCC_PCCn_CGC_MASK;
    PCC->PCCn[PCC_PORTC_INDEX] |= PCC_PCCn_CGC_MASK;
    PCC->PCCn[PCC_PORTD_INDEX] |= PCC_PCCn_CGC_MASK;
    PCC->PCCn[PCC_PORTE_INDEX] |= PCC_PCCn_CGC_MASK;

    scan_port('A', PORTA, PTA, 18u);
    scan_port('B', PORTB, PTB, 18u);
    scan_port('C', PORTC, PTC, 18u);
    scan_port('D', PORTD, PTD, 18u);
    scan_port('E', PORTE, PTE, 17u);

    /* Unresolved pins, reported as read. */
    hal_uart_puts("  -- the open questions from the board review --\r\n");
    hal_uart_puts("     PTC2/PTC3 are the only plausible second UART.\r\n");
    hal_uart_puts("     A floating result means nothing is attached there on V2.\r\n");
}

/* ------------------------------------------------------------------ T9: ADC */

static void t9_adc(void)
{
    hal_uart_puts("\r\n[T9] ADC channels\r\n");

    hal_adc_init();

    /* EN_TEMP gates power to the thermistor dividers; the NTC channels read near
     * zero until it is driven high. */
    PCC->PCCn[PCC_PORTD_INDEX] |= PCC_PCCn_CGC_MASK;
    BRD_EN_TEMP_PORT->PCR[BRD_EN_TEMP_PIN] = PORT_PCR_MUX(1);
    BRD_EN_TEMP_GPIO->PDDR |= (1u << BRD_EN_TEMP_PIN);

    static const uint8_t ch[] = BRD_ADC_NTC_CHANNELS;

    for (uint32_t pass = 0u; pass < 2u; pass++) {
        if (pass == 0u) {
            BRD_EN_TEMP_GPIO->PCOR = (1u << BRD_EN_TEMP_PIN);
            hal_uart_puts("  -- EN_TEMP low --\r\n");
        } else {
            BRD_EN_TEMP_GPIO->PSOR = (1u << BRD_EN_TEMP_PIN);
            hal_uart_puts("  -- EN_TEMP high --\r\n");
        }
        delay_ms(50u);
        for (uint32_t i = 0u; i < sizeof(ch)/sizeof(ch[0]); i++) {
            uint16_t raw = hal_adc_read_channel(ch[i]);
            hal_uart_printf("  NTC%lu ch%-2u raw=%4u %4u mV\r\n",
                            (unsigned long)(i + 1u), ch[i], raw, hal_adc_to_mv(raw));
        }
    }
    BRD_EN_TEMP_GPIO->PCOR = (1u << BRD_EN_TEMP_PIN);   /* leave it off */

    hal_uart_puts("  -- verdicts, EN_TEMP high --\r\n");
    BRD_EN_TEMP_GPIO->PSOR = (1u << BRD_EN_TEMP_PIN);
    delay_ms(50u);
    for (uint32_t i = 0u; i < sizeof(ch)/sizeof(ch[0]); i++) {
        uint16_t raw = hal_adc_read_channel(ch[i]);
        uint16_t mv  = hal_adc_to_mv(raw);
        const char *verdict = (raw < 40u)   ? "at ground / shorted"
                            : (raw > 4050u) ? "at rail / open (no thermistor)"
                                            : "mid-scale (thermistor present)";
        hal_uart_printf("  NTC%lu  ch%-2u  raw=%4u  %4u mV  %s\r\n",
                        (unsigned long)(i + 1u), ch[i], raw, mv, verdict);
    }

    hal_uart_printf("  die temp   ch26  raw=%4u\r\n", hal_adc_read_channel(26u));
    hal_uart_printf("  bandgap    ch27  raw=%4u\r\n", hal_adc_read_channel(27u));
    hal_uart_printf("  VDDA       ch29  raw=%4u  %4u mV\r\n",
                    hal_adc_read_channel(29u), hal_adc_to_mv(hal_adc_read_channel(29u)));
}

/* ------------------------------------------- T10: crystal vs the host clock */

/*
 * Absolute crystal frequency with no SIRC in the chain: LPTMR0 counts SOSCDIV2 at
 * /128 and free-runs the full 16 bits, so each wrap is 65536 * 128 = 8388608 crystal
 * cycles. One line is printed every WRAPS_PER_LINE wraps; the host divides the
 * change in cycles by the change in its own timestamps.
 */
#define XTAL_CYCLES_PER_WRAP  8388608u      /* 65536 counts * 128 prescale */
#define WRAPS_PER_LINE        10u           /* ~2.1 s per line at 40 MHz */

static void t10_crystal_vs_host(void)
{
    hal_uart_puts("\r\n[T10] Crystal against the host clock -- no SIRC in the chain\r\n");

    if (s_sosc_hz == 0u) {
        hal_uart_puts("  skipped: no oscillator\r\n");
        for (;;) { delay_ms(1000u); }
    }

    /* T5 dropped into VLPR, which stops SOSC; SOSCEN stays set, so check that it
     * restarted on the way back to RUN. */
    if ((SCG->SOSCCSR & SCG_SOSCCSR_SOSCVLD_MASK) == 0u) {
        hal_uart_puts("  SOSC not valid after the VLPR excursion; restarting it\r\n");
        (void)sosc_try(s_sosc_range, 1u, 1u);
    }
    hal_uart_printf("  SOSCCSR 0x%08lX, %lu cycles per line\r\n",
                    (unsigned long)SCG->SOSCCSR,
                    (unsigned long)(XTAL_CYCLES_PER_WRAP * WRAPS_PER_LINE));
    hal_uart_puts("  format: [XTAL] line=<n>; cycles = n * cycles-per-line\r\n");

    PCC->PCCn[PCC_LPTMR0_INDEX] &= ~PCC_PCCn_CGC_MASK;
    PCC->PCCn[PCC_LPTMR0_INDEX]  = PCC_PCCn_PCS(1);        /* SOSCDIV2 */
    PCC->PCCn[PCC_LPTMR0_INDEX] |= PCC_PCCn_CGC_MASK;

    LPTMR0->CSR = 0u;
    LPTMR0->PSR = LPTMR_PSR_PCS(3) | LPTMR_PSR_PRESCALE(6);   /* /128, not bypassed */
    LPTMR0->CMR = 0xFFFFu;
    LPTMR0->CSR = LPTMR_CSR_TEN_MASK;

    uint32_t wraps = 0u, lines = 0u;
    for (;;) {
        /* TCF sets when the counter reaches CMR; clear it by writing the flag back
         * with TEN still set. */
        if ((LPTMR0->CSR & LPTMR_CSR_TCF_MASK) != 0u) {
            LPTMR0->CSR |= LPTMR_CSR_TCF_MASK;
            wraps++;
            if (wraps >= WRAPS_PER_LINE) {
                wraps = 0u;
                lines++;
                /* Line number only: cycles = line * 83886080 overflows 32 bits at
                 * line 51, so the host does that multiply. */
                hal_uart_printf("[XTAL] line=%lu\r\n", (unsigned long)lines);
            }
        }
    }
}

/* ------------------------------------------------------------------- main */

int main(void)
{
    SMC->PMPROT = SMC_PMPROT_AVLP_MASK;

    /* Same SIRC dividers as the application, so peripherals here are clocked as in
     * the real firmware. Stays in RUN apart from the T5 excursion. */
    SCG->SIRCDIV = SCG_SIRCDIV_SIRCDIV1(1) | SCG_SIRCDIV_SIRCDIV2(2);
    SCG->SIRCCSR |= SCG_SIRCCSR_SIRCLPEN_MASK;

    lpit_init();
    hal_uart_init();

    delay_ms(50u);
    hal_uart_puts("\r\n\r\n==================================================\r\n");
    hal_uart_puts("S32K116 V2 hardware probe -- discovery only\r\n");
    hal_uart_puts("no application logic, nothing written to any memory\r\n");
    hal_uart_printf("board_pins.h says: %s\r\n", BRD_NAME);
    hal_uart_puts("==================================================\r\n");

    t1_identity();
    t2_crystal();
    t3_sirc_error();
    t4_lpo();
    t5_sosc_in_vlpr();
    t6_spi_flash();
    t7_can();
    t8_pin_scan();
    t9_adc();

    hal_uart_puts("\r\n[DONE] probe complete.\r\n");
    hal_uart_puts("==================================================\r\n\r\n");

    t10_crystal_vs_host();
}
