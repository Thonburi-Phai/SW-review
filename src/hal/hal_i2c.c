/*
 * hal_i2c.c -- LPI2C0 master wrapper for S32K116 BMS
 *
 * PTA2 = SDA (ALT3), PTA3 = SCL (ALT3)
 * SIRC 8 MHz, PRESCALE=2, ~77 kHz SCL
 * Every wait loop is bounded by a timeout.
 */

#include "hal_i2c.h"
#include "device_registers.h"

/* LPI2C MTDR CMD field values */
#define CMD_TX_DATA     (0u << 8u)
#define CMD_RX_DATA     (1u << 8u)
#define CMD_STOP        (2u << 8u)
#define CMD_START       (4u << 8u)

/* Timeout: ~50ms at SIRC 8MHz (loop overhead ~10 cycles) */
#define I2C_TIMEOUT     40000u

/* Error flags to check */
#define I2C_ERR_FLAGS   (LPI2C_MSR_NDF_MASK | LPI2C_MSR_ALF_MASK | \
                         LPI2C_MSR_FEF_MASK | LPI2C_MSR_PLTF_MASK)

static int lpi2c_wait_tdf(void)
{
    uint32_t timeout = I2C_TIMEOUT;
    while (((LPI2C0->MSR & LPI2C_MSR_TDF_MASK) == 0u) && (--timeout > 0u)) {
        if ((LPI2C0->MSR & I2C_ERR_FLAGS) != 0u) return -1;
    }
    return (timeout == 0u) ? -1 : 0;
}

static int lpi2c_wait_rdf(void)
{
    uint32_t timeout = I2C_TIMEOUT;
    while (((LPI2C0->MSR & LPI2C_MSR_RDF_MASK) == 0u) && (--timeout > 0u)) {
        if ((LPI2C0->MSR & I2C_ERR_FLAGS) != 0u) return -1;
    }
    return (timeout == 0u) ? -1 : 0;
}

static int lpi2c_wait_stop(void)
{
    uint32_t timeout = I2C_TIMEOUT;
    while (((LPI2C0->MSR & LPI2C_MSR_SDF_MASK) == 0u) && (--timeout > 0u)) {}
    if (timeout == 0u) return -1;
    LPI2C0->MSR = LPI2C_MSR_SDF_MASK;
    return 0;
}

static void lpi2c_clear_status(void)
{
    LPI2C0->MSR = LPI2C_MSR_EPF_MASK | LPI2C_MSR_SDF_MASK
                | LPI2C_MSR_NDF_MASK | LPI2C_MSR_ALF_MASK
                | LPI2C_MSR_FEF_MASK | LPI2C_MSR_PLTF_MASK
                | LPI2C_MSR_DMF_MASK;
}

static void lpi2c_bus_recovery(void)
{
    /* Reset the LPI2C master: disable, flush FIFOs, re-enable */
    LPI2C0->MCR = 0;
    LPI2C0->MCR = LPI2C_MCR_RST_MASK;
    LPI2C0->MCR = 0;

    /* Re-apply configuration */
    LPI2C0->MCFGR1 = LPI2C_MCFGR1_PRESCALE(2);
    LPI2C0->MCCR0  = LPI2C_MCCR0_CLKLO(18)
                    | LPI2C_MCCR0_CLKHI(6)
                    | LPI2C_MCCR0_SETHOLD(6)
                    | LPI2C_MCCR0_DATAVD(3);
    LPI2C0->MFCR   = LPI2C_MFCR_TXWATER(0) | LPI2C_MFCR_RXWATER(0);

    /* Re-enable master */
    LPI2C0->MCR = LPI2C_MCR_MEN_MASK | LPI2C_MCR_DBGEN_MASK;

    lpi2c_clear_status();
}

static void i2c_gpio_delay(void)
{
    /* ~5µs at 8MHz (40 cycles) */
    for (volatile int i = 0; i < 40; i++) {}
}

static void i2c_bus_level_recovery(void)
{
    /* Toggle SCL (PTA3) as GPIO to clock out a stuck slave byte; PTA2 = SDA. */

    /* Set SCL (PTA3) as GPIO output HIGH, SDA (PTA2) as GPIO input */
    PORTA->PCR[3] = PORT_PCR_MUX(1) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;
    PORTA->PCR[2] = PORT_PCR_MUX(1) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;
    PTA->PDDR |= (1u << 3);    /* SCL = output */
    PTA->PDDR &= ~(1u << 2);   /* SDA = input */
    PTA->PSOR = (1u << 3);     /* SCL HIGH */
    i2c_gpio_delay();

    /* Clock 9 pulses on SCL to free stuck SDA */
    for (int i = 0; i < 9; i++) {
        PTA->PCOR = (1u << 3);  /* SCL LOW */
        i2c_gpio_delay();
        PTA->PSOR = (1u << 3);  /* SCL HIGH */
        i2c_gpio_delay();
        /* Stop early once SDA is released (HIGH) */
        if ((PTA->PDIR >> 2) & 1u) break;
    }

    /* Generate STOP: SDA LOW->HIGH while SCL HIGH */
    PTA->PDDR |= (1u << 2);    /* SDA = output */
    PTA->PCOR = (1u << 2);     /* SDA LOW */
    i2c_gpio_delay();
    PTA->PSOR = (1u << 3);     /* SCL HIGH */
    i2c_gpio_delay();
    PTA->PSOR = (1u << 2);     /* SDA HIGH (STOP condition) */
    i2c_gpio_delay();

    /* Release both pins back to input before switching to I2C mux */
    PTA->PDDR &= ~((1u << 2) | (1u << 3));
}

void hal_i2c_init(void)
{
    /* PORTA clock (I2C pins) */
    PCC->PCCn[PCC_PORTA_INDEX] |= PCC_PCCn_CGC_MASK;

    /* Bus-level recovery: clock out stuck SDA before enabling LPI2C */
    i2c_bus_level_recovery();

    /* LPI2C0: select SIRC (PCS=2), enable clock */
    PCC->PCCn[PCC_LPI2C0_INDEX] = PCC_PCCn_PCS(2) | PCC_PCCn_CGC_MASK;

    /* PTA2 = LPI2C0_SDA (ALT3), PTA3 = LPI2C0_SCL (ALT3) with pull-ups */
    PORTA->PCR[2] = PORT_PCR_MUX(3) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;
    PORTA->PCR[3] = PORT_PCR_MUX(3) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK;

    /* Disable master before configuring */
    LPI2C0->MCR = 0;

    /* SIRC=8MHz, PRESCALE=2 -> fclk=2MHz, SCL=2M/(18+6+2)=~77kHz */
    LPI2C0->MCFGR1 = LPI2C_MCFGR1_PRESCALE(2);
    LPI2C0->MCCR0  = LPI2C_MCCR0_CLKLO(18)
                    | LPI2C_MCCR0_CLKHI(6)
                    | LPI2C_MCCR0_SETHOLD(6)
                    | LPI2C_MCCR0_DATAVD(3);
    LPI2C0->MFCR   = LPI2C_MFCR_TXWATER(0) | LPI2C_MFCR_RXWATER(0);

    /* Enable master, debug mode */
    LPI2C0->MCR = LPI2C_MCR_MEN_MASK | LPI2C_MCR_DBGEN_MASK;
}

int hal_i2c_write(uint8_t addr7, const uint8_t *data, uint16_t len)
{
    if (!data || len == 0u) return -1;

    lpi2c_clear_status();

    /* START + slave address (write) */
    if (lpi2c_wait_tdf() != 0) goto fail;
    LPI2C0->MTDR = CMD_START | ((uint32_t)addr7 << 1u);

    /* Send all data bytes (data[0]=reg, data[1..]=payload) */
    for (uint16_t i = 0u; i < len; i++) {
        if (lpi2c_wait_tdf() != 0) goto fail;
        if ((LPI2C0->MSR & LPI2C_MSR_NDF_MASK) != 0u) goto fail;
        LPI2C0->MTDR = CMD_TX_DATA | (uint32_t)data[i];
    }

    /* STOP */
    if (lpi2c_wait_tdf() != 0) goto fail;
    LPI2C0->MTDR = CMD_STOP;

    /* Wait for STOP to complete */
    if (lpi2c_wait_stop() != 0) goto fail;

    return ((LPI2C0->MSR & LPI2C_MSR_NDF_MASK) != 0u) ? -1 : 0;

fail:
    lpi2c_bus_recovery();
    return -1;
}

int hal_i2c_read_reg(uint8_t addr7, uint8_t reg, uint8_t *data, uint16_t len)
{
    if (!data || len == 0u) return -1;

    lpi2c_clear_status();

    /* Flush RX FIFO */
    LPI2C0->MCR |= LPI2C_MCR_RRF_MASK;

    /* START + slave address (write) */
    if (lpi2c_wait_tdf() != 0) goto fail;
    LPI2C0->MTDR = CMD_START | ((uint32_t)addr7 << 1u);

    /* Register address */
    if (lpi2c_wait_tdf() != 0) goto fail;
    LPI2C0->MTDR = CMD_TX_DATA | (uint32_t)reg;

    /* Repeated START + slave address (read) */
    if (lpi2c_wait_tdf() != 0) goto fail;
    LPI2C0->MTDR = CMD_START | ((uint32_t)addr7 << 1u) | 1u;

    /* Request len bytes */
    if (lpi2c_wait_tdf() != 0) goto fail;
    LPI2C0->MTDR = CMD_RX_DATA | (uint32_t)(len - 1u);

    /* STOP after receiving */
    if (lpi2c_wait_tdf() != 0) goto fail;
    LPI2C0->MTDR = CMD_STOP;

    /* Read received bytes */
    for (uint16_t i = 0u; i < len; i++) {
        if (lpi2c_wait_rdf() != 0) goto fail;
        uint32_t rd = LPI2C0->MRDR;
        if ((rd & LPI2C_MRDR_RXEMPTY_MASK) != 0u) goto fail;
        data[i] = (uint8_t)(rd & LPI2C_MRDR_DATA_MASK);
    }

    return ((LPI2C0->MSR & LPI2C_MSR_NDF_MASK) != 0u) ? -1 : 0;

fail:
    lpi2c_bus_recovery();
    return -1;
}

int hal_i2c_probe(uint8_t addr7)
{
    lpi2c_clear_status();

    if (lpi2c_wait_tdf() != 0) goto fail;
    LPI2C0->MTDR = CMD_START | ((uint32_t)addr7 << 1u);

    if (lpi2c_wait_tdf() != 0) goto fail;
    LPI2C0->MTDR = CMD_STOP;

    if (lpi2c_wait_stop() != 0) goto fail;

    return ((LPI2C0->MSR & LPI2C_MSR_NDF_MASK) != 0u) ? -1 : 0;

fail:
    lpi2c_bus_recovery();
    return -1;
}
