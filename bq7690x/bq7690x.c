#include "bq7690x.h"
#include <string.h>

/* ========== Direct Command Functions ========== */

/* Read single byte from direct command register */
bq_status_t bq_read_reg8(bq7690x_t *dev, uint8_t reg, uint8_t *value)
{
    if (!dev || !value) return BQ_ERR_PARAM;
    uint8_t buf;
    if (dev->read(dev->i2c_addr7, reg, &buf, 1) != 0)
        return BQ_ERR_I2C;
    *value = buf;
    return BQ_OK;
}

/* Read 2 bytes from direct command (little-endian) */
bq_status_t bq_read_reg16(bq7690x_t *dev, uint8_t reg, uint16_t *value)
{
    if (!dev || !value) return BQ_ERR_PARAM;

    uint8_t buf[2];
    if (dev->read(dev->i2c_addr7, reg, buf, 2) != 0)
        return BQ_ERR_I2C;
    *value = (uint16_t)((buf[1] << 8) | buf[0]);
    return BQ_OK;
}

/* Read 4 bytes from direct command (little-endian) */
bq_status_t bq_read_reg32(bq7690x_t *dev, uint8_t reg, uint32_t *value)
{
    if (!dev || !value) return BQ_ERR_PARAM;

    uint8_t buf[4];
    if (dev->read(dev->i2c_addr7, reg, buf, 4) != 0)
        return BQ_ERR_I2C;

    *value = (uint32_t)((buf[3] << 24) | (buf[2] << 16) | (buf[1] << 8) | buf[0]);
    return BQ_OK;
}

/* Write single byte to direct command register */
bq_status_t bq_write_reg8(bq7690x_t *dev, uint8_t reg, uint8_t val)
{
    if (!dev) return BQ_ERR_PARAM;

    uint8_t b[2] = { reg, val };
    if (dev->write(dev->i2c_addr7, b, 2) != 0)
        return BQ_ERR_I2C;
    return BQ_OK;
}

/* Write 2 bytes in little-endian format */
bq_status_t bq_write_reg16(bq7690x_t *dev, uint8_t reg, uint16_t val_le)
{
    if (!dev) return BQ_ERR_PARAM;

    uint8_t b[3] = { reg, (uint8_t)(val_le & 0xFF), (uint8_t)(val_le >> 8) };
    if (dev->write(dev->i2c_addr7, b, 3) != 0)
        return BQ_ERR_I2C;
    return BQ_OK;
}

/* ========== Data Memory (Subcommand) Functions ========== */

/* Compute 8-bit modulo-256 checksum over: [subcmd LSB, subcmd MSB, data bytes], then bitwise invert */
uint8_t dm_checksum(uint16_t subcmd, const uint8_t *data, uint8_t len)
{
    uint32_t sum = (subcmd & 0xFF) + ((subcmd >> 8) & 0xFF);
    for (uint8_t i = 0; i < len; ++i) sum += data[i];
    return (uint8_t)(~(sum & 0xFF));
}

/* Checksum for RAM writes: 8-bit sum, bitwise inverted */
uint8_t Checksum(uint8_t *ptr, uint8_t len)
{
    uint8_t checksum = 0;

    for (uint8_t i = 0; i < len; i++)
        checksum += ptr[i];

    checksum = 0xFF & ~checksum;

    return checksum;
}


/* Read data from subcommand (Data Memory) */
bq_status_t bq_dm_read(bq7690x_t *dev, uint16_t subcmd, uint8_t *buf, uint8_t len)
{
    if (!dev || !buf || !len || len > 32) return BQ_ERR_PARAM;

    /* 1) Write subcommand address (LSB->MSB) to 0x3E/0x3F */
    if (bq_write_reg16(dev, CMD_ADDR_LO, subcmd) != BQ_OK)
        return BQ_ERR_I2C;

    /* 2) Read back transfer buffer (device auto-populates) */
    if (dev->read(dev->i2c_addr7, XFER_BUF_BASE, buf, len) != 0)
        return BQ_ERR_I2C;

    return BQ_OK;
}

/* Write data to subcommand (Data Memory) */
bq_status_t bq_dm_write(bq7690x_t *dev, uint16_t subcmd, const uint8_t *buf, uint8_t len)
{
    if (!dev || !buf || !len || len > 32) return BQ_ERR_PARAM;

    /* 1) Write subcommand address (LSB->MSB) */
    if (bq_write_reg16(dev, CMD_ADDR_LO, subcmd) != BQ_OK)
        return BQ_ERR_DM_WRITE;

    /* 2) Overwrite transfer buffer with new data */
    uint8_t frame[1 + 32]; /* reg + up to 32 bytes */
    frame[0] = XFER_BUF_BASE;
    for (uint8_t i = 0; i < len; ++i)
        frame[1 + i] = buf[i];
    if (dev->write(dev->i2c_addr7, frame, (uint16_t)(1 + len)) != 0)
        return BQ_ERR_DM_WRITE;

    /* 3) Write checksum */
    uint8_t csum = dm_checksum(subcmd, buf, len);
    if (bq_write_reg8(dev, XFER_CHECKSUM, csum) != BQ_OK)
        return BQ_ERR_DM_WRITE;

    /* 4) Write length: includes 2 bytes(0x3E..0x3F) + 2 bytes(0x60..0x61) + data length */
    uint8_t total_len = (uint8_t)(4 + len);
    if (bq_write_reg8(dev, XFER_LENGTH, total_len) != BQ_OK)
        return BQ_ERR_DM_WRITE;

    /* Device transfers immediately after length is written */
    dev->delay_ms(2);
    return BQ_OK;
}

bq_status_t bq_dm_read_8(bq7690x_t *dev, uint16_t subcmd, uint8_t *out)
{
    if (!out) return BQ_ERR_PARAM;
    int rc = bq_dm_read(dev, subcmd, out, 1);
    return rc ? BQ_ERR_DM_READ : BQ_OK;
}

bq_status_t bq_dm_write_8(bq7690x_t *dev, uint16_t subcmd, uint8_t val)
{
    int rc = bq_dm_write(dev, subcmd, &val, 1);
    return rc ? BQ_ERR_DM_WRITE : BQ_OK;
}

bq_status_t bq_dm_read_16(bq7690x_t *dev, uint16_t subcmd, uint16_t *out)
{
    if (!out) return BQ_ERR_PARAM;
    uint8_t b[2] = {0};
    int rc = bq_dm_read(dev, subcmd, b, 2);
    if (rc) return BQ_ERR_DM_READ;
    *out = (uint16_t)(b[0] | (b[1] << 8));
    return BQ_OK;
}

bq_status_t bq_dm_write_16(bq7690x_t *dev, uint16_t subcmd, uint16_t val)
{
    uint8_t b[2] = { (uint8_t)(val & 0xFF), (uint8_t)(val >> 8) };
    int rc = bq_dm_write(dev, subcmd, b, 2);
    return rc ? BQ_ERR_DM_WRITE : BQ_OK;
}

/* Execute a command-only subcommand (no data payload).
 * TRM: device processes command after subcmd address write; checksum/length not required.
 * Note: For DEEPSLEEP/SHUTDOWN subcommand you will need to call this function twice consecutively */
bq_status_t bq_command_subcmd(bq7690x_t *dev, uint16_t subcmd)
{
    if (!dev) return BQ_ERR_PARAM;

    uint8_t buf[3];
    buf[0] = CMD_ADDR_LO;
    buf[1] = (uint8_t)(subcmd & 0xFF);
    buf[2] = (uint8_t)(subcmd >> 8);
    if (dev->write(dev->i2c_addr7, buf, 3) != 0)
        return BQ_ERR_I2C;

    dev->delay_ms(2);
    return BQ_OK;
}

/* ========== CONFIG_UPDATE Mode Helpers ========== */

bq_status_t bq_dm_read_8_cfg(bq7690x_t *dev, uint16_t subcmd, uint8_t *value)
{
    if (!dev || !value) return BQ_ERR_PARAM;
    bq_status_t st = bq7690x_set_config_update(dev);
    if (st != BQ_OK) return BQ_ERR_CFGUPDATE_ENTER;
    st = bq_dm_read_8(dev, subcmd, value);
    bq_status_t st2 = bq7690x_exit_config_update(dev);
    return (st == BQ_OK) ? st2 : st;
}

bq_status_t bq_dm_write_8_cfg(bq7690x_t *dev, uint16_t subcmd, uint8_t value)
{
    if (!dev) return BQ_ERR_PARAM;
    bq_status_t st = bq7690x_set_config_update(dev);
    if (st != BQ_OK) return BQ_ERR_CFGUPDATE_ENTER;
    st = bq_dm_write_8(dev, subcmd, value);
    bq_status_t st2 = bq7690x_exit_config_update(dev);
    return (st == BQ_OK) ? st2 : st;
}

bq_status_t bq_dm_read_16_cfg(bq7690x_t *dev, uint16_t subcmd, uint16_t *value)
{
    if (!dev || !value) return BQ_ERR_PARAM;
    bq_status_t st = bq7690x_set_config_update(dev);
    if (st != BQ_OK) return BQ_ERR_CFGUPDATE_ENTER;
    st = bq_dm_read_16(dev, subcmd, value);
    bq_status_t st2 = bq7690x_exit_config_update(dev);
    return (st == BQ_OK) ? st2 : st;
}

bq_status_t bq_dm_write_16_cfg(bq7690x_t *dev, uint16_t subcmd, uint16_t value)
{
    if (!dev) return BQ_ERR_PARAM;
    bq_status_t st = bq7690x_set_config_update(dev);
    if (st != BQ_OK) return BQ_ERR_CFGUPDATE_ENTER;
    st = bq_dm_write_16(dev, subcmd, value);
    bq_status_t st2 = bq7690x_exit_config_update(dev);
    return (st == BQ_OK) ? st2 : st;
}

/* ---- Public API ---- */

/* Initialize BQ7690x device - validate required callbacks */
bq_status_t bq7690x_init(bq7690x_t *dev)
{
    if (!dev || !dev->write || !dev->read || !dev->delay_ms)
        return BQ_ERR_PARAM;
    dev->delay_ms(10);
    return BQ_OK;
}
