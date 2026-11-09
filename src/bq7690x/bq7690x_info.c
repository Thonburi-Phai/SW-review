/**
 * @file bq7690x_info.c
 * @brief BQ7690x Device Information Implementation
 */

#include "bq7690x_info.h"

/* ========== Device Information ========== */

/* Read device number (identifies BQ76905/BQ76907, etc.) */
bq_status_t bq7690x_read_device_number(bq7690x_t *dev, uint16_t *value)
{
    if (!dev || !value) return BQ_ERR_PARAM;

    uint8_t buf[3] = { CMD_ADDR_LO, DEVICE_NUMBER & 0xFF, DEVICE_NUMBER >> 8 };
    if (dev->write(dev->i2c_addr7, buf, 3) != 0)
        return BQ_ERR_I2C;

    dev->delay_ms(2);

    uint8_t data[2];
    if (dev->read(dev->i2c_addr7, XFER_BUF_BASE, data, 2) != 0)
        return BQ_ERR_I2C;

    *value = (uint16_t)((data[1] << 8) | data[0]);
    return BQ_OK;
}

bq_status_t bq7690x_read_fw_version_full(bq7690x_t *dev, uint8_t *fw_buf)
{
    if (!dev || !fw_buf) return BQ_ERR_PARAM;

    uint8_t cmd_buf[3] = { CMD_ADDR_LO, FW_VERSION & 0xFF, FW_VERSION >> 8 };
    if (dev->write(dev->i2c_addr7, cmd_buf, 3) != 0)
        return BQ_ERR_I2C;

    dev->delay_ms(2);

    if (dev->read(dev->i2c_addr7, XFER_BUF_BASE, fw_buf, 6) != 0)
        return BQ_ERR_I2C;

    return BQ_OK;
}

bq_status_t bq7690x_read_hw_version(bq7690x_t *dev, uint16_t *hw_ver)
{
    if (!dev || !hw_ver) return BQ_ERR_PARAM;

    uint8_t cmd_buf[3] = { CMD_ADDR_LO, HW_VERSION & 0xFF, HW_VERSION >> 8 };
    if (dev->write(dev->i2c_addr7, cmd_buf, 3) != 0)
        return BQ_ERR_I2C;

    dev->delay_ms(2);

    uint8_t data[2];
    if (dev->read(dev->i2c_addr7, XFER_BUF_BASE, data, 2) != 0)
        return BQ_ERR_I2C;

    *hw_ver = (uint16_t)((data[1] << 8) | data[0]);
    return BQ_OK;
}

/* ========== PASSQ ========== */

bq_status_t bq7690x_read_passq(bq7690x_t *dev, uint8_t *passq_buf)
{
    if (!dev || !passq_buf) return BQ_ERR_PARAM;

    uint8_t cmd_buf[3] = { CMD_ADDR_LO, PASSQ & 0xFF, PASSQ >> 8 };
    if (dev->write(dev->i2c_addr7, cmd_buf, 3) != 0)
        return BQ_ERR_I2C;

    dev->delay_ms(2);

    if (dev->read(dev->i2c_addr7, XFER_BUF_BASE, passq_buf, 12) != 0)
        return BQ_ERR_I2C;

    return BQ_OK;
}

/* ========== Cell Balancing Active Cells ========== */

bq_status_t bq7690x_read_cb_active_cells(bq7690x_t *dev, uint8_t *active_cells)
{
    if (!dev || !active_cells) return BQ_ERR_PARAM;

    uint8_t cmd_buf[3] = { CMD_ADDR_LO, CB_ACTIVE_CELLS & 0xFF, CB_ACTIVE_CELLS >> 8 };
    if (dev->write(dev->i2c_addr7, cmd_buf, 3) != 0)
        return BQ_ERR_I2C;

    dev->delay_ms(2);

    if (dev->read(dev->i2c_addr7, XFER_BUF_BASE, active_cells, 1) != 0)
        return BQ_ERR_I2C;

    return BQ_OK;
}

bq_status_t bq7690x_write_cb_active_cells(bq7690x_t *dev, uint8_t active_cells)
{
    if (!dev) return BQ_ERR_PARAM;

    /* Subcommand address and data must share one I2C transaction for
     * subcommand-range addresses (app note SLUAAP7). */
    uint8_t cmd_data[4] = {
        CMD_ADDR_LO,                       /* 0x3E */
        (uint8_t)(CB_ACTIVE_CELLS & 0xFF), /* 0x83 */
        (uint8_t)(CB_ACTIVE_CELLS >> 8),   /* 0x00 */
        active_cells                        /* data */
    };
    if (dev->write(dev->i2c_addr7, cmd_data, 4) != 0)
        return BQ_ERR_DM_WRITE;

    uint8_t csum = dm_checksum(CB_ACTIVE_CELLS, &active_cells, 1);
    uint8_t csum_len[3] = { XFER_CHECKSUM, csum, 0x05 };
    if (dev->write(dev->i2c_addr7, csum_len, 3) != 0)
        return BQ_ERR_DM_WRITE;

    dev->delay_ms(2);
    return BQ_OK;
}

/* ========== Programmable Timer ========== */

bq_status_t bq7690x_read_prog_timer(bq7690x_t *dev, uint16_t *timer_val)
{
    if (!dev || !timer_val) return BQ_ERR_PARAM;

    uint8_t cmd_buf[3] = { CMD_ADDR_LO, PROG_TIMER & 0xFF, PROG_TIMER >> 8 };
    if (dev->write(dev->i2c_addr7, cmd_buf, 3) != 0)
        return BQ_ERR_I2C;

    dev->delay_ms(2);

    uint8_t data[2];
    if (dev->read(dev->i2c_addr7, XFER_BUF_BASE, data, 2) != 0)
        return BQ_ERR_I2C;

    *timer_val = (uint16_t)((data[1] << 8) | data[0]);
    return BQ_OK;
}

bq_status_t bq7690x_write_prog_timer(bq7690x_t *dev, uint16_t timer_val)
{
    uint8_t buf[2] = { (uint8_t)(timer_val & 0xFF), (uint8_t)(timer_val >> 8) };
    return bq_dm_write(dev, PROG_TIMER, buf, 2);
}

/* ========== Protection Recovery ========== */

bq_status_t bq7690x_read_prot_recovery(bq7690x_t *dev, uint8_t *prot_recovery)
{
    if (!dev || !prot_recovery) return BQ_ERR_PARAM;

    uint8_t cmd_buf[3] = { CMD_ADDR_LO, PROT_RECOVERY & 0xFF, PROT_RECOVERY >> 8 };
    if (dev->write(dev->i2c_addr7, cmd_buf, 3) != 0)
        return BQ_ERR_I2C;

    dev->delay_ms(2);

    if (dev->read(dev->i2c_addr7, XFER_BUF_BASE, prot_recovery, 1) != 0)
        return BQ_ERR_I2C;

    return BQ_OK;
}

bq_status_t bq7690x_write_prot_recovery(bq7690x_t *dev, uint8_t prot_recovery)
{
    return bq_dm_write(dev, PROT_RECOVERY, &prot_recovery, 1);
}

/* ========== Security ========== */

bq_status_t bq7690x_write_security_keys(bq7690x_t *dev, const uint8_t *key_buf)
{
    return bq_dm_write(dev, SECURITY_KEYS, key_buf, 4);
}
