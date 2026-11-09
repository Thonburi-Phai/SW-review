/**
 * @file bq7690x.h
 * @brief TI BQ76905/BQ76907 driver core: register and data memory access. 3-7 series cells.
 */

#pragma once
#include "bq7690x_reg.h"

#include <stdint.h>

/**
 * @struct bq7690x_t
 * @brief Device instance: I2C address, I2C and delay callbacks, cell count.
 */
typedef struct
{
    uint8_t i2c_addr7;  /**< 7-bit I2C address (default 0x08) */

    /** I2C write callback - returns 0 on success */
    int (*write)(uint8_t addr7, const uint8_t *data, uint16_t len);

    /** I2C read callback - returns 0 on success */
    int (*read)(uint8_t addr7, uint8_t reg, uint8_t *data, uint16_t len);

    /** Millisecond delay callback */
    void (*delay_ms)(uint32_t ms);

    uint8_t cells;  /**< Number of series cells (3-7) */
} bq7690x_t;

typedef enum {
    BQ_OK = 0,                    // Success
    BQ_ERR_I2C = 1,              // Generic I2C error
    BQ_ERR_CFGUPDATE_ENTER = 2, // Failed to enter CONFIG_UPDATE
    BQ_ERR_DM_WRITE = 3,        // Failed to write Data Memory
    BQ_ERR_DM_READ = 4,         // Failed to read back
    BQ_ERR_CFGUPDATE_EXIT = 5,  // Failed to exit CONFIG_UPDATE
    BQ_ERR_VERIFY_MISMATCH = 6,  // Read-back value mismatch
    BQ_ERR_PARAM = 7
} bq_status_t;

/* -------- Low-level Register Access (Direct Commands) ------- */

/** Read 8-bit register */
bq_status_t bq_read_reg8(bq7690x_t *dev, uint8_t reg, uint8_t *value);

/** Read 16-bit register (little-endian) */
bq_status_t bq_read_reg16(bq7690x_t *dev, uint8_t reg, uint16_t *value);

/** Read 32-bit register (little-endian) */
bq_status_t bq_read_reg32(bq7690x_t *dev, uint8_t reg, uint32_t *value);

/** Write 8-bit register */
bq_status_t bq_write_reg8(bq7690x_t *dev, uint8_t reg, uint8_t val);

/** Write 16-bit register (little-endian) */
bq_status_t bq_write_reg16(bq7690x_t *dev, uint8_t reg, uint16_t val_le);

/* -------- Main API Functions -------- */
bq_status_t bq7690x_init(bq7690x_t *dev);

/* -------- Helper Functions -------- */
uint8_t dm_checksum(uint16_t subcmd, const uint8_t *data, uint8_t len);
bq_status_t bq_command_subcmd(bq7690x_t *dev, uint16_t subcmd);
unsigned char Checksum(unsigned char *ptr, unsigned char len);
bq_status_t bq_dm_read(bq7690x_t *dev, uint16_t subcmd, uint8_t *buf, uint8_t len);
bq_status_t bq_dm_write(bq7690x_t *dev, uint16_t subcmd, const uint8_t *buf, uint8_t len);

bq_status_t bq_dm_read_8(bq7690x_t *dev, uint16_t subcmd, uint8_t *out);
bq_status_t bq_dm_write_8(bq7690x_t *dev, uint16_t subcmd, uint8_t val);
bq_status_t bq_dm_read_16(bq7690x_t *dev, uint16_t subcmd, uint16_t *out);
bq_status_t bq_dm_write_16(bq7690x_t *dev, uint16_t subcmd, uint16_t val);

bq_status_t bq_command_subcmd(bq7690x_t *dev, uint16_t subcmd);

/* -------- CONFIG_UPDATE Mode Helpers -------- */
bq_status_t bq_dm_read_8_cfg(bq7690x_t *dev, uint16_t subcmd, uint8_t *value);
bq_status_t bq_dm_write_8_cfg(bq7690x_t *dev, uint16_t subcmd, uint8_t value);
bq_status_t bq_dm_read_16_cfg(bq7690x_t *dev, uint16_t subcmd, uint16_t *value);
bq_status_t bq_dm_write_16_cfg(bq7690x_t *dev, uint16_t subcmd, uint16_t value);

/* -------- Module Headers ------- */
#include "bq7690x_measurements.h"
#include "bq7690x_status.h"
#include "bq7690x_calibration.h"
#include "bq7690x_config.h"
#include "bq7690x_protection.h"
#include "bq7690x_info.h"
#include "bq7690x_balance.h"
#include "bq7690x_fet.h"
#include "bq7690x_power.h"
#include "bq7690x_power_modes.h"
#include "bq7690x_system.h"