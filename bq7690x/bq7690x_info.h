/**
 * @file bq7690x_info.h
 * @brief BQ7690x device information and subcommand access.
 */

#pragma once
#include "bq7690x.h"
#include "bq7690x_reg.h"

/* ========== Device Information (Read-Only) ========== */

 bq_status_t bq7690x_read_device_number(bq7690x_t *dev, uint16_t *value);

/**
 * Read firmware version (full 6 bytes)
 * @param dev Device handle
 * @param fw_buf Buffer to receive 6 bytes: [ver_word0, ver_word1, ver_word2]
 */
bq_status_t bq7690x_read_fw_version_full(bq7690x_t *dev, uint8_t *fw_buf);

/**
 * Read hardware version
 * @param dev Device handle
 * @param hw_ver Pointer to receive hardware version (U2)
 */
bq_status_t bq7690x_read_hw_version(bq7690x_t *dev, uint16_t *hw_ver);

/* ========== PASSQ (Pass Charge) Subcommand ========== */

/**
 * Read PASSQ data (12 bytes total)
 * Offset 0: Passed charge (U4)
 * Offset 4: DOD0 (I4)
 * Offset 8: Elapsed time (U4)
 *
 * @param dev Device handle
 * @param passq_buf Buffer to receive 12 bytes
 */
bq_status_t bq7690x_read_passq(bq7690x_t *dev, uint8_t *passq_buf);

/* ========== Cell Balancing Active Cells ========== */

/**
 * Read which cells are being actively balanced (bitmask)
 * @param dev Device handle
 * @param active_cells Pointer to receive bitmask (bit 0=cell1, bit 1=cell2, etc.)
 */
bq_status_t bq7690x_read_cb_active_cells(bq7690x_t *dev, uint8_t *active_cells);

/**
 * Write which cells to balance (bitmask)
 * Note: In SEALED mode this is read-only
 * @param dev Device handle
 * @param active_cells Bitmask (bit 0=cell1, bit 1=cell2, etc.)
 */
bq_status_t bq7690x_write_cb_active_cells(bq7690x_t *dev, uint8_t active_cells);

/* ========== Programmable Timer ========== */

/**
 * Read programmable timer value (U2)
 * @param dev Device handle
 * @param timer_val Pointer to receive timer value in seconds
 */
bq_status_t bq7690x_read_prog_timer(bq7690x_t *dev, uint16_t *timer_val);

/**
 * Write programmable timer value (U2)
 * Note: In SEALED mode this is read-only
 * @param dev Device handle
 * @param timer_val Timer value in seconds
 */
bq_status_t bq7690x_write_prog_timer(bq7690x_t *dev, uint16_t timer_val);

/* ========== Protection Recovery ========== */

/**
 * Read protection recovery settings (H1)
 * Bitmask to allow recovery of selected protection faults
 * @param dev Device handle
 * @param prot_recovery Pointer to receive bitmask
 */
bq_status_t bq7690x_read_prot_recovery(bq7690x_t *dev, uint8_t *prot_recovery);

/**
 * Write protection recovery settings (H1)
 * Note: In SEALED mode this is read-only
 * @param dev Device handle
 * @param prot_recovery Bitmask to enable recovery
 */
bq_status_t bq7690x_write_prot_recovery(bq7690x_t *dev, uint8_t prot_recovery);

/* ========== Security ========== */

/**
 * Write security keys to transition from SEALED to FULLACCESS mode
 * @param dev Device handle
 * @param key_buf 4-byte security key
 */
bq_status_t bq7690x_write_security_keys(bq7690x_t *dev, const uint8_t *key_buf);
