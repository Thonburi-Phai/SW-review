/**
 * @file bq7690x_balance.c
 * @brief Host-managed cell balancing for BQ76905/BQ76907.
 */

#include "bq7690x_balance.h"
#include "bq7690x_config.h"
#include <stdbool.h>

/* ========== Balancing Configuration ========== */

bq_status_t bq7690x_set_balance_config(bq7690x_t *dev, uint8_t cb_delay, uint8_t cb_no_cmd)
{
    if (!dev) return BQ_ERR_PARAM;
    if (cb_delay > 7 || cb_no_cmd > 1) return BQ_ERR_PARAM;

    /* BalancingConfiguration (0x9020), TRM SLUUCJ0 §12.2.2.1:
     * bit 0:     CB_NO_CMD - 0 = host writes accepted, 1 = writes ignored
     * bits[3:1]: CBDLY[2:0] - measurement settling delay (0 = 0ms .. 7 = 64ms)
     * bits[7:4]: reserved
     * Factory default 0x02 (CB_NO_CMD=0, CBDLY=1, 1ms settling) */
    uint8_t config = ((cb_delay & 0x07) << 1) | (cb_no_cmd & 0x01);

    bq_status_t st = bq7690x_set_config_update(dev);
    if (st != BQ_OK) return st;

    st = bq_dm_write_8(dev, BalancingConfiguration, config);

    bq_status_t st2 = bq7690x_exit_config_update(dev);
    return (st == BQ_OK) ? st2 : st;
}

bq_status_t bq7690x_balance_init(bq7690x_t *dev)
{
    if (!dev) return BQ_ERR_PARAM;

    /* CBDLY=0, host control mode: a non-zero CBDLY restarts its settling
     * countdown on every CB_ACTIVE_CELLS write. */
    bq_status_t st = bq7690x_set_balance_config(dev, 0, 0);
    if (st != BQ_OK) return st;

    /* Cell balancing temperature limits, TRM SLUUCJ0 §12.2.2.
     * MinTempThreshold (0x9021) and MaxTempThreshold (0x9022) are in 256-LSB
     * ADC code units, not °C. With an NTC thermistor, high ADC = cold.
     *   MinT: blocks if TS_ADC > MinT*256; 255 -> 65280, unreachable -> disabled
     *   MaxT: blocks if TS_ADC < MaxT*256; 0 -> 0, never below -> disabled
     * MaxInternalTemp (0x9023) is in °C. */
    bq7690x_write_min_temp_threshold(dev, 255);  /* factory default — disable cold check */
    bq7690x_write_max_temp_threshold(dev, 0);    /* factory default — disable hot check */
    bq7690x_write_max_internal_temp(dev, 85);    /* internal die limit 85°C */

    return BQ_OK;
}

/* ========== Balancing Control ========== */

bq_status_t bq7690x_balance_apply(bq7690x_t *dev, uint8_t bal_mask, int ncell)
{
    bal_mask &= 0x7F;

    /* CB_ACTIVE_CELLS bit K controls CBK; bit 0 is reserved, so the logical
     * mask shifts left by 1. */
    uint8_t hw_mask = (bal_mask << 1) & 0xFE;

    return bq7690x_write_cb_active_cells(dev, hw_mask);
}

bq_status_t bq7690x_balance_read_status(bq7690x_t *dev, uint8_t *bal_mask, int ncell)
{
    uint8_t hw_mask = 0;
    bq_status_t st = bq7690x_read_cb_active_cells(dev, &hw_mask);
    if (st == BQ_OK && bal_mask) {
        *bal_mask = (hw_mask >> 1) & 0x7F;
    }
    return st;
}

bq_status_t bq7690x_balance_stop_all(bq7690x_t *dev)
{
    return bq7690x_balance_apply(dev, 0x00, 7);
}

/* ========== Balancing Algorithm ========== */

/**
 * @brief Helper to select cells to balance (with adjacent cell protection)
 */
static uint8_t select_balance_mask_no_adjacent(const int16_t *cell_mv, int ncell,
                                               int16_t start_mv, int16_t stop_mv,
                                               uint8_t prev_mask)
{
    if (ncell <= 1 || ncell > 7) return 0;

    int16_t vmax = cell_mv[0];
    int16_t vmin = cell_mv[0];
    for (int i = 1; i < ncell; ++i) {
        if (cell_mv[i] > vmax) vmax = cell_mv[i];
        if (cell_mv[i] < vmin) vmin = cell_mv[i];
    }
    int16_t delta = vmax - vmin;

    /* Start/stop hysteresis on delta only */
    int16_t thresh = prev_mask ? stop_mv : start_mv;
    if (delta < thresh) return 0;

    /* Fresh selection every tick: candidates above vmin + stop_mv */
    uint8_t sorted[7];
    int nsorted = 0;
    for (int i = 0; i < ncell; ++i) {
        if (cell_mv[i] >= vmin + stop_mv) {
            sorted[nsorted++] = (uint8_t)i;
        }
    }
    for (int a = 0; a < nsorted - 1; ++a) {
        for (int b = a + 1; b < nsorted; ++b) {
            if (cell_mv[sorted[b]] > cell_mv[sorted[a]]) {
                uint8_t tmp = sorted[a]; sorted[a] = sorted[b]; sorted[b] = tmp;
            }
        }
    }

    /* Greedy select: highest voltage first, skip if adjacent already chosen */
    uint8_t mask = 0;
    for (int k = 0; k < nsorted; ++k) {
        int i = sorted[k];
        if ((i > 0 && (mask & (1u << (i - 1)))) ||
            (i < ncell - 1 && (mask & (1u << (i + 1))))) {
            continue;
        }
        mask |= (1u << i);
    }

    return mask;
}

/**
 * @brief Helper to select cells to balance (allowing adjacent cells)
 */
static uint8_t select_balance_mask_allow_adjacent(const int16_t *cell_mv, int ncell,
                                                  int16_t start_mv, int16_t stop_mv,
                                                  uint8_t prev_mask)
{
    if (ncell <= 1) return 0;

    int16_t vmax = cell_mv[0];
    int16_t vmin = cell_mv[0];
    for (int i = 1; i < ncell; ++i) {
        if (cell_mv[i] > vmax) vmax = cell_mv[i];
        if (cell_mv[i] < vmin) vmin = cell_mv[i];
    }
    int16_t delta = vmax - vmin;

    /* Start/stop hysteresis on delta only */
    int16_t thresh = prev_mask ? stop_mv : start_mv;
    if (delta < thresh) return 0;

    /* Fresh selection every tick */
    uint8_t mask = 0;
    for (int i = 0; i < ncell; ++i) {
        if (cell_mv[i] >= vmin + stop_mv) {
            mask |= (1u << i);
        }
    }

    return mask;
}

uint8_t bq7690x_balance_calculate_mask(const int16_t *cell_mv, int ncell,
                                       const bq_balance_config_t *config,
                                       uint8_t prev_mask)
{
    if (!cell_mv || !config || ncell < 1 || ncell > 7) return 0;
    if (config->bal_stop_mv < 1) return 0;

    if (config->allow_adjacent) {
        return select_balance_mask_allow_adjacent(cell_mv, ncell,
                                                  config->bal_start_mv,
                                                  config->bal_stop_mv,
                                                  prev_mask);
    } else {
        return select_balance_mask_no_adjacent(cell_mv, ncell,
                                               config->bal_start_mv,
                                               config->bal_stop_mv,
                                               prev_mask);
    }
}

/* ========== Balancing Tick Function ========== */

bq_status_t bq7690x_balance_tick(bq7690x_t *dev, const bq_balance_config_t *config,
                                 const bq_balance_input_t *input, uint8_t *prev_mask)
{
    if (!dev || !config || !input || !prev_mask) return BQ_ERR_PARAM;
    if (!input->cell_mv) return BQ_ERR_PARAM;

    int ncell = input->ncell;
    if (ncell < 3 || ncell > 7) return BQ_ERR_PARAM;

    /* COV/CUV do not block balancing; all other safety faults do. */
    uint8_t blocking_faults_a = input->safety_status_a & ~(BQ_SAFETY_STATUS_A_COV | BQ_SAFETY_STATUS_A_CUV);
    bool any_fault = (blocking_faults_a != 0) || (input->safety_status_b != 0);

    /* Heavy discharge blocks balancing; charge current does not. */
    bool heavy_discharge = (input->current_ma < -config->bal_max_discharge_ma);
    bool unsafe = any_fault || heavy_discharge || (input->internal_temp_c > config->bal_max_temp_c);

    /* Check if any cell is below minimum voltage */
    for (int i = 0; i < ncell; ++i) {
        if (input->cell_mv[i] < config->bal_min_cell_mv) {
            unsafe = true;
            break;
        }
    }

    /* Calculate new balancing mask */
    uint8_t new_mask = 0;
    if (!unsafe) {
        new_mask = bq7690x_balance_calculate_mask(input->cell_mv, ncell, config, *prev_mask);
    }

    /* Re-write every tick: the chip disables balancing if CB_ACTIVE_CELLS is
     * not refreshed within its 20-second watchdog. */
    bq_status_t st = bq7690x_balance_apply(dev, new_mask, ncell);
    if (st == BQ_OK) {
        *prev_mask = new_mask;
    }
    return st;
}
