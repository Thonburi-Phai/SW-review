/**
 * @file bq7690x_balance.h
 * @brief Host-managed cell balancing control.
 */

#pragma once
#include "bq7690x_reg.h"
#include "bq7690x.h"
#include <stdbool.h>

/* ========== Configuration Parameters ========== */

/**
 * @brief Cell balancing configuration structure
 */
typedef struct {
    int16_t bal_start_mv;        /**< Start balancing when delta >= this value (mV) */
    int16_t bal_stop_mv;         /**< Stop balancing when delta < this value (mV) */
    int16_t bal_min_cell_mv;     /**< Disable balancing if any cell < this value (mV) */
    int16_t bal_max_temp_c;      /**< Disable balancing if temperature > this value (°C) */
    int32_t bal_max_discharge_ma; /**< Disable balancing only when discharging > this value (mA). Charge any value OK. */
    bool allow_adjacent;         /**< Allow balancing adjacent cells */
} bq_balance_config_t;

/**
 * @brief Default balancing configuration for LiFePO4 cells
 */
#define BQ_BALANCE_CONFIG_DEFAULT { \
    .bal_start_mv = 20,         \
    .bal_stop_mv = 10,          \
    .bal_min_cell_mv = 2800,    \
    .bal_max_temp_c = 45,       \
    .bal_max_discharge_ma = 500,\
    .allow_adjacent = false     \
}

/**
 * @brief Measurement inputs for balancing decisions
 */
typedef struct {
    const int16_t *cell_mv;     /**< Array of cell voltages in mV */
    int ncell;                   /**< Number of cells */
    int32_t current_ma;          /**< Battery current in mA */
    int16_t internal_temp_c;     /**< Internal die temperature (°C) */
    uint8_t safety_status_a;     /**< Safety Status A register */
    uint8_t safety_status_b;     /**< Safety Status B register */
} bq_balance_input_t;

/* ========== Balancing Functions ========== */

/**
 * @brief Initialize balancing configuration in device
 * @param dev Device handle
 * @return BQ_OK or error code
 */
bq_status_t bq7690x_balance_init(bq7690x_t *dev);

/**
 * @brief Set balancing configuration delay
 * @param dev Device handle
 * @param cb_delay CBDLY, BalancingConfiguration bits[3:1], range 0-7
 * @param cb_no_cmd CB_NO_CMD bit (0 = host writes accepted)
 * @return BQ_OK or error code
 */
bq_status_t bq7690x_set_balance_config(bq7690x_t *dev, uint8_t cb_delay, uint8_t cb_no_cmd);

/**
 * @brief Apply cell balancing mask
 * @param dev Device handle
 * @param bal_mask Bitmask of cells to balance (bit 0 = cell 1)
 * @return BQ_OK or error code
 */
bq_status_t bq7690x_balance_apply(bq7690x_t *dev, uint8_t bal_mask, int ncell);

/**
 * @brief Read current cell balancing status
 * @param dev Device handle
 * @param bal_mask Pointer to store current balancing mask
 * @return BQ_OK or error code
 */
bq_status_t bq7690x_balance_read_status(bq7690x_t *dev, uint8_t *bal_mask, int ncell);

/**
 * @brief Calculate which cells need balancing
 * @param cell_mv Array of cell voltages in mV
 * @param ncell Number of cells
 * @param config Balancing configuration
 * @param prev_mask Previous balancing mask (for hysteresis)
 * @return Bitmask of cells to balance
 */
uint8_t bq7690x_balance_calculate_mask(const int16_t *cell_mv, int ncell,
                                       const bq_balance_config_t *config,
                                       uint8_t prev_mask);

/**
 * @brief Balancing tick; call periodically
 * @param dev Device handle
 * @param config Balancing configuration
 * @param input Cached measurement data (cell voltages, current, temp, safety)
 * @param prev_mask Pointer to previous mask (updated on success)
 * @return BQ_OK or error code
 */
bq_status_t bq7690x_balance_tick(bq7690x_t *dev, const bq_balance_config_t *config,
                                 const bq_balance_input_t *input, uint8_t *prev_mask);

/**
 * @brief Stop all cell balancing
 * @param dev Device handle
 * @return BQ_OK or error code
 */
bq_status_t bq7690x_balance_stop_all(bq7690x_t *dev);
