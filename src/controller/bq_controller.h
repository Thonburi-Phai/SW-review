/**
 * @file bq_controller.h
 * @brief BQ7690x controller: init, monitoring, protection and cell balancing
 *        for a 4S LiFePO4 pack (BQ76905/BQ76907).
 */

#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "cell_open_wire.h"

/* ========== Config Verify Error Bitfield ========== */
#define BQ_CFG_ERR_COV_THRESHOLD    (1u << 0)
#define BQ_CFG_ERR_COV_DELAY        (1u << 1)
#define BQ_CFG_ERR_COV_RECOVERY     (1u << 2)
#define BQ_CFG_ERR_CUV_THRESHOLD    (1u << 3)
#define BQ_CFG_ERR_CUV_DELAY        (1u << 4)
#define BQ_CFG_ERR_CUV_RECOVERY     (1u << 5)
#define BQ_CFG_ERR_PROT_A           (1u << 6)
#define BQ_CFG_ERR_PROT_B           (1u << 7)
#define BQ_CFG_ERR_CHG_FET_A        (1u << 8)
#define BQ_CFG_ERR_DSG_FET_A        (1u << 9)
#define BQ_CFG_ERR_FET_OPTIONS      (1u << 10)
#define BQ_CFG_ERR_VCELL_MODE       (1u << 11)
#define BQ_CFG_ERR_OCC_THRESHOLD    (1u << 12)
#define BQ_CFG_ERR_OCD1_THRESHOLD   (1u << 13)
#define BQ_CFG_ERR_OCD2_THRESHOLD   (1u << 14)
#define BQ_CFG_ERR_SCD_THRESHOLD    (1u << 15)
#define BQ_CFG_ERR_CC1_GAIN         (1u << 16)
#define BQ_CFG_ERR_CURR_GAIN        (1u << 17)
#define BQ_CFG_ERR_DA_CONFIG        (1u << 18)
#define BQ_CFG_ERR_POWER_CONFIG     (1u << 19)

/* ========== Battery Status Structure ========== */

typedef struct {
    /* Cell Voltages */
    int16_t cell_voltage_mv[4];     // Individual cell voltages in mV
    int16_t stack_voltage_mv;        // Total pack voltage in mV (signed, like cells)
    int16_t cell_min_mv;             // Minimum cell voltage
    int16_t cell_max_mv;             // Maximum cell voltage
    int16_t cell_delta_mv;           // Delta between max and min

    /* Current & Power */
    int32_t current_ma;              // Battery current in mA (CC2 gain-scaled, clips at CFG_MAX_CURRENT_A)
    int32_t raw_current_ma;          // Battery current in mA (RawCurrent 32-bit ADC, no saturation up to ±1000A @ 200µΩ)
    int32_t current_a;               // Battery current in A (for display)
    int32_t power_mw;                // Power in mW
    int32_t passed_charge_uah;       // Passed charge from coulomb counter (µAh)
    int32_t passq_raw;               // Raw PASSQ PassedCharge register (before µAh conversion)
    int32_t dod0_uah;                // DOD0 from PASSQ (µAh, signed)
    uint32_t passq_elapsed_time_sec; // Elapsed time from PASSQ (seconds, converted from 250ms ticks)

    /* Temperature */
    int16_t internal_temp_c;         // Internal die temperature (°C)
    int16_t ts1_temp_c;              // TS1 sensor temperature (°C)
    bool ts1_open_wire;              // Confirmed TS1 open wire fault (debounced)

    /* Status Flags */
    uint8_t safety_alert_a;          // Safety Alert A register
    uint8_t safety_status_a;         // Safety Status A register
    uint8_t safety_alert_b;          // Safety Alert B register
    uint8_t safety_status_b;         // Safety Status B register
    uint16_t battery_status;         // Battery Status register
    uint16_t alarm_status;           // Alarm Status register

    /* Balancing */
    uint8_t balance_active_cells;    // Active cell balancing mask

    /* Connection Status */
    bool is_connected;               // Device communication OK
    uint32_t last_update_time_ms;    // Last successful update timestamp

    /* Config Verification */
    uint32_t config_verify_errors;   // BQ_CFG_ERR_* bitfield — nonzero means readback mismatch
} bq_battery_status_t;

/* ========== Balance Debug Snapshot ========== */
/**
 * Snapshot captured at the last balance tick; field semantics match
 * _link_bms_balance_debug_t in link_bms_balance_debug.h. Read-only,
 * populated by run_balancing_task().
 */
typedef struct {
    /* Host state */
    uint8_t  enabled;                /* g_balancing_enabled at last tick */
    uint8_t  block_flags;            /* BAL_BLOCK_* bitmask */
    uint8_t  cmd_mask;               /* g_balance_prev_mask (host bit 0 = cell 1) */
    uint8_t  hw_active_mask;         /* CB_ACTIVE_CELLS readback raw (bit 1 = cell 1) */
    uint8_t  alarm_cb;               /* 1 if AlarmStatus[CB] set */
    uint8_t  cfg_bal_config_reg;     /* BalancingConfiguration (0x9020) raw */
    uint16_t tick_count;
    uint16_t write_err_count;
    /* Ticks where the commanded mask and the chip's read-back disagreed. */
    uint16_t mask_mismatch_count;
    int8_t   last_write_status;      /* bq_status_t */
    uint8_t  cfg_allow_adjacent;

    /* Inputs */
    int16_t  vmin_mv;
    int16_t  vmax_mv;
    int16_t  delta_mv;
    int32_t  current_ma;
    int16_t  internal_temp_c;
    uint8_t  vmin_idx;
    uint8_t  vmax_idx;
    uint8_t  ncell;

    /* Config */
    uint16_t cfg_start_mv;
    uint16_t cfg_stop_mv;
    uint16_t cfg_min_cell_mv;
    uint32_t cfg_max_curr_ma;
    uint8_t  cfg_max_temp_c;
    uint16_t cfg_min_start_cell_mv;
} bq_balance_debug_t;

/* ========== Controller Functions ========== */

/**
 * @brief Initialize BQ7690x controller: wake via TS pulse, probe I2C, init
 *        device, configure protections, enable FETs.
 */
void bq_controller_init(void);

/**
 * @brief Main controller run function; call periodically (e.g. every 100ms).
 * @return 1 when I2C measurements were updated this cycle (every 600ms), else 0.
 */
int bq_controller_run(void);

/**
 * @brief Get current battery status
 * @return Pointer to battery status structure
 */
const bq_battery_status_t* bq_controller_get_status(void);

/**
 * @brief Get last balance-tick debug snapshot, updated once per balance tick
 *        (~1s); used for the binary protocol debug broadcast.
 * @return Pointer to internal snapshot (always non-NULL).
 */
const bq_balance_debug_t* bq_controller_get_balance_debug(void);

/**
 * @brief Toggle the balance override cycle test (open-wire bench testing)
 *
 * Bypasses normal balancing; CB cycles through each cell sequentially
 * (1..BATTERY_CELLS, 3 s each). Cell voltages keep updating during balancing.
 * Toggled by the QQQ UART command.
 */
void bq_controller_balance_override_toggle(void);

/**
 * @brief Rewrite all protection settings to the device.
 * @return 0 on success, -1 on error
 */
int bq_controller_configure_protections(void);

/**
 * @brief Enable/disable cell balancing
 * @param enable true to enable, false to disable
 * @return 0 on success, -1 on error
 */
int bq_controller_set_balancing_enable(bool enable);

/**
 * @brief Enable/disable FETs
 *
 * Writing ON bits creates a persistent host override that disables autonomous
 * OCD/OCC protection until reboot; use bq_controller_manual_shutdown() /
 * bq_controller_restore_autonomous() instead.
 *
 * @param chg_en true to enable charge FET
 * @param dsg_en true to enable discharge FET
 * @return 0 on success, -1 on error
 */
int bq_controller_set_fets(bool chg_en, bool dsg_en);

/**
 * @brief Write FETControl (0x68) verbatim and verify the read-back.
 *
 * bq_controller_set_fets() always writes both FETs' bits, so it cannot express
 * 0x00 ("no override, everything autonomous"). A stale override survives
 * CONFIG_UPDATE, so the wire-fault path needs the real 0x00.
 *
 * @return 0 if the value was written and read back identical.
 */
int bq_controller_set_fets_raw(uint8_t fetctl);

/*
 * Host FET-OFF arbiter.
 *
 * FETControl's host OFF bits are one register, so two automatic writers would
 * clobber each other: an owner writing only its own bits clears everyone
 * else's. Each owner publishes the bits it wants held OFF; the union of all
 * owners is what reaches the chip.
 *
 * OFF bits suspend no protection and survive CONFIG_UPDATE, so an owner's
 * request costs no pack-open and needs no CONFIG_UPDATE cycle.
 */
typedef enum {
    FET_OFF_OWNER_WIRE_FAULT = 0,  /* wire_fault.c                            */
    FET_OFF_OWNER_NTC,             /* ntc_fault.c                             */
    FET_OFF_OWNER_MANUAL,          /* operator commands and manual_shutdown   */
    FET_OFF_OWNER_COUNT
} fet_off_owner_t;

/**
 * @brief Publish one owner's desired OFF bits and apply the union.
 *
 * Idempotent: writes FETControl only when the union changes, so calling this
 * every tick with an unchanged request costs no I2C.
 *
 * @param who Owner slot
 * @param off_bits BQ_FET_DSG_OFF and/or BQ_FET_CHG_OFF; 0 withdraws
 * @return 0 on success, -1 if the write or its read-back failed
 */
int bq_controller_request_fets_off(fet_off_owner_t who, uint8_t off_bits);

/**
 * @brief Write the union to FETControl unconditionally.
 *
 * Used where FETControl must be forced to a known state, which the
 * change-detecting path cannot do: a stale host force-ON leaves the union
 * unchanged and so would not be cleared.
 *
 * @return 0 on success, -1 if the write or its read-back failed
 */
int bq_controller_fets_off_resync(void);

/** @brief Union of every owner's OFF bits, as last applied. */
uint8_t bq_controller_fets_off_union(void);

/** Initialise the cell open-wire classifier and the wire-fault response. */
void bq_controller_open_wire_init(void);

/**
 * @brief Reconfigure a chip that lost its settings, WITHOUT resetting it first.
 *
 * The sequence bq_controller_bare_reset() runs after its CMD_RESET, factored out
 * so the link supervisor can recover a chip that reset by itself, where another
 * reset would destroy PASSQ a second time.
 *
 * Leaves FET_EN alone; whether the FETs may come back on is the caller's
 * decision.
 *
 * @return 0 on success.
 */
int bq_controller_reconfigure(void);

/** Read-only view of the classifier state, for telemetry. */
const ow_ctx_t *bq_controller_open_wire_ctx(void);

/**
 * @brief Turn OFF both FETs without creating a host override.
 *
 * Uses only OFF bits in FETControl, preserving autonomous mode; call
 * bq_controller_restore_autonomous() to re-enable.
 */
int bq_controller_manual_shutdown(void);

/**
 * @brief Restore autonomous FET control after manual_shutdown.
 *
 * Rewrites FET_OPTIONS with FET_EN in a CONFIG_UPDATE cycle; the device
 * auto-enables FETs if no protection fault is active.
 */
int bq_controller_restore_autonomous(void);

/**
 * @brief Full autonomous reset: CONFIG_UPDATE cycle + PROT_RECOVERY.
 *
 * Clears any host FETControl override and triggers protection re-evaluation.
 */
int bq_controller_reset_autonomous(void);

/**
 * @brief Hard reset BQ7690x IC via CMD_RESET subcmd.
 *
 * IC resets to POR state, then all protections are reconfigured with FETs OFF.
 * The OOO command re-enables FET_EN afterwards.
 */
int bq_controller_bare_reset(void);

/**
 * @brief Enter sleep mode
 * @return 0 on success, -1 on error
 */
int bq_controller_enter_sleep(void);

/**
 * @brief Enter shutdown mode
 * @return 0 on success, -1 on error
 */
int bq_controller_enter_shutdown(void);

/**
 * @brief Check if manual FET override is active (XXX safe-off).
 * @return true if FETs are manually held off
 */
bool bq_controller_is_manual_override(void);

/**
 * @brief Check if any protection fault is active.
 * @return true if fault active, false otherwise
 */
bool bq_controller_has_fault(void);

/**
 * @brief Print detailed fault information.
 */
void bq_controller_print_fault_details(void);

/**
 * @brief Clear recoverable protection faults
 * @return 0 on success, -1 on error
 */
int bq_controller_clear_faults(void);

/**
 * @brief Print battery status via UART.
 */
void bq_controller_print_status(void);

/**
 * @brief Check if ALERT pin is active (low)
 *
 * On an active ALERT: read the alarm/safety status registers for the cause,
 * handle the fault, then clear AlarmStatus to deassert the pin.
 *
 * @return true if ALERT pin is active (LOW), false otherwise
 *
 * @note ALERT pin is active LOW
 * @note Uses hal_gpio_read_alert() internally
 */
bool bq_controller_is_alert_active(void);

/**
 * @brief Handle ALERT pin interrupt: read alarm and safety status, print fault
 *        information, clear the alarm to deassert the ALERT pin.
 *
 * @return 0 on success, -1 on error
 */
int bq_controller_handle_alert(void);

/**
 * @brief Get passed charge from the BQ7690x hardware coulomb counter.
 *
 * @return Passed charge in uAh, or 0 if read failed
 *
 * @note The BQ7690x accumulates charge in both directions (charge/discharge)
 */
int32_t bq_controller_get_passed_charge_uah(void);

/**
 * @brief Reset the PASSQ coulomb counter: sends ResetPassQ to the BQ7690x and
 *        zeroes the cached values.
 *
 * @return 0 on success, -1 on error
 */
int bq_controller_reset_passq(void);

/**
 * @brief Print calibration values and current measurements: CC1 gain/offset
 *        (PASSQ integration), CC2 gain/offset (current measurement), CC1
 *        current (0x3C), raw current (0x36) and battery status.
 */
void bq_controller_print_calibration(void);

/**
 * @brief Print FET diagnostic information
 *
 * Reads and prints all FET-related registers:
 * BatteryStatus, FETControl, FETOptions, BodyDiodeThreshold.
 * Shows FET_EN, DSG/CHG state, SFET, HOST_ON/OFF bits.
 */
void bq_controller_print_fet_diag(void);

/**
 * @brief Send wake pulse via TS pin (push-pull HIGH)
 *
 * MCU drives HIGH, current flows through the diode and TS rises; the BQ7690x
 * wakes from SHUTDOWN on a rising edge >1.2V on TS.
 *
 * @param high_ms Duration of HIGH pulse in milliseconds
 */
void bq_controller_wake_pulse(uint32_t high_ms);

/**
 * @brief Start BQ sleep/wake test
 *
 * Suppresses periodic I2C reads so the BQ76907 can enter SLEEP autonomously and
 * monitors the ALERT pin for SLEEP entry and WAKE-from-current events. Auto-stops
 * on WAKE detection.
 */
void bq_controller_sleep_test_start(void);
void bq_controller_sleep_test_stop(void);
bool bq_controller_is_sleep_test_active(void);

/**
 * @brief Handle ALERT during sleep test (minimal I2C)
 * @return 1 if WAKE detected (test passed), 0 otherwise
 */
int bq_controller_sleep_test_handle_alert(void);

