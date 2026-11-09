/**
 * @file bq_controller.c
 * @brief BQ7690x controller: init, monitoring, protection and cell balancing
 *        for a 4S LiFePO4 pack.
 */

#include "bq_controller.h"
#include "bq_config_4s_lfp.h"
#include "battery_soc_soh.h"
#include "hal_i2c.h"
#include "hal_uart.h"
#include "hal_gpio.h"
#include "hal_systime.h"
#include <stdlib.h>  // For abs()
#include "parameter.h"

#include "main_controller.h"
#include "cell_open_wire.h"
#include "wire_fault.h"
#include "bq_link_supervisor.h"
#include "bq7690x.h"
#include "bq7690x_measurements.h"
#include "bq7690x_status.h"
#include "bq7690x_protection.h"
#include "bq7690x_config.h"
#include "bq7690x_fet.h"
#include "bq7690x_power.h"
#include "bq7690x_balance.h"
#include "link_bms_balance_debug.h"
#include "bq7690x_info.h"
#include "bq7690x_system.h"
#include "bq7690x_calibration.h"
#include "bq7690x_power_modes.h"

// #define SILENT_MODE     // Suppress all non-essential prints for plotter

/* ========== Private Variables ========== */

static bq_battery_status_t g_battery_status = {0};
static uint8_t g_balance_prev_mask = 0;
static bool g_balancing_enabled = false;
static bool g_inhibit_fet_en = false;  // When true, strip FET_EN from FET_OPTIONS
static uint32_t g_last_monitor_tick_ms = 0;

/* Balance debug snapshot — updated each balance tick, exposed via getter */
static bq_balance_debug_t g_balance_debug = {0};
static uint8_t g_bal_config_reg_cached = 0; /* BalancingConfiguration (0x9020) — read once at init */

/* CC1/CC2 gain scaling for actual sense resistor */
static uint16_t g_cc1_gain_factory = 0;   // Factory OTP CC1 gain (per-chip, 1mΩ cal)
static uint16_t g_curr_gain_factory = 0;  // Factory OTP CC2/Curr gain (per-chip, 1mΩ cal)
static uint32_t g_cc1_ua_per_lsb = 1000;  // µA per CC1 LSB (default: 1mA at 1mΩ)
static uint32_t g_cc2_ua_per_lsb = 1000;  // µA per CC2 LSB (default: 1mA at 1mΩ)


/* Manual FET override — when set, autonomous recovery is blocked */
static bool g_manual_fet_override = false;

/* Host FET-OFF arbiter: per-owner requests and the union last written.
 * FET_OFF_APPLIED_UNKNOWN forces the next request through, used after any path
 * that writes FETControl outside the arbiter. */
#define FET_OFF_APPLIED_UNKNOWN   0xFFu
static uint8_t g_fet_off_req[FET_OFF_OWNER_COUNT] = {0};
static uint8_t g_fet_off_applied = FET_OFF_APPLIED_UNKNOWN;

#define CFG_VERIFY_MAX_RETRIES 2
static uint32_t verify_bq_config(void);

/* Defined next to param_apply_to_bq(); declared here so verify_bq_config()
 * derives the same register values the apply path writes. */
static inline uint8_t bq_ms_to_adscan(uint16_t ms);
static uint8_t runtime_occ_reg(uint16_t current_a, uint32_t rsense_uohm);
static uint8_t runtime_ocd_reg(uint16_t current_a, uint32_t rsense_uohm);
static uint8_t runtime_scd_reg(uint16_t current_a, uint32_t rsense_uohm);

/* mV -> recovery-hysteresis register encoding (0=none, 1=50, 2=100, 3=200 mV). */
static inline uint8_t hyst_mv_to_reg(uint16_t hyst_mv)
{
    return (hyst_mv >= 200) ? 3u : (hyst_mv >= 100) ? 2u : (hyst_mv >= 50) ? 1u : 0u;
}

/*
 * FETOptions as it must appear on the chip right now.
 *
 * FET_EN is firmware-owned, not a parameter: it is held clear while
 * g_inhibit_fet_en is raised and restored by whoever re-enables it. Every other
 * bit comes from the parameter cache, so PARAM_FET_OPTIONS is a partial
 * parameter -- writing its FET_EN bit has no effect; use the FET commands.
 */
static inline uint8_t fet_options_now(void)
{
    uint8_t opts = (uint8_t)(param_cache[PARAM_FET_OPTIONS].u8 & (uint8_t)~BQ_FETOPT_FET_EN);
    if (!g_inhibit_fet_en) {
        opts |= (uint8_t)BQ_FETOPT_FET_EN;
    }
    return opts;
}

/* Sleep wake test — suppresses periodic I2C so BQ can enter SLEEP */
static volatile uint8_t g_sleep_test_active = 0;

/* TS1 open wire detection state */
static int16_t  g_ts1_prev_temp_c = 0;
static bool     g_ts1_prev_valid = false;
static uint8_t  g_ts1_ow_debounce = 0;
static uint8_t  g_ts1_ow_recovery = 0;
static bool     g_die_temp_valid = false;
#define TS1_OW_DEBOUNCE_COUNT      5
#define TS1_OW_RECOVERY_COUNT      10
#define TS1_OW_RATE_MAX_C          5
#define TS1_OW_PLAUSIBILITY_C      40

// Stale data detection - track previous readings to detect ADC freeze
static int32_t g_last_current_ma = 0;
static int16_t g_last_cell_mv[4] = {0};
static int16_t g_last_stack_mv = 0;
static uint8_t g_stale_count = 0;
#define STALE_THRESHOLD 5  // Mark as stale after 5 identical readings

/* ========== BQ Device Instance ========== */

static bq7690x_t bq76907 = {
    .i2c_addr7 = DEV_ADDR,
    .write     = hal_i2c_write,
    .read      = hal_i2c_read_reg,
    .delay_ms  = delay_ms,
    .cells     = BATTERY_CELLS
};

/* ========== Device Wake & Probe ========== */

/**
 * Wake BQ76907 via TS pin using a HIGH pulse (rising edge).
 * Circuit: [MCU PA11] ->(diode)-> [TS pin]; driving HIGH pushes current into TS.
 * BQ7690x wakes from SHUTDOWN on a rising edge >1.2V on TS.
 *
 * @param high_ms Duration of HIGH pulse in milliseconds
 */
static void BQ_Wake_TS_Pulse(uint32_t high_ms) {
    // Start with pin LOW (ensure TS is low before rising edge)
    hal_gpio_ts_wake_low();
    delay_ms(5);

    // Drive HIGH -> current flows through diode -> TS rises above 1.2V (RISING EDGE)
    hal_gpio_ts_wake_high();
    delay_ms(high_ms);

    // Drive LOW (S32K116 doesn't have easy hi-Z GPIO mode, just drive low)
    hal_gpio_ts_wake_low();
}

void bq_controller_wake_pulse(uint32_t high_ms) {
    BQ_Wake_TS_Pulse(high_ms);
}

static int BQ_Probe(void) {
    for (int i = 0; i < 5; ++i) {
        if (hal_i2c_probe(DEV_ADDR) == 0)
            return 0;
        delay_ms(5);
    }
    return -1;
}

/* ========== Full Device Reset ========== */

/**
 * Full reset sequence for BQ76907, covering all security modes:
 *  1. Read security mode from BatteryStatus
 *  2. If SEALED -> unseal with default keys, then CMD_RESET
 *  3. If UNSEALED/FULLACCESS -> CMD_RESET directly
 *  4. If CMD_RESET fails -> fallback to SHUTDOWN + wake
 *  5. Verify reset by checking POR bit in BatteryStatus
 *
 * @return 0 on success, -1 on failure
 */
static int BQ_FullReset(void) {
    bq_status_t st;
    uint16_t bat_stat = 0;
    uint8_t sec_mode;

    /* --- Step 0: Wake device and establish I2C communication --- */
    hal_uart_printf("[RESET] Step 0: Waking device...\r\n");

    /* Try to discharge TS1 by toggling EN_Temperature OFF briefly.
     * Without NTC, TS1 may sit above 1.2V wake threshold from pull-up. */
    hal_gpio_set_en_temperature(0);
    delay_ms(100);

    /* Try aggressive wake: multiple pulses with increasing duration */
    for (int attempt = 0; attempt < 5; attempt++) {
        /* Ensure TS_WAKE low for discharge time before rising edge */
        hal_gpio_ts_wake_low();
        delay_ms(50);

        BQ_Wake_TS_Pulse(100);
        delay_ms(20);
        BQ_Wake_TS_Pulse(100);
        delay_ms(50);

        if (BQ_Probe() == 0) {
            hal_uart_printf("[RESET] Device responding on I2C (attempt %d)\r\n", attempt + 1);
            break;
        }
        hal_uart_printf("[RESET] No I2C response (attempt %d), trying longer pulse...\r\n", attempt + 1);

        /* Longer pulse to wake from deeper power states */
        BQ_Wake_TS_Pulse(500);
        delay_ms(100);
    }

    /* Re-enable temperature sensing */
    hal_gpio_set_en_temperature(1);

    /* Final check after all wake attempts */
    if (BQ_Probe() != 0) {
        hal_uart_printf("[RESET] WARN: Device still not on I2C after wake\r\n");
        hal_uart_printf("[RESET] I2C probe of 0x%02X failed\r\n", DEV_ADDR);
    }

    /* Init device callbacks */
    bq7690x_init(&bq76907);

    /* --- Step 1: Read current security mode --- */
    /* BQ7690x SEC1:SEC0 encoding (TRM): 0=POR, 1=FULLACCESS, 3=SEALED */
    hal_uart_printf("[RESET] Step 1: Reading security mode...\r\n");
    st = bq_read_reg16(&bq76907, BatteryStatus, &bat_stat);
    if (st != BQ_OK) {
        hal_uart_printf("[RESET] WARN: Cannot read BatteryStatus (err=%d), trying blind reset\r\n", st);
        sec_mode = 0xFF; // unknown
    } else {
        sec_mode = (bat_stat >> 10) & 0x03;
        hal_uart_printf("[RESET] BatteryStatus=0x%04X, SecMode=%d (%s)\r\n",
                          bat_stat, sec_mode,
                          sec_mode == 0 ? "POR" :
                          sec_mode == 1 ? "FULLACCESS" :
                          sec_mode == 3 ? "SEALED" : "UNKNOWN");
    }

    /* --- Step 2: Unseal if device is in SEALED mode (SEC1:SEC0 = 3) --- */
    if (sec_mode == 3) {
        hal_uart_printf("[RESET] Step 2: Device is SEALED, sending unseal keys...\r\n");

        /* BQ7690x default full-access key: 0x04143672 (Step1=0x0414, Step2=0x3672) */
        uint8_t key[4] = { 0x14, 0x04, 0x72, 0x36 }; // little-endian
        st = bq7690x_write_security_keys(&bq76907, key);
        if (st != BQ_OK) {
            hal_uart_printf("[RESET] WARN: Security key write failed (err=%d)\r\n", st);
        }
        delay_ms(50);

        /* Verify unseal */
        st = bq_read_reg16(&bq76907, BatteryStatus, &bat_stat);
        if (st == BQ_OK) {
            sec_mode = (bat_stat >> 10) & 0x03;
            hal_uart_printf("[RESET] After unseal: SecMode=%d (%s)\r\n",
                              sec_mode,
                              sec_mode == 1 ? "FULLACCESS" :
                              sec_mode == 3 ? "SEALED" : "OTHER");
        }

        if (sec_mode == 3) {
            hal_uart_printf("[RESET] WARN: Still SEALED (keys may be non-default)\r\n");
        }
    } else {
        hal_uart_printf("[RESET] Step 2: Skipped (%s)\r\n",
                          sec_mode == 1 ? "already FULLACCESS" : "not SEALED");
    }

    /* --- Step 3: Try CMD_RESET (software reset) --- */
    hal_uart_printf("[RESET] Step 3: Sending CMD_RESET (0x0012)...\r\n");
    st = bq7690x_reset(&bq76907);
    if (st != BQ_OK) {
        hal_uart_printf("[RESET] WARN: CMD_RESET I2C failed (err=%d)\r\n", st);
    } else {
        hal_uart_printf("[RESET] CMD_RESET sent OK\r\n");
    }
    delay_ms(100); // Wait for reset to complete

    /* --- Step 4: Verify reset via POR bit --- */
    hal_uart_printf("[RESET] Step 4: Verifying reset (POR bit)...\r\n");

    /* After reset, device may need wake pulse to respond on I2C */
    BQ_Wake_TS_Pulse(30);
    delay_ms(20);

    st = bq_read_reg16(&bq76907, BatteryStatus, &bat_stat);
    if (st == BQ_OK && (bat_stat & BQ_BATT_STAT_POR)) {
        hal_uart_printf("[RESET] OK: POR bit set, device reset confirmed (BatStat=0x%04X)\r\n", bat_stat);
        return 0;
    }

    if (st == BQ_OK) {
        hal_uart_printf("[RESET] WARN: POR bit not set (BatStat=0x%04X), CMD_RESET may have been ignored\r\n", bat_stat);
    } else {
        hal_uart_printf("[RESET] WARN: Cannot read BatteryStatus after reset (err=%d)\r\n", st);
    }

    /* --- Step 5: Fallback - SHUTDOWN + wake (works in any security mode) --- */
    hal_uart_printf("[RESET] Step 5: Fallback - SHUTDOWN + wake cycle...\r\n");
    st = bq7690x_shutdown(&bq76907);
    if (st != BQ_OK) {
        hal_uart_printf("[RESET] WARN: SHUTDOWN command failed (err=%d)\r\n", st);
    } else {
        hal_uart_printf("[RESET] SHUTDOWN sent OK, waiting...\r\n");
    }
    delay_ms(200);

    /* Wake from shutdown with long TS pulse */
    hal_uart_printf("[RESET] Sending long TS wake pulse (500ms)...\r\n");
    BQ_Wake_TS_Pulse(500);
    delay_ms(50);
    BQ_Wake_TS_Pulse(30);
    delay_ms(50);

    /* Probe I2C after wake */
    if (BQ_Probe() != 0) {
        hal_uart_printf("[RESET] FAIL: Device not responding after SHUTDOWN+wake\r\n");
        return -1;
    }
    hal_uart_printf("[RESET] Device responding after SHUTDOWN+wake\r\n");

    /* Final verification */
    st = bq_read_reg16(&bq76907, BatteryStatus, &bat_stat);
    if (st == BQ_OK) {
        hal_uart_printf("[RESET] Final BatteryStatus=0x%04X, POR=%d\r\n",
                          bat_stat, (bat_stat & BQ_BATT_STAT_POR) ? 1 : 0);
    }

    hal_uart_printf("[RESET] Reset sequence complete\r\n");
    return 0;
}

/* ========== Configuration Functions ========== */

static int configure_voltage_protection(void) {
    bq_status_t st;

    // Cell Overvoltage
    st = bq7690x_write_cell_ov_threshold(&bq76907, CFG_CELL_OV_THRESHOLD_MV);
    if (st != BQ_OK) return -1;
    st = bq7690x_write_cell_ov_delay(&bq76907, CFG_CELL_OV_DELAY_ADSCAN);
    if (st != BQ_OK) return -1;
    st = bq7690x_write_cell_ov_recovery(&bq76907, CFG_CELL_OV_RECOVERY_REG);
    if (st != BQ_OK) return -1;

    // Cell Undervoltage
    st = bq7690x_write_cell_uv_threshold(&bq76907, CFG_CELL_UV_THRESHOLD_MV);
    if (st != BQ_OK) return -1;
    st = bq7690x_write_cell_uv_delay(&bq76907, CFG_CELL_UV_DELAY_ADSCAN);
    if (st != BQ_OK) return -1;
    st = bq7690x_write_cell_uv_recovery(&bq76907, CFG_CELL_UV_RECOVERY_REG);
    if (st != BQ_OK) return -1;


    return 0;
}

static int configure_current_protection(void) {
    bq_status_t st;

    // Register values computed from CFG_*_THRESHOLD_A and CFG_RSENSE_MICRO_OHM

    // OCC - Overcurrent in Charge
    st = bq7690x_write_occ_threshold(&bq76907, CFG_OCC_THRESHOLD_REG);
    if (st != BQ_OK) return -1;
    st = bq7690x_write_occ_delay(&bq76907, CFG_OCC_DELAY_REG);
    if (st != BQ_OK) return -1;

    // OCD1 - Overcurrent in Discharge 1
    st = bq7690x_write_ocd1_threshold(&bq76907, CFG_OCD1_THRESHOLD_REG);
    if (st != BQ_OK) return -1;
    st = bq7690x_write_ocd1_delay(&bq76907, CFG_OCD1_DELAY_REG);
    if (st != BQ_OK) return -1;

    // OCD2 - Overcurrent in Discharge 2
    st = bq7690x_write_ocd2_threshold(&bq76907, CFG_OCD2_THRESHOLD_REG);
    if (st != BQ_OK) return -1;
    st = bq7690x_write_ocd2_delay(&bq76907, CFG_OCD2_DELAY_REG);
    if (st != BQ_OK) return -1;

    // SCD - Short Circuit in Discharge
    st = bq7690x_write_scd_threshold(&bq76907, CFG_SCD_THRESHOLD_REG);
    if (st != BQ_OK) return -1;
    st = bq7690x_write_scd_delay(&bq76907, CFG_SCD_DELAY_REG);
    if (st != BQ_OK) return -1;

    // Latch limit - permanent lockout after N overcurrent recovery cycles
    st = bq7690x_write_latch_limit(&bq76907, CFG_LATCH_LIMIT);
    if (st != BQ_OK) return -1;

    // Recovery time
    st = bq7690x_write_recovery_time(&bq76907, CFG_CURRENT_RECOVERY_TIME_S);
    if (st != BQ_OK) return -1;

    // Read-back verification
    uint8_t rb_occ_thr = 0, rb_occ_dly = 0;
    uint8_t rb_ocd1_thr = 0, rb_ocd1_dly = 0;
    uint8_t rb_ocd2_thr = 0, rb_ocd2_dly = 0;
    uint8_t rb_scd_thr = 0, rb_scd_dly = 0;
    uint8_t rb_latch = 0, rb_recov = 0;

    bq7690x_read_occ_threshold(&bq76907, &rb_occ_thr);
    bq7690x_read_occ_delay(&bq76907, &rb_occ_dly);
    bq7690x_read_ocd1_threshold(&bq76907, &rb_ocd1_thr);
    bq7690x_read_ocd1_delay(&bq76907, &rb_ocd1_dly);
    bq7690x_read_ocd2_threshold(&bq76907, &rb_ocd2_thr);
    bq7690x_read_ocd2_delay(&bq76907, &rb_ocd2_dly);
    bq7690x_read_scd_threshold(&bq76907, &rb_scd_thr);
    bq7690x_read_scd_delay(&bq76907, &rb_scd_dly);
    bq7690x_read_latch_limit(&bq76907, &rb_latch);
    bq7690x_read_recovery_time(&bq76907, &rb_recov);

    hal_uart_printf("[PROT] Current protection readback:\r\n");
    hal_uart_printf("  OCC:  thr=0x%02X(exp 0x%02X) dly=0x%02X(exp 0x%02X)\r\n",
                      rb_occ_thr, (uint8_t)CFG_OCC_THRESHOLD_REG, rb_occ_dly, (uint8_t)CFG_OCC_DELAY_REG);
    hal_uart_printf("  OCD1: thr=0x%02X(exp 0x%02X) dly=0x%02X(exp 0x%02X)\r\n",
                      rb_ocd1_thr, (uint8_t)CFG_OCD1_THRESHOLD_REG, rb_ocd1_dly, (uint8_t)CFG_OCD1_DELAY_REG);
    hal_uart_printf("  OCD2: thr=0x%02X(exp 0x%02X) dly=0x%02X(exp 0x%02X)\r\n",
                      rb_ocd2_thr, (uint8_t)CFG_OCD2_THRESHOLD_REG, rb_ocd2_dly, (uint8_t)CFG_OCD2_DELAY_REG);
    hal_uart_printf("  SCD:  thr=0x%02X(exp 0x%02X) dly=0x%02X(exp 0x%02X)\r\n",
                      rb_scd_thr, (uint8_t)CFG_SCD_THRESHOLD_REG, rb_scd_dly, (uint8_t)CFG_SCD_DELAY_REG);
    hal_uart_printf("  Latch=%u(exp %u) Recovery=%us(exp %us)\r\n",
                      rb_latch, (uint8_t)CFG_LATCH_LIMIT, rb_recov, (uint8_t)CFG_CURRENT_RECOVERY_TIME_S);

    // Verify critical values match
    if (rb_latch != CFG_LATCH_LIMIT) {
        hal_uart_printf("[PROT] ERROR: LatchLimit mismatch!\r\n");
        return -1;
    }
    if (rb_occ_thr != CFG_OCC_THRESHOLD_REG || rb_ocd1_thr != CFG_OCD1_THRESHOLD_REG ||
        rb_ocd2_thr != CFG_OCD2_THRESHOLD_REG || rb_scd_thr != CFG_SCD_THRESHOLD_REG) {
        hal_uart_printf("[PROT] ERROR: Current threshold mismatch!\r\n");
        return -1;
    }
    if (rb_recov != CFG_CURRENT_RECOVERY_TIME_S) {
        hal_uart_printf("[PROT] ERROR: RecoveryTime mismatch!\r\n");
        return -1;
    }

    return 0;
}

static int configure_temperature_protection(void) {
    bq_status_t st;

    // OTC - Overtemperature in Charge (external thermistor - uses register value)
    st = bq7690x_write_otc_threshold(&bq76907, CFG_OTC_THRESHOLD_REG);
    if (st != BQ_OK) return -1;
    st = bq7690x_write_otc_delay(&bq76907, CFG_OTC_DELAY_MEAS);
    if (st != BQ_OK) return -1;
    st = bq7690x_write_otc_recovery(&bq76907, CFG_OTC_RECOVERY_REG);
    if (st != BQ_OK) return -1;

    // UTC - Undertemperature in Charge (external thermistor - uses register value)
    st = bq7690x_write_utc_threshold(&bq76907, CFG_UTC_THRESHOLD_REG);
    if (st != BQ_OK) return -1;
    st = bq7690x_write_utc_delay(&bq76907, CFG_UTC_DELAY_MEAS);
    if (st != BQ_OK) return -1;
    st = bq7690x_write_utc_recovery(&bq76907, CFG_UTC_RECOVERY_REG);
    if (st != BQ_OK) return -1;

    // OTD - Overtemperature in Discharge (external thermistor - uses register value)
    st = bq7690x_write_otd_threshold(&bq76907, CFG_OTD_THRESHOLD_REG);
    if (st != BQ_OK) return -1;
    st = bq7690x_write_otd_delay(&bq76907, CFG_OTD_DELAY_MEAS);
    if (st != BQ_OK) return -1;
    st = bq7690x_write_otd_recovery(&bq76907, CFG_OTD_RECOVERY_REG);
    if (st != BQ_OK) return -1;

    // UTD - Undertemperature in Discharge (external thermistor - uses register value)
    st = bq7690x_write_utd_threshold(&bq76907, CFG_UTD_THRESHOLD_REG);
    if (st != BQ_OK) return -1;
    st = bq7690x_write_utd_delay(&bq76907, CFG_UTD_DELAY_MEAS);
    if (st != BQ_OK) return -1;
    st = bq7690x_write_utd_recovery(&bq76907, CFG_UTD_RECOVERY_REG);
    if (st != BQ_OK) return -1;

    // OTINT - Internal Overtemperature (uses °C directly!)
    st = bq7690x_write_otint_threshold(&bq76907, CFG_OTINT_THRESHOLD_C);
    if (st != BQ_OK) return -1;
    st = bq7690x_write_otint_delay(&bq76907, CFG_OTINT_DELAY_MEAS);
    if (st != BQ_OK) return -1;
    st = bq7690x_write_otint_recovery(&bq76907, CFG_OTINT_RECOVERY_C);
    if (st != BQ_OK) return -1;

    return 0;
}

static int configure_protection_enables(void) {
    bq_status_t st;

    /*
     * No NTC-presence check: the enable masks and the CHG FET map are written as
     * configured. An open thermistor reads as maximum resistance, i.e. extreme
     * cold, so UTC trips and opens the CHG FET and charging stops; UTD is not in
     * CFG_DSG_FET_PROTECTIONS_A (0xF5, UTD bit clear), so discharge is
     * unaffected. The MCU reports ts1_temp_c = -999 and raises ts1_open_wire in
     * packet telemetry.
     */
    uint8_t prot_b    = CFG_ENABLED_PROTECTIONS_B;
    uint8_t chg_fet_a = CFG_CHG_FET_PROTECTIONS_A;

    // Enable protection bits
    st = bq7690x_write_enabled_protections_a(&bq76907, CFG_ENABLED_PROTECTIONS_A);
    if (st != BQ_OK) return -1;

    st = bq7690x_write_enabled_protections_b(&bq76907, prot_b);
    if (st != BQ_OK) return -1;

    // Configure which protections control FETs
    st = bq7690x_write_dsg_fet_protections_a(&bq76907, CFG_DSG_FET_PROTECTIONS_A);
    if (st != BQ_OK) return -1;

    st = bq7690x_write_chg_fet_protections_a(&bq76907, chg_fet_a);
    if (st != BQ_OK) return -1;

    st = bq7690x_write_both_fet_protections_b(&bq76907, CFG_BOTH_FET_PROTECTIONS_B);
    if (st != BQ_OK) return -1;
    return 0;
}

static int configure_general_settings(void) {
    bq_status_t st;

    // Set number of cells
    st = bq7690x_write_vcell_mode(&bq76907, BATTERY_CELLS);
    if (st != BQ_OK) return -1;

    // FET options (strip FET_EN if inhibited — bare_reset keeps FETs off)
    uint8_t fet_opts_wr = g_inhibit_fet_en
                        ? (CFG_FET_OPTIONS & ~BQ_FETOPT_FET_EN)
                        : CFG_FET_OPTIONS;
    st = bq7690x_write_fet_options(&bq76907, fet_opts_wr);
    if (st != BQ_OK) {
        hal_uart_printf("[ERROR] Failed to write FET options (status=%d)\r\n", st);
        return -1;
    }

    // Small delay to ensure write completes
    delay_ms(10);


    // Body diode threshold
    st = bq7690x_write_body_diode_threshold(&bq76907, CFG_BODY_DIODE_THRESHOLD);
    if (st != BQ_OK) {
        hal_uart_printf("[WARN] Failed to write body diode threshold\r\n");
    }

    // Charge detector time
    st = bq7690x_write_charge_detector_time(&bq76907, CFG_CHARGE_DETECTOR_TIME_100MS);
    if (st != BQ_OK) return -1;

    // Watchdog timeout
    st = bq7690x_write_host_watchdog_timeout(&bq76907, CFG_HOST_WATCHDOG_TIMEOUT_S);
    if (st != BQ_OK) return -1;

    // DA Config — ADC speed settings
    st = bq7690x_write_da_config(&bq76907, CFG_DA_CONFIG);
    if (st != BQ_OK) {
        hal_uart_printf("[ERROR] Failed to write DA Config\r\n");
        return -1;
    }

    // Power Config — ADC loop speed + sleep enable
    st = bq7690x_write_power_config(&bq76907, CFG_POWER_CONFIG);
    if (st != BQ_OK) {
        hal_uart_printf("[ERROR] Failed to write Power Config\r\n");
        return -1;
    }

    // Default Alarm Mask — ALERT pin routing at POR
    st = bq7690x_write_default_alarm_mask(&bq76907, CFG_DEFAULT_ALARM_MASK);
    if (st != BQ_OK) {
        hal_uart_printf("[WARN] Failed to write Default Alarm Mask\r\n");
    }

    return 0;
}

/**
 * Read per-chip factory CC1/CC2 gains, scale them so CC1 full-scale covers
 * MAX_CURRENT_A, and write back. After scaling, g_cc1_ua_per_lsb / g_cc2_ua_per_lsb
 * hold the µA-per-LSB conversion factors for all current/PASSQ calculations.
 */
static int configure_current_gain(void) {
    uint32_t rsense = CFG_RSENSE_MICRO_OHM;
    uint32_t max_current_a = CFG_MAX_CURRENT_A;

    // Read factory-calibrated gains (per-chip OTP, calibrated at 1mΩ)
    if (bq7690x_read_cc1_gain(&bq76907, &g_cc1_gain_factory) != BQ_OK) return -1;
    if (bq7690x_read_curr_gain(&bq76907, &g_curr_gain_factory) != BQ_OK) return -1;

    // Factory CC1 range: at factory gain, CC1 clips at ±32767.
    // 1 factory LSB = (1000/Rsense_µΩ) mA → max_mA = 32767 × 1000 / Rsense_µΩ
    uint32_t factory_range_ma = 32767UL * 1000 / rsense;
    uint32_t desired_range_ma = max_current_a * 1000;

    uint16_t cc1_gain_new, curr_gain_new;

    if (factory_range_ma >= desired_range_ma) {
        // Factory gain already covers desired range — don't modify
        cc1_gain_new = g_cc1_gain_factory;
        curr_gain_new = g_curr_gain_factory;
    } else {
        // Scale gain down so full-scale covers desired current
        cc1_gain_new = (uint16_t)((uint32_t)g_cc1_gain_factory * factory_range_ma / desired_range_ma);
        curr_gain_new = (uint16_t)((uint32_t)g_curr_gain_factory * factory_range_ma / desired_range_ma);

        if (cc1_gain_new == 0) cc1_gain_new = 1;
        if (curr_gain_new == 0) curr_gain_new = 1;

        if (bq7690x_write_cc1_gain(&bq76907, cc1_gain_new) != BQ_OK) return -1;
        if (bq7690x_write_curr_gain(&bq76907, curr_gain_new) != BQ_OK) return -1;
    }

    // Compute µA per LSB: 1 factory LSB = 1000µA at 1mΩ
    // After scaling: LSB_µA = 1000000 × factory_gain / (Rsense_µΩ × new_gain)
    g_cc1_ua_per_lsb = (uint32_t)((uint64_t)1000000 * g_cc1_gain_factory / ((uint64_t)rsense * cc1_gain_new));
    g_cc2_ua_per_lsb = (uint32_t)((uint64_t)1000000 * g_curr_gain_factory / ((uint64_t)rsense * curr_gain_new));

    hal_uart_printf("[GAIN] CC1: factory=%u scaled=%u  CC2: factory=%u scaled=%u\r\n",
                      g_cc1_gain_factory, cc1_gain_new,
                      g_curr_gain_factory, curr_gain_new);
    hal_uart_printf("[GAIN] CC1_LSB=%luuA CC2_LSB=%luuA\r\n",
                      (unsigned long)g_cc1_ua_per_lsb, (unsigned long)g_cc2_ua_per_lsb);

    return 0;
}

static int configure_power_settings(void) {
    bq_status_t st;

    // Sleep settings — SleepCurrent register is in userA, convert from mA
    uint16_t sleep_userA = (uint16_t)((uint32_t)CFG_SLEEP_CURRENT_MA * 1000 / g_cc1_ua_per_lsb);
    if (sleep_userA == 0) sleep_userA = 1;
    st = bq7690x_write_sleep_current(&bq76907, sleep_userA);
    if (st != BQ_OK) return -1;

    st = bq7690x_write_voltage_time(&bq76907, CFG_SLEEP_VOLTAGE_TIME_S);
    if (st != BQ_OK) return -1;

    st = bq7690x_write_wake_comparator_current(&bq76907, CFG_WAKE_COMPARATOR_CURR);
    if (st != BQ_OK) return -1;

    // Shutdown settings
    st = bq7690x_write_shutdown_cell_voltage(&bq76907, CFG_SHUTDOWN_CELL_MV);
    if (st != BQ_OK) return -1;

    st = bq7690x_write_shutdown_stack_voltage(&bq76907, CFG_SHUTDOWN_STACK_MV);
    if (st != BQ_OK) return -1;

    st = bq7690x_write_shutdown_temperature(&bq76907, CFG_SHUTDOWN_TEMP_C);
    if (st != BQ_OK) return -1;

    st = bq7690x_write_auto_shutdown_time(&bq76907, CFG_AUTO_SHUTDOWN_TIME_MIN);
    if (st != BQ_OK) return -1;

    return 0;
}

static int configure_alert_pin(void) {
    bq_status_t st;

    // Configure AlarmEnable register to enable ALERT pin for critical faults
    st = bq7690x_write_alarm_enable(&bq76907, CFG_ALARM_ENABLE_MASK);
    if (st != BQ_OK) {
        return -1;
    }

    // Clear any existing alarms
    uint16_t alarm_status;
    st = bq7690x_read_alarm_status(&bq76907, &alarm_status);
    if (st == BQ_OK && alarm_status != 0) {
        bq7690x_write_alarm_status(&bq76907, alarm_status);
    }

    return 0;
}

/* TS1 open wire check: range + rate-of-change + plausibility vs die temp */
static void check_ts1_open_wire(void)
{
    int16_t ts1 = g_battery_status.ts1_temp_c;
    bool ts1_valid = (ts1 != -999);
    bool suspect = false;

    /* Layer 1: range — disconnect already detected by voltage check */
    if (!ts1_valid) {
        suspect = true;
    }

    /* Layer 2: rate-of-change */
    if (!suspect && ts1_valid && g_ts1_prev_valid) {
        int16_t delta = ts1 - g_ts1_prev_temp_c;
        if (delta < 0) delta = -delta;
        if (delta > TS1_OW_RATE_MAX_C) {
            suspect = true;
        }
    }

    /* Layer 3: plausibility vs die temp */
    if (!suspect && ts1_valid && g_die_temp_valid) {
        int16_t diff = ts1 - g_battery_status.internal_temp_c;
        if (diff < 0) diff = -diff;
        if (diff > TS1_OW_PLAUSIBILITY_C) {
            suspect = true;
        }
    }

    if (suspect) {
        if (g_ts1_ow_debounce < TS1_OW_DEBOUNCE_COUNT) {
            g_ts1_ow_debounce++;
        }
        if (g_ts1_ow_debounce >= TS1_OW_DEBOUNCE_COUNT && !g_battery_status.ts1_open_wire) {
            g_battery_status.ts1_open_wire = true;
            hal_uart_printf("[OPEN WIRE] BQ TS1 confirmed\r\n");
        }
        g_ts1_ow_recovery = 0;
    } else {
        if (g_battery_status.ts1_open_wire) {
            if (g_ts1_ow_recovery < TS1_OW_RECOVERY_COUNT) {
                g_ts1_ow_recovery++;
            }
            if (g_ts1_ow_recovery >= TS1_OW_RECOVERY_COUNT) {
                g_battery_status.ts1_open_wire = false;
                g_ts1_ow_debounce = 0;
                g_ts1_ow_recovery = 0;
                hal_uart_printf("[OPEN WIRE] BQ TS1 recovered\r\n");
            }
        } else {
            g_ts1_ow_debounce = 0;
        }
    }

    g_ts1_prev_temp_c = ts1;
    g_ts1_prev_valid = ts1_valid;
}

/* ========== Monitoring Functions ========== */

/* Balance override cycle test (open-wire bench testing): bypasses the normal
 * balancing algorithm and cycles CB through each cell sequentially
 * (1..BATTERY_CELLS, BAL_OVERRIDE_STEP_MS each). Cell voltages keep updating
 * while balancing. MCU-side balance blocks (fault/current/temp/vmin) are not
 * applied; the BQ76907 still enforces its internal-temp gate and kills CB on
 * CUV/COV alerts. Off at boot; QQQ toggles it. */
#define BAL_OVERRIDE_STEP_MS 3000u
static bool     s_bal_override      = false;
static uint32_t s_bal_override_t0   = 0;
static uint8_t  s_bal_override_last = 0xFF;
static void run_balance_override_task(void);

static void update_battery_measurements(void) {
    bq_status_t st;

    /* Balance FET IR drop distorts cell voltage readings, so cell voltages are
     * updated only when balance is OFF; stack voltage and current are read
     * unconditionally below. */
    bool bal_active = (g_balance_prev_mask != 0) && !s_bal_override;

    if (!bal_active) {
        for (int i = 0; i < BATTERY_CELLS; i++) {
            st = bq7690x_read_cell_voltage(&bq76907, i + 1, &g_battery_status.cell_voltage_mv[i]);
            if (st != BQ_OK) {
                g_battery_status.is_connected = false;
                return;
            }
        }

        g_battery_status.cell_min_mv = g_battery_status.cell_voltage_mv[0];
        g_battery_status.cell_max_mv = g_battery_status.cell_voltage_mv[0];
        for (int i = 1; i < BATTERY_CELLS; i++) {
            if (g_battery_status.cell_voltage_mv[i] < g_battery_status.cell_min_mv)
                g_battery_status.cell_min_mv = g_battery_status.cell_voltage_mv[i];
            if (g_battery_status.cell_voltage_mv[i] > g_battery_status.cell_max_mv)
                g_battery_status.cell_max_mv = g_battery_status.cell_voltage_mv[i];
        }
        g_battery_status.cell_delta_mv = g_battery_status.cell_max_mv - g_battery_status.cell_min_mv;
    }

    bq7690x_read_stack_voltage(&bq76907, &g_battery_status.stack_voltage_mv);

    // Read current from CC2/Current register (16-bit, gain-scaled for actual Rsense)
    {
        int16_t curr_reg;
        st = bq7690x_read_current(&bq76907, &curr_reg);
        if (st != BQ_OK) {
            g_battery_status.is_connected = false;
            return;
        }
        g_battery_status.current_ma = (int32_t)curr_reg * (int32_t)g_cc2_ua_per_lsb / 1000;
        g_battery_status.current_a = g_battery_status.current_ma / 1000;
    }

    // Read RawCurrent (32-bit ADC, no gain scaling — measures up to AFE limit without saturation)
    {
        int32_t raw;
        st = bq7690x_read_raw_current(&bq76907, &raw);
        if (st == BQ_OK) {
            g_battery_status.raw_current_ma = RAW_CURRENT_TO_MA(raw);
        }
    }

    // Calculate power using raw current (int64 intermediate avoids overflow at 3000A × 60V)
    g_battery_status.power_mw = (int32_t)(((int64_t)g_battery_status.raw_current_ma *
                                  (int32_t)g_battery_status.stack_voltage_mv) / 1000);

    // Read temperatures
    int16_t temp_raw;
    st = bq7690x_read_internal_temperature(&bq76907, &temp_raw);
    if (st == BQ_OK) {
        g_battery_status.internal_temp_c = temp_raw;
        g_die_temp_valid = true;
    } else {
        g_battery_status.internal_temp_c = 0;
        g_die_temp_valid = false;
    }

    st = bq7690x_read_ts1_temperature(&bq76907, &temp_raw);
    if (st == BQ_OK) {
        // TS1 returns 16-bit ADC counts (not 0.1K units!)
        // LSB: 92µV (internal 1.8V reference)
        // V_ts(mV) = ADC_counts × 92 / 1000
        int32_t v_ts_mv = ((int32_t)temp_raw * 92L) / 1000L;

        // Disconnect detection: V near 1800mV = open NTC, V near 0 = shorted
        // Open circuit reads ~1779mV (raw=19346), -40°C NTC is 195kΩ → ~1619mV
        // Use 1700mV as threshold: above = open circuit, below = valid NTC
        if (v_ts_mv < 5 || v_ts_mv > 1700) {
            g_battery_status.ts1_temp_c = -999;  // Invalid/disconnect
        } else {
            // R_NTC = R_pullup × V_ts / (V_ref - V_ts)
            // R_pullup = 20000Ω, V_ref = 1800mV
            int32_t r_ntc_ohm = (20000L * v_ts_mv) / (1800L - v_ts_mv);
            int16_t temp_c = ntc_resistance_to_temp(r_ntc_ohm);
            g_battery_status.ts1_temp_c = temp_c;
        }
        check_ts1_open_wire();
    } else {
        g_battery_status.ts1_temp_c = -999;
        /* I2C failure — don't touch debounce counter (not necessarily open wire) */
    }

    // Read passed charge from coulomb counter (PASSQ: 12 bytes)
    // PASSQ accumulates CC1 samples at 250ms intervals. CC1 Gain was scaled in
    // configure_current_gain() so CC1 doesn't clip at max expected current.
    // µAh = PASSQ_raw × g_cc1_ua_per_lsb / 14400
    {
        uint8_t passq_buf[12] = {0};
        bq_status_t pq_rc = bq7690x_read_passq(&bq76907, passq_buf);
        if (pq_rc == BQ_OK) {
            int32_t passq_raw = (int32_t)(
                ((uint32_t)passq_buf[0] <<  0) |
                ((uint32_t)passq_buf[1] <<  8) |
                ((uint32_t)passq_buf[2] << 16) |
                ((uint32_t)passq_buf[3] << 24)
            );
            g_battery_status.passq_raw = passq_raw;
            g_battery_status.passed_charge_uah = (int32_t)(
                ((int64_t)passq_raw * g_cc1_ua_per_lsb) / 14400
            );

            int32_t dod0_raw = (int32_t)(
                ((uint32_t)passq_buf[4] <<  0) |
                ((uint32_t)passq_buf[5] <<  8) |
                ((uint32_t)passq_buf[6] << 16) |
                ((uint32_t)passq_buf[7] << 24)
            );
            g_battery_status.dod0_uah = (int32_t)(
                ((int64_t)dod0_raw * g_cc1_ua_per_lsb) / 14400
            );

            uint32_t elapsed_raw = (uint32_t)(
                ((uint32_t)passq_buf[8]  <<  0) |
                ((uint32_t)passq_buf[9]  <<  8) |
                ((uint32_t)passq_buf[10] << 16) |
                ((uint32_t)passq_buf[11] << 24)
            );
            g_battery_status.passq_elapsed_time_sec = elapsed_raw / 4;  // 250ms ticks → seconds
        } else {
            static uint32_t s_passq_err_cnt = 0;
            if ((++s_passq_err_cnt % 10) == 1) {
                hal_uart_printf("[PASSQ-ERR] read rc=%d\r\n", (int)pq_rc);
            }
        }
    }

    // Stale data detection - compare current readings with previous readings
    // This detects BQ76907 ADC freeze where I2C works but values don't change
    bool readings_identical = (g_battery_status.current_ma == g_last_current_ma) &&
                              (g_battery_status.cell_voltage_mv[0] == g_last_cell_mv[0]) &&
                              (g_battery_status.cell_voltage_mv[1] == g_last_cell_mv[1]) &&
                              (g_battery_status.cell_voltage_mv[2] == g_last_cell_mv[2]) &&
                              (g_battery_status.cell_voltage_mv[3] == g_last_cell_mv[3]) &&
                              (g_battery_status.stack_voltage_mv == g_last_stack_mv);

    // Only check stale once previous readings exist (not the first read)
    if (readings_identical && g_last_stack_mv != 0) {
        g_stale_count++;
        if (g_stale_count >= STALE_THRESHOLD) {
            // ADC appears frozen - mark as stale (not connected)
            g_battery_status.is_connected = false;
            /* Raise the flag only; the link supervisor owns the wake pulse and
             * its backoff. A resting pack can read identical values, so this
             * detector produces false positives. */
            g_stale_count = 0;
            return;
        }
    } else {
        g_stale_count = 0;  // Reset on good data
    }

    // Save current readings for next comparison
    g_last_current_ma = g_battery_status.current_ma;
    g_last_cell_mv[0] = g_battery_status.cell_voltage_mv[0];
    g_last_cell_mv[1] = g_battery_status.cell_voltage_mv[1];
    g_last_cell_mv[2] = g_battery_status.cell_voltage_mv[2];
    g_last_cell_mv[3] = g_battery_status.cell_voltage_mv[3];
    g_last_stack_mv = g_battery_status.stack_voltage_mv;

    g_battery_status.is_connected = true;
    g_battery_status.last_update_time_ms = systime_milli();
}

static void update_status_flags(void) {
    // Read safety alerts and status
    bq7690x_read_safety_alert_a(&bq76907, &g_battery_status.safety_alert_a);
    bq7690x_read_safety_status_a(&bq76907, &g_battery_status.safety_status_a);
    bq7690x_read_safety_alert_b(&bq76907, &g_battery_status.safety_alert_b);
    bq7690x_read_safety_status_b(&bq76907, &g_battery_status.safety_status_b);

    // Read battery status and alarm
    bq7690x_read_battery_status(&bq76907, &g_battery_status.battery_status);
    bq7690x_read_alarm_status(&bq76907, &g_battery_status.alarm_status);

    uint8_t hw_cb = 0;
    bq7690x_read_cb_active_cells(&bq76907, &hw_cb);
    g_battery_status.balance_active_cells = (hw_cb >> 1) & 0x7F;

}

/* Duty-cycle balance state.
 * Counter counts 0..cycle_ticks-1, each tick = 600ms.
 * Phase 0%:  balance OFF → cells settle for clean measurement
 * Phase 20%: measure+decide → run cell selection on clean voltages
 * Phase ON%: balance ON → apply decided mask, refresh until cycle wraps
 */
static uint16_t s_bal_cycle_counter = 0;
static uint8_t  s_bal_decided_mask  = 0;

/* ---- cell open-wire classifier + response ------------------------------- */
static ow_ctx_t    s_ow_ctx;
static ow_config_t s_ow_cfg;

void bq_controller_open_wire_init(void)
{
    ow_config_defaults(&s_ow_cfg);
    ow_init(&s_ow_ctx);
    wire_fault_init();
}

/*
 * One tick of detection and response, run after update_battery_measurements()
 * and before run_balancing_task(), which the verdict can lock out.
 */
const ow_ctx_t *bq_controller_open_wire_ctx(void) { return &s_ow_ctx; }

static void run_open_wire_task(void)
{
    ow_sample_t sm;
    bool fresh;

    if (param_cache[PARAM_OW_ENABLE].u8 == 0u) {
        return;
    }

    /* Thresholds come from the parameter cache, seeded by ow_config_defaults(). */
    s_ow_cfg.cell_min_plausible_mv = param_cache[PARAM_OW_CELL_MIN_MV].i16;
    s_ow_cfg.cell_max_plausible_mv = param_cache[PARAM_OW_CELL_MAX_MV].i16;
    s_ow_cfg.drift_eps_mv          = param_cache[PARAM_OW_DRIFT_EPS_MV].i16;
    s_ow_cfg.pair_sum_tol_mv       = param_cache[PARAM_OW_PAIR_TOL_MV].i16;
    s_ow_cfg.stack_tol_mv          = param_cache[PARAM_OW_STACK_TOL_MV].i16;

    for (uint8_t i = 0; i < OW_CELLS; i++) {
        sm.cell_mv[i] = g_battery_status.cell_voltage_mv[i];
    }
    sm.stack_mv   = g_battery_status.stack_voltage_mv;
    sm.current_ma = g_battery_status.current_ma;

    /*
     * Fresh only while balancing is off, the AFE answers, POR is clear and config
     * verify passed. A reset AFE reports its factory 7-cell map, which on the
     * interleaved wiring puts cells 2 and 4 at 0 mV while the stack still reads
     * correctly. BatteryStatus[POR] is the "configuration was lost" flag (TRM:
     * set on any full reset, cleared only on CONFIG_UPDATE exit).
     */
    fresh = (g_balance_prev_mask == 0u)
            && g_battery_status.is_connected
            && ((g_battery_status.battery_status & BQ_BATT_STAT_POR) == 0u)
            && (g_battery_status.config_verify_errors == 0u);

    if (ow_tick(&s_ow_ctx, &sm, &s_ow_cfg, fresh)) {
        const ow_verdict_t *v = &s_ow_ctx.verdict;
        hal_uart_printf("[OW] %s tap=%u cell=%u %s sum-stack=%d mV (event %u)\r\n",
                        ow_kind_name(v->kind), v->tap, v->cell,
                        v->saturated ? "settled" : "drift",
                        v->sum_minus_stack_mv, s_ow_ctx.events);
    }

    /* CONFIRMED or RECOVERY, per wire_fault_tick()'s contract; CONFIRMED alone
     * discards the classifier's clear debounce. */
    wire_fault_tick(&s_ow_ctx.verdict,
                    (s_ow_ctx.state == OW_ST_CONFIRMED) ||
                    (s_ow_ctx.state == OW_ST_RECOVERY),
                    sm.cell_mv, sm.stack_mv);
}


static void run_balancing_task(void) {
    /* A wire fault blocks balancing: it would otherwise pump a floating channel
     * for the whole grace window, and update_battery_measurements() skips cell
     * reads whenever the previous mask is non-zero, so a stale mask freezes the
     * cell voltages. */
    if (wire_fault_balancing_blocked()) {
        return;
    }

    if (s_bal_override) {
        run_balance_override_task();
        return;
    }

    int32_t bal_max_discharge_ma = (int32_t)param_cache[PARAM_BAL_MAX_DISCHARGE_A].u8 * 1000L;

    bq_balance_config_t bal_config = {
        .bal_start_mv = param_cache[PARAM_BAL_START_MV].u16,
        .bal_stop_mv = param_cache[PARAM_BAL_STOP_MV].u16,
        .bal_min_cell_mv = param_cache[PARAM_BAL_MIN_CELL_MV].u16,
        .bal_max_temp_c = param_cache[PARAM_BAL_MAX_TEMP_C].u8,
        .bal_max_discharge_ma = bal_max_discharge_ma,
        .allow_adjacent = param_cache[PARAM_BAL_ALLOW_ADJACENT].u8
    };

    uint8_t cycle_total = param_cache[PARAM_BAL_CYCLE_TICKS].u8;
    uint8_t on_pct      = param_cache[PARAM_BAL_ON_PCT].u8;
    if (cycle_total < 3) cycle_total = 3;

    uint16_t tick_measure = (uint16_t)(20u * cycle_total / 100u);
    uint16_t tick_on      = (uint16_t)((uint16_t)on_pct * cycle_total / 100u);
    if (tick_measure < 1) tick_measure = 1;
    if (tick_on <= tick_measure) tick_on = tick_measure + 1;

    // Find vmin/vmax + indices (always, for debug snapshot)
    int16_t vmax = g_battery_status.cell_voltage_mv[0];
    int16_t vmin = g_battery_status.cell_voltage_mv[0];
    uint8_t vmin_idx = 0, vmax_idx = 0;
    for (int i = 1; i < BATTERY_CELLS; ++i) {
        if (g_battery_status.cell_voltage_mv[i] > vmax) { vmax = g_battery_status.cell_voltage_mv[i]; vmax_idx = (uint8_t)i; }
        if (g_battery_status.cell_voltage_mv[i] < vmin) { vmin = g_battery_status.cell_voltage_mv[i]; vmin_idx = (uint8_t)i; }
    }
    int16_t delta = (int16_t)(vmax - vmin);

    uint8_t blocking_a = g_battery_status.safety_status_a & ~(BQ_SAFETY_STATUS_A_COV | BQ_SAFETY_STATUS_A_CUV);
    bool fault_block = (blocking_a != 0) || (g_battery_status.safety_status_b != 0);
    bool curr_block = (g_battery_status.current_ma < -bal_max_discharge_ma);
    bool temp_block = (g_battery_status.internal_temp_c > (int16_t)param_cache[PARAM_BAL_MAX_TEMP_C].u8);
    bool vcell_block = (vmin < (int16_t)param_cache[PARAM_BAL_MIN_CELL_MV].u16);
    int16_t min_start_cell_mv = (int16_t)param_cache[PARAM_BAL_MIN_START_CELL_MV].u16;
    bool vmax_block = (vmax < min_start_cell_mv);
    bool below_start = (delta < (int16_t)bal_config.bal_start_mv);

    bq_status_t st = BQ_OK;

    if (!g_balancing_enabled) {
        if (s_bal_decided_mask != 0 || g_balance_prev_mask != 0) {
            bq7690x_balance_stop_all(&bq76907);
            s_bal_decided_mask = 0;
            g_balance_prev_mask = 0;
        }
        s_bal_cycle_counter = 0;
    } else if (s_bal_cycle_counter == 0) {
        /* Phase OFF: turn off balance FETs, let cells settle */
        st = bq7690x_balance_stop_all(&bq76907);
        g_balance_prev_mask = 0;
        hal_uart_printf("[BAL] OFF d=%d I=%ldmA cyc=%d/%d duty=%d%%\r\n",
                        (int)delta, (long)g_battery_status.current_ma,
                        (int)s_bal_cycle_counter, (int)cycle_total,
                        (int)(100u - on_pct));
    } else if (s_bal_cycle_counter == tick_measure) {
        /* Phase MEASURE: voltages are clean, run cell selection */
        bool unsafe = fault_block || curr_block || temp_block || vcell_block || vmax_block;
        if (!unsafe) {
            s_bal_decided_mask = bq7690x_balance_calculate_mask(
                g_battery_status.cell_voltage_mv, BATTERY_CELLS,
                &bal_config, s_bal_decided_mask);
        } else {
            s_bal_decided_mask = 0;
        }
        hal_uart_printf("[BAL] DECIDE d=%d mask=0x%02X blk:F%d C%d T%d V%d Vm%d\r\n",
                        (int)delta, (int)s_bal_decided_mask,
                        (int)fault_block, (int)curr_block, (int)temp_block, (int)vcell_block,
                        (int)vmax_block);
    } else if (s_bal_cycle_counter == tick_on) {
        /* Phase ON: apply decided mask */
        st = bq7690x_balance_apply(&bq76907, s_bal_decided_mask, BATTERY_CELLS);
        g_balance_prev_mask = s_bal_decided_mask;
        if (st != BQ_OK) g_balance_debug.write_err_count++;
        hal_uart_printf("[BAL] ON mask=0x%02X d=%d\r\n",
                        (int)s_bal_decided_mask, (int)delta);
    } else if (s_bal_cycle_counter > tick_on && s_bal_decided_mask != 0) {
        /* Refresh: re-write mask to reset BQ76907 20s watchdog */
        st = bq7690x_balance_apply(&bq76907, s_bal_decided_mask, BATTERY_CELLS);
        if (st != BQ_OK) g_balance_debug.write_err_count++;
    }

    if (g_balancing_enabled) {
        s_bal_cycle_counter++;
        if (s_bal_cycle_counter >= cycle_total)
            s_bal_cycle_counter = 0;
    }

    /* Read back HW state */
    uint8_t hw_cb_raw = 0;
    bq7690x_read_cb_active_cells(&bq76907, &hw_cb_raw);
    uint8_t hw_cb_logical = (hw_cb_raw >> 1) & 0x7F;

    // ----- Populate debug snapshot (used by binary protocol BMS_BALANCE_DEBUG) -----
    uint8_t bf = 0;
    if (fault_block)            bf |= BAL_BLOCK_FAULT;
    if (curr_block)             bf |= BAL_BLOCK_HEAVY_CURR;
    if (temp_block)             bf |= BAL_BLOCK_HOT;
    if (vcell_block)            bf |= BAL_BLOCK_LOW_CELL;
    if (below_start)            bf |= BAL_BLOCK_BELOW_START;
    if (vmax_block)             bf |= BAL_BLOCK_LOW_VMAX;
    if (!g_balancing_enabled)   bf |= BAL_BLOCK_DISABLED;

    g_balance_debug.enabled              = g_balancing_enabled ? 1u : 0u;
    g_balance_debug.block_flags          = bf;
    g_balance_debug.cmd_mask             = g_balance_prev_mask;
    g_balance_debug.hw_active_mask       = hw_cb_logical;

    /* Commanded-but-not-active means something overrode the host: the chip kills
     * balancing by itself on a CUV/COV alert and when its balancing watchdog
     * expires. Logged on transitions only, since this runs every tick. */
    {
        static uint8_t s_last_mismatch_cmd = 0xFFu;
        static uint8_t s_last_mismatch_hw  = 0xFFu;
        if (g_balance_prev_mask != hw_cb_logical) {
            if ((g_balance_prev_mask != s_last_mismatch_cmd) ||
                (hw_cb_logical != s_last_mismatch_hw)) {
                hal_uart_printf("[BAL] mask mismatch: commanded 0x%02X, chip reports 0x%02X%s\r\n",
                                g_balance_prev_mask, hw_cb_logical,
                                (hw_cb_logical == 0u && g_balance_prev_mask != 0u)
                                    ? " (chip refused or stopped balancing)" : "");
                s_last_mismatch_cmd = g_balance_prev_mask;
                s_last_mismatch_hw  = hw_cb_logical;
            }
            g_balance_debug.mask_mismatch_count++;
        } else {
            s_last_mismatch_cmd = 0xFFu;
            s_last_mismatch_hw  = 0xFFu;
        }
    }
    g_balance_debug.alarm_cb             = (g_battery_status.alarm_status & BQ_ALARM_CB) ? 1u : 0u;
    g_balance_debug.cfg_bal_config_reg   = g_bal_config_reg_cached;
    g_balance_debug.tick_count          += 1u;
    g_balance_debug.last_write_status    = (int8_t)st;
    g_balance_debug.cfg_allow_adjacent   = bal_config.allow_adjacent ? 1u : 0u;
    g_balance_debug.vmin_mv              = vmin;
    g_balance_debug.vmax_mv              = vmax;
    g_balance_debug.delta_mv             = delta;
    g_balance_debug.current_ma           = g_battery_status.current_ma;
    g_balance_debug.internal_temp_c      = g_battery_status.internal_temp_c;
    g_balance_debug.vmin_idx             = vmin_idx;
    g_balance_debug.vmax_idx             = vmax_idx;
    g_balance_debug.ncell                = (uint8_t)BATTERY_CELLS;
    g_balance_debug.cfg_start_mv         = bal_config.bal_start_mv;
    g_balance_debug.cfg_stop_mv          = bal_config.bal_stop_mv;
    g_balance_debug.cfg_min_cell_mv      = bal_config.bal_min_cell_mv;
    g_balance_debug.cfg_max_curr_ma      = (uint32_t)bal_max_discharge_ma;
    g_balance_debug.cfg_max_temp_c       = (uint8_t)bal_config.bal_max_temp_c;
    g_balance_debug.cfg_min_start_cell_mv = (uint16_t)min_start_cell_mv;
}

static void run_balance_override_task(void) {
    uint32_t elapsed = systime_milli() - s_bal_override_t0;
    uint8_t cell = (uint8_t)((elapsed / BAL_OVERRIDE_STEP_MS) % BATTERY_CELLS); /* 0-based */
    uint8_t mask = (uint8_t)(1u << cell);

    /* Re-write every tick: applies step changes and feeds the BQ76907 20s
     * CB watchdog. */
    bq_status_t st = bq7690x_balance_apply(&bq76907, mask, BATTERY_CELLS);
    if (st != BQ_OK) g_balance_debug.write_err_count++;
    g_balance_prev_mask = mask;

    uint8_t hw_cb_raw = 0;
    bq7690x_read_cb_active_cells(&bq76907, &hw_cb_raw);
    uint8_t hw_cb_logical = (hw_cb_raw >> 1) & 0x7F;

    if (cell != s_bal_override_last) {
        s_bal_override_last = cell;
        hal_uart_printf("[BALOVR] step -> cell %d (mask=0x%02X)\r\n",
                        (int)cell + 1, (int)mask);
    }
    hal_uart_printf("[BALOVR] c%d V=%d,%d,%d,%d hw=0x%02X I=%ldmA\r\n",
                    (int)cell + 1,
                    (int)g_battery_status.cell_voltage_mv[0],
                    (int)g_battery_status.cell_voltage_mv[1],
                    (int)g_battery_status.cell_voltage_mv[2],
                    (int)g_battery_status.cell_voltage_mv[3],
                    (int)hw_cb_logical,
                    (long)g_battery_status.current_ma);

    int16_t vmax = g_battery_status.cell_voltage_mv[0];
    int16_t vmin = g_battery_status.cell_voltage_mv[0];
    uint8_t vmin_idx = 0, vmax_idx = 0;
    for (int i = 1; i < BATTERY_CELLS; ++i) {
        if (g_battery_status.cell_voltage_mv[i] > vmax) { vmax = g_battery_status.cell_voltage_mv[i]; vmax_idx = (uint8_t)i; }
        if (g_battery_status.cell_voltage_mv[i] < vmin) { vmin = g_battery_status.cell_voltage_mv[i]; vmin_idx = (uint8_t)i; }
    }

    g_balance_debug.enabled           = 1u;
    g_balance_debug.block_flags       = 0u;
    g_balance_debug.cmd_mask          = mask;
    g_balance_debug.hw_active_mask    = hw_cb_logical;
    g_balance_debug.alarm_cb          = (g_battery_status.alarm_status & BQ_ALARM_CB) ? 1u : 0u;
    g_balance_debug.tick_count       += 1u;
    g_balance_debug.last_write_status = (int8_t)st;
    g_balance_debug.vmin_mv           = vmin;
    g_balance_debug.vmax_mv           = vmax;
    g_balance_debug.delta_mv          = (int16_t)(vmax - vmin);
    g_balance_debug.current_ma        = g_battery_status.current_ma;
    g_balance_debug.internal_temp_c   = g_battery_status.internal_temp_c;
    g_balance_debug.vmin_idx          = vmin_idx;
    g_balance_debug.vmax_idx          = vmax_idx;
    g_balance_debug.ncell             = (uint8_t)BATTERY_CELLS;
}

void bq_controller_balance_override_toggle(void) {
    s_bal_override = !s_bal_override;
    if (s_bal_override) {
        s_bal_override_t0   = systime_milli();
        s_bal_override_last = 0xFF;
        hal_uart_printf("[BALOVR] ENABLED: cycle cells 1..%d, %d ms each (QQQ to stop)\r\n",
                        (int)BATTERY_CELLS, (int)BAL_OVERRIDE_STEP_MS);
    } else {
        bq7690x_balance_stop_all(&bq76907);
        g_balance_prev_mask = 0;
        s_bal_decided_mask  = 0;
        s_bal_cycle_counter = 0;
        hal_uart_printf("[BALOVR] DISABLED: normal balancing resumed\r\n");
    }
}

const bq_balance_debug_t* bq_controller_get_balance_debug(void) {
    return &g_balance_debug;
}


/* ========== Unseal Helper ========== */

static void ensure_unsealed(void) {
    uint16_t bs = 0;
    bq_read_reg16(&bq76907, BatteryStatus, &bs);
    uint8_t sec = (bs >> 10) & 0x03;
    if (sec != 3) return;

    uint8_t key[4] = { 0x14, 0x04, 0x72, 0x36 };
    bq7690x_write_security_keys(&bq76907, key);
    delay_ms(50);
    bq_read_reg16(&bq76907, BatteryStatus, &bs);
    sec = (bs >> 10) & 0x03;
    hal_uart_printf("[UNSEAL] SecMode=%d\r\n", sec);
}

/* ========== FET Enable Helper ========== */

/**
 * Read-before-toggle: send FET_ENABLE only if FET_EN is currently OFF.
 * CMD_FET_ENABLE (0x0022) is a TOGGLE, not a SET -- calling it twice disables
 * FET control.
 */
static void ensure_fet_en(void) {
    uint16_t batt_stat = 0;
    bq7690x_read_battery_status(&bq76907, &batt_stat);
    hal_uart_printf("[FET_EN] BatStat=0x%04X FET_EN=%s\r\n",
                      batt_stat, (batt_stat & BQ_BATT_STAT_FET_EN) ? "ON" : "OFF");

    if (!(batt_stat & BQ_BATT_STAT_FET_EN)) {
        // Try up to 3 times
        for (int i = 0; i < 3; i++) {
            bq7690x_fet_enable(&bq76907);
            delay_ms(50);
            bq7690x_read_battery_status(&bq76907, &batt_stat);
            hal_uart_printf("[FET_EN] Attempt %d: BatStat=0x%04X FET_EN=%s\r\n",
                              i + 1, batt_stat,
                              (batt_stat & BQ_BATT_STAT_FET_EN) ? "ON" : "OFF");
            if (batt_stat & BQ_BATT_STAT_FET_EN) break;
        }
        if (!(batt_stat & BQ_BATT_STAT_FET_EN)) {
            hal_uart_printf("[FET_EN] WARN: FET_EN still OFF after 3 attempts\r\n");
        }
    }
}

/* ========== Public Functions ========== */

void bq_controller_init(void) {
#ifndef SILENT_MODE
    hal_uart_printf("\r\n=== BQ7690x Controller Init ===\r\n");
    hal_uart_printf("[BUILD] Compiled: %s %s\r\n", __DATE__, __TIME__);
    hal_uart_printf("[CONFIG] UV Threshold: %dmV, FET Options: 0x%02X\r\n",
                      CFG_CELL_UV_THRESHOLD_MV, CFG_FET_OPTIONS);
#endif

    // Full reset: wake + probe + unseal + reset (all handled inside)
    if (BQ_FullReset() != 0) {
#ifndef SILENT_MODE
        hal_uart_printf("[FAIL] Full reset failed\r\n");
#endif
        return;
    }

    // Read device info
    uint16_t dev_num = 0;
    bq7690x_read_device_number(&bq76907, &dev_num);

    // Configure protections with readback verify + retry
    {
        uint32_t cfg_err = 0;
        for (int attempt = 0; attempt <= CFG_VERIFY_MAX_RETRIES; attempt++) {
            if (bq_controller_configure_protections() != 0) {
                hal_uart_printf("[INIT] configure_protections failed (attempt %d)\r\n", attempt);
                if (attempt == CFG_VERIFY_MAX_RETRIES) {
                    g_battery_status.config_verify_errors = 0xFFFFFFFF;
                    return;
                }
                continue;
            }
            cfg_err = verify_bq_config();
            if (cfg_err == 0) break;
            if (attempt < CFG_VERIFY_MAX_RETRIES) {
                hal_uart_printf("[INIT] Config verify failed (0x%08lX), retry %d\r\n",
                                  (unsigned long)cfg_err, attempt + 1);
            }
        }
        g_battery_status.config_verify_errors = cfg_err;
        if (cfg_err != 0) {
            hal_uart_printf("[INIT] WARNING: Config verify errors persist: 0x%08lX\r\n",
                              (unsigned long)cfg_err);
        }
    }

    // Configure ALERT pin
    if (configure_alert_pin() != 0) {
        return;
    }

    // Initialize balancing
    if (bq7690x_balance_init(&bq76907) == BQ_OK) {
        g_balancing_enabled = true;  /* Enabled by default after successful balance_init */
        // Verify all balancing config registers
        uint8_t bal_cfg = 0xFF, min_t = 0, max_t = 0;
        int8_t max_int = 0;
        bq_dm_read_8_cfg(&bq76907, BalancingConfiguration, &bal_cfg);
        bq_dm_read_8_cfg(&bq76907, MinTempThreshold, &min_t);
        bq_dm_read_8_cfg(&bq76907, MaxTempThreshold, &max_t);
        bq_dm_read_8_cfg(&bq76907, MaxInternalTemp, (uint8_t*)&max_int);
        g_bal_config_reg_cached = bal_cfg;
        hal_uart_printf("[BAL] Config=0x%02X (CBDLY=%d CB_NO_CMD=%d) MinT=%d MaxT=%d MaxIntT=%d\r\n",
                          bal_cfg, (bal_cfg >> 1) & 0x07, bal_cfg & 0x01,
                          (int)min_t, (int)max_t, (int)max_int);
    }

    // Reset PASSQ counter — destroyed by CONFIG_UPDATE cycles during init.
    // SOC module will start delta tracking from zero after this.
    bq7690x_reset_pass_q(&bq76907);
    hal_uart_printf("[INIT] PASSQ reset (post-config)\r\n");

    // Populate g_battery_status (cells, current, PASSQ=0) before SOC init reads it.
    update_battery_measurements();
    update_status_flags();

    // FET_EN: With FET_OPTIONS.FET_EN=1, device auto-sets BatteryStatus.FET_EN
    // after CONFIG_UPDATE exit. Verify it's ON; toggle only if needed.
    ensure_fet_en();

    // Wake comparator threshold = 500 uV x setting (TRM: setting 1..10, min 1);
    // at 200 uOhm setting 1 is 2.5 A in either direction, too coarse for this
    // shunt, so BQ SLEEP stays disabled and the MCU's VLPS handles low-power idle.
    bq7690x_sleep_disable(&bq76907);

    // CB_ACTIVE_CELLS, PROT_RECOVERY, CMD_FET_ENABLE are restricted in
    // SEALED mode. Unseal so runtime operations work.
    ensure_unsealed();

    // Do NOT send a host FET command (bq_controller_set_fets) here: a host
    // FETControl write overrides ALL autonomous protection FET control for the FET
    // it forces, not just the overcurrent protections. For DSG that is CUV, SCD,
    // OCD1, OCD2, OTD and OTINT; for CHG it is COV, OCC, UTC, OTC and OTINT. An
    // override also survives CONFIG_UPDATE. With FET_EN=1 and no override, the
    // autonomous controller manages the FETs from protection state.

    bq_controller_open_wire_init();
    bq_link_supervisor_init();

    // Log final FET state for verification
    uint16_t init_bs = 0;
    bq7690x_read_battery_status(&bq76907, &init_bs);
    hal_uart_printf("[INIT] Final BatStat=0x%04X FET_EN=%s DSG=%s CHG=%s SecMode=%d\r\n",
                      init_bs,
                      (init_bs & BQ_BATT_STAT_FET_EN) ? "ON" : "OFF",
                      (init_bs & BQ_BATT_STAT_DSG_FET) ? "ON" : "OFF",
                      (init_bs & BQ_BATT_STAT_CHG_FET) ? "ON" : "OFF",
                      (init_bs >> 10) & 0x03);
}

int bq_controller_run(void) {
    if (g_sleep_test_active) return 0;

    uint32_t now = systime_milli();

    // Monitor every 600ms — all I2C grouped in one wake cycle
    if ((now - g_last_monitor_tick_ms) >= 600) {
        g_last_monitor_tick_ms = now;

        update_battery_measurements();
        controller_poll_uart();
        update_status_flags();

        /* Classify and respond before balancing decides anything: the verdict
         * can lock the balancer out. */
        run_open_wire_task();

        // Balancing uses I2C too — run in same cycle (1s internal gate)
        run_balancing_task();
        return 1;
    }

    return 0;
}

const bq_battery_status_t* bq_controller_get_status(void) {
    return &g_battery_status;
}

int bq_controller_configure_protections(void) {
    if (configure_general_settings() != 0) return -1;
    if (configure_current_gain() != 0) return -1;
    if (configure_voltage_protection() != 0) return -1;
    if (configure_current_protection() != 0) return -1;
    if (configure_temperature_protection() != 0) return -1;
    if (configure_protection_enables() != 0) return -1;
    if (configure_power_settings() != 0) return -1;
    return 0;
}

/*
 * Verify the chip against the PARAMETER CACHE, not against the CFG_* macros: the
 * cache is what the chip is supposed to hold, and the macros only seed the cache
 * at boot.
 *
 * Every derived value below uses the same conversion helpers param_apply_to_bq()
 * uses, so the two cannot disagree.
 */
static uint32_t verify_bq_config(void) {
    uint32_t err = 0;
    int16_t rb16;
    uint8_t rb8;
    uint16_t rb16u;
    uint32_t rsense = param_cache[PARAM_RSENSE_MICRO_OHM].u32;

    /* Voltage protection */
    if (bq7690x_read_cell_ov_threshold(&bq76907, &rb16) != BQ_OK || rb16 != (int16_t)param_cache[PARAM_CELL_OV_THRESHOLD_MV].u16)
        err |= BQ_CFG_ERR_COV_THRESHOLD;
    if (bq7690x_read_cell_ov_delay(&bq76907, &rb8) != BQ_OK || rb8 != bq_ms_to_adscan(param_cache[PARAM_CELL_OV_DELAY_MS].u16))
        err |= BQ_CFG_ERR_COV_DELAY;
    if (bq7690x_read_cell_ov_recovery(&bq76907, &rb8) != BQ_OK || rb8 != hyst_mv_to_reg(param_cache[PARAM_CELL_OV_RECOVERY_MV].u16))
        err |= BQ_CFG_ERR_COV_RECOVERY;
    if (bq7690x_read_cell_uv_threshold(&bq76907, &rb16) != BQ_OK || rb16 != (int16_t)param_cache[PARAM_CELL_UV_THRESHOLD_MV].u16)
        err |= BQ_CFG_ERR_CUV_THRESHOLD;
    if (bq7690x_read_cell_uv_delay(&bq76907, &rb8) != BQ_OK || rb8 != bq_ms_to_adscan(param_cache[PARAM_CELL_UV_DELAY_MS].u16))
        err |= BQ_CFG_ERR_CUV_DELAY;
    if (bq7690x_read_cell_uv_recovery(&bq76907, &rb8) != BQ_OK || rb8 != hyst_mv_to_reg(param_cache[PARAM_CELL_UV_RECOVERY_MV].u16))
        err |= BQ_CFG_ERR_CUV_RECOVERY;

    /* Protection enables */
    if (bq7690x_read_enabled_protections_a(&bq76907, &rb8) != BQ_OK || rb8 != param_cache[PARAM_ENABLED_PROTECTIONS_A].u8)
        err |= BQ_CFG_ERR_PROT_A;
    if (bq7690x_read_enabled_protections_b(&bq76907, &rb8) != BQ_OK
        || rb8 != param_cache[PARAM_ENABLED_PROTECTIONS_B].u8)
        err |= BQ_CFG_ERR_PROT_B;

    /* FET protections */
    if (bq7690x_read_chg_fet_protections_a(&bq76907, &rb8) != BQ_OK
        || rb8 != param_cache[PARAM_CHG_FET_PROTECTIONS_A].u8)
        err |= BQ_CFG_ERR_CHG_FET_A;
    if (bq7690x_read_dsg_fet_protections_a(&bq76907, &rb8) != BQ_OK || rb8 != param_cache[PARAM_DSG_FET_PROTECTIONS_A].u8)
        err |= BQ_CFG_ERR_DSG_FET_A;

    /* General settings */
    uint8_t fet_opts_expect = fet_options_now();
    if (bq7690x_read_fet_options(&bq76907, &rb8) != BQ_OK || rb8 != fet_opts_expect)
        err |= BQ_CFG_ERR_FET_OPTIONS;
    if (bq7690x_read_vcell_mode(&bq76907, &rb8) != BQ_OK || rb8 != BATTERY_CELLS)
        err |= BQ_CFG_ERR_VCELL_MODE;

    /* Current protection thresholds */
    if (bq7690x_read_occ_threshold(&bq76907, &rb8) != BQ_OK || rb8 != runtime_occ_reg(param_cache[PARAM_OCC_THRESHOLD_A].u16, rsense))
        err |= BQ_CFG_ERR_OCC_THRESHOLD;
    if (bq7690x_read_ocd1_threshold(&bq76907, &rb8) != BQ_OK || rb8 != runtime_ocd_reg(param_cache[PARAM_OCD1_THRESHOLD_A].u16, rsense))
        err |= BQ_CFG_ERR_OCD1_THRESHOLD;
    if (bq7690x_read_ocd2_threshold(&bq76907, &rb8) != BQ_OK || rb8 != runtime_ocd_reg(param_cache[PARAM_OCD2_THRESHOLD_A].u16, rsense))
        err |= BQ_CFG_ERR_OCD2_THRESHOLD;
    if (bq7690x_read_scd_threshold(&bq76907, &rb8) != BQ_OK || rb8 != runtime_scd_reg(param_cache[PARAM_SCD_THRESHOLD_A].u16, rsense))
        err |= BQ_CFG_ERR_SCD_THRESHOLD;

    /* DA Config & Power Config */
    if (bq7690x_read_da_config(&bq76907, &rb16u) != BQ_OK || rb16u != param_cache[PARAM_DA_CONFIG].u16)
        err |= BQ_CFG_ERR_DA_CONFIG;
    if (bq7690x_read_power_config(&bq76907, &rb8) != BQ_OK || rb8 != param_cache[PARAM_POWER_CONFIG].u8)
        err |= BQ_CFG_ERR_POWER_CONFIG;

    if (err != 0) {
        hal_uart_printf("[VERIFY] Config mismatch: 0x%08lX\r\n", (unsigned long)err);
        for (int b = 0; b < 20; b++) {
            if (err & (1u << b)) {
                hal_uart_printf("[VERIFY]   bit %d FAIL\r\n", b);
            }
        }
    } else {
        hal_uart_printf("[VERIFY] All config registers OK\r\n");
    }

    return err;
}

int bq_controller_set_balancing_enable(bool enable) {
    g_balancing_enabled = enable;
    if (!enable) {
        g_balance_prev_mask = 0;
        return (bq7690x_balance_stop_all(&bq76907) == BQ_OK) ? 0 : -1;
    }
    return 0;
}

int bq_controller_set_fets(bool chg_en, bool dsg_en) {
    uint8_t fet_ctrl = 0;
    if (chg_en) fet_ctrl |= BQ_FET_CHG_ON;
    else        fet_ctrl |= BQ_FET_CHG_OFF;
    if (dsg_en) fet_ctrl |= BQ_FET_DSG_ON;
    else        fet_ctrl |= BQ_FET_DSG_OFF;

    uint8_t verify = 0;
    bq_status_t st = bq7690x_set_fet_control(&bq76907, fet_ctrl, &verify);
    if (st == BQ_ERR_VERIFY_MISMATCH) {
        hal_uart_printf("[FET] Verify mismatch: wrote=0x%02X read=0x%02X\r\n", fet_ctrl, verify);
    }
    return (st == BQ_OK) ? 0 : -1;
}

int bq_controller_set_fets_raw(uint8_t fetctl) {
    uint8_t rb = 0xFFu;
    ensure_unsealed();
    if (bq7690x_write_fet_control(&bq76907, fetctl) != BQ_OK) {
        hal_uart_printf("[FET] raw FETControl write 0x%02X failed\r\n", fetctl);
        return -1;
    }
    delay_ms(20);
    (void)bq7690x_read_fet_control(&bq76907, &rb);
    if (rb != fetctl) {
        hal_uart_printf("[FET] raw FETControl 0x%02X reads 0x%02X -- mismatch\r\n", fetctl, rb);
        return -1;
    }
    return 0;
}

static uint8_t fet_off_union(void)
{
    uint8_t want = 0;
    for (uint8_t i = 0; i < (uint8_t)FET_OFF_OWNER_COUNT; i++) {
        want |= g_fet_off_req[i];
    }
    return want;
}

int bq_controller_request_fets_off(fet_off_owner_t who, uint8_t off_bits)
{
    uint8_t want;

    if (who >= FET_OFF_OWNER_COUNT) {
        return -1;
    }
    g_fet_off_req[who] = (uint8_t)(off_bits & (uint8_t)(BQ_FET_DSG_OFF | BQ_FET_CHG_OFF));

    want = fet_off_union();
    if (want == g_fet_off_applied) {
        return 0;
    }
    if (bq_controller_set_fets_raw(want) != 0) {
        g_fet_off_applied = FET_OFF_APPLIED_UNKNOWN;
        return -1;
    }
    g_fet_off_applied = want;
    return 0;
}

int bq_controller_fets_off_resync(void)
{
    uint8_t want = fet_off_union();

    if (bq_controller_set_fets_raw(want) != 0) {
        g_fet_off_applied = FET_OFF_APPLIED_UNKNOWN;
        return -1;
    }
    g_fet_off_applied = want;
    return 0;
}

uint8_t bq_controller_fets_off_union(void)
{
    return fet_off_union();
}

int bq_controller_manual_shutdown(void) {
    hal_uart_printf("[FET] Manual shutdown: disable FET_EN then OFF...\r\n");

    uint16_t batt_stat = 0;
    bq7690x_read_battery_status(&bq76907, &batt_stat);
    hal_uart_printf("[FET] Before: BatStat=0x%04X FET_EN=%s DSG=%s CHG=%s\r\n",
                      batt_stat,
                      (batt_stat & BQ_BATT_STAT_FET_EN) ? "ON" : "OFF",
                      (batt_stat & BQ_BATT_STAT_DSG_FET) ? "ON" : "OFF",
                      (batt_stat & BQ_BATT_STAT_CHG_FET) ? "ON" : "OFF");

    ensure_unsealed();

    // Toggle FET_EN OFF so autonomous controller stops re-enabling FETs
    if (batt_stat & BQ_BATT_STAT_FET_EN) {
        bq7690x_fet_enable(&bq76907);  // toggle
        delay_ms(50);
        bq7690x_read_battery_status(&bq76907, &batt_stat);
        hal_uart_printf("[FET] FET_EN toggled: %s\r\n",
                          (batt_stat & BQ_BATT_STAT_FET_EN) ? "still ON" : "OFF");
    }

    // Now write OFF bits — without autonomous control they stay off
    bq7690x_write_fet_control(&bq76907, BQ_FET_DSG_OFF | BQ_FET_CHG_OFF);
    delay_ms(50);

    g_manual_fet_override = true;
    /* Record the hold in the arbiter so a later owner change re-applies it
     * instead of writing a union that has forgotten it. */
    g_fet_off_req[FET_OFF_OWNER_MANUAL] = (uint8_t)(BQ_FET_DSG_OFF | BQ_FET_CHG_OFF);
    g_fet_off_applied = fet_off_union();

    bq7690x_read_battery_status(&bq76907, &batt_stat);
    hal_uart_printf("[FET] After: BatStat=0x%04X FET_EN=%s DSG=%s CHG=%s\r\n",
                      batt_stat,
                      (batt_stat & BQ_BATT_STAT_FET_EN) ? "ON" : "OFF",
                      (batt_stat & BQ_BATT_STAT_DSG_FET) ? "ON" : "OFF",
                      (batt_stat & BQ_BATT_STAT_CHG_FET) ? "ON" : "OFF");

    if (batt_stat & (BQ_BATT_STAT_DSG_FET | BQ_BATT_STAT_CHG_FET)) {
        hal_uart_printf("[FET] WARN: FETs still ON after OFF command\r\n");
        return -1;
    }
    return 0;
}

int bq_controller_restore_autonomous(void) {
    hal_uart_printf("[FET] Restoring autonomous FET control...\r\n");

    uint16_t batt_stat = 0;
    bq7690x_read_battery_status(&bq76907, &batt_stat);
    hal_uart_printf("[FET] Before: BatStat=0x%04X FET_EN=%s DSG=%s CHG=%s\r\n",
                      batt_stat,
                      (batt_stat & BQ_BATT_STAT_FET_EN) ? "ON" : "OFF",
                      (batt_stat & BQ_BATT_STAT_DSG_FET) ? "ON" : "OFF",
                      (batt_stat & BQ_BATT_STAT_CHG_FET) ? "ON" : "OFF");

    ensure_unsealed();

    // Clear host OFF bits first
    bq7690x_write_fet_control(&bq76907, 0x00);
    delay_ms(50);

    // Re-enable FET_EN (autonomous mode) — toggle only if currently OFF
    bq7690x_read_battery_status(&bq76907, &batt_stat);
    if (!(batt_stat & BQ_BATT_STAT_FET_EN)) {
        bq7690x_fet_enable(&bq76907);  // toggle ON
        delay_ms(50);
    }

    g_manual_fet_override = false;
    /* The direct 0x00 above bypassed the arbiter: drop the manual hold, then
     * put any surviving owner's request back. */
    g_fet_off_req[FET_OFF_OWNER_MANUAL] = 0;
    g_fet_off_applied = FET_OFF_APPLIED_UNKNOWN;
    if (fet_off_union() != 0) {
        (void)bq_controller_fets_off_resync();
    } else {
        g_fet_off_applied = 0;
    }

    bq7690x_read_battery_status(&bq76907, &batt_stat);
    uint8_t fet_ctrl = 0;
    bq7690x_read_fet_control(&bq76907, &fet_ctrl);
    hal_uart_printf("[FET] After: BatStat=0x%04X FET_EN=%s DSG=%s CHG=%s FETCtrl=0x%02X\r\n",
                      batt_stat,
                      (batt_stat & BQ_BATT_STAT_FET_EN) ? "ON" : "OFF",
                      (batt_stat & BQ_BATT_STAT_DSG_FET) ? "ON" : "OFF",
                      (batt_stat & BQ_BATT_STAT_CHG_FET) ? "ON" : "OFF",
                      fet_ctrl);

    if (!(batt_stat & BQ_BATT_STAT_FET_EN)) {
        hal_uart_printf("[FET] WARN: FET_EN still OFF — restore failed\r\n");
        return -1;
    }

    if (!(batt_stat & (BQ_BATT_STAT_DSG_FET | BQ_BATT_STAT_CHG_FET))) {
        hal_uart_printf("[FET] NOTE: FETs still OFF — protection fault may be active\r\n");
    }

    return 0;
}

int bq_controller_reset_autonomous(void) {
    hal_uart_printf("[FET] Full autonomous reset via CONFIG_UPDATE + PROT_RECOVERY...\r\n");

    uint16_t batt_stat = 0;
    bq7690x_read_battery_status(&bq76907, &batt_stat);
    hal_uart_printf("[FET] Before: BatStat=0x%04X FET_EN=%s DSG=%s CHG=%s\r\n",
                      batt_stat,
                      (batt_stat & BQ_BATT_STAT_FET_EN) ? "ON" : "OFF",
                      (batt_stat & BQ_BATT_STAT_DSG_FET) ? "ON" : "OFF",
                      (batt_stat & BQ_BATT_STAT_CHG_FET) ? "ON" : "OFF");

    update_battery_measurements();
    battery_soc_soh_pre_config_update();

    ensure_unsealed();

    bq_status_t st = bq7690x_set_config_update(&bq76907);
    if (st != BQ_OK) {
        hal_uart_printf("[FET] FAIL: Enter CONFIG_UPDATE returned %d\r\n", st);
        return -1;
    }
    delay_ms(100);

    st = bq7690x_exit_config_update(&bq76907);
    if (st != BQ_OK) {
        hal_uart_printf("[FET] FAIL: Exit CONFIG_UPDATE returned %d\r\n", st);
        return -1;
    }
    bq7690x_reset_pass_q(&bq76907);
    battery_soc_soh_post_config_update();
    bq7690x_sleep_disable(&bq76907);
    delay_ms(200);

    // EXIT_CONFIG_UPDATE re-seals → unseal again
    ensure_unsealed();
    ensure_fet_en();

    bq7690x_write_fet_control(&bq76907, 0x00);
    delay_ms(50);

    hal_uart_printf("[FET] Sending PROT_RECOVERY to trigger FET re-evaluation...\r\n");
    bq7690x_write_prot_recovery(&bq76907, 0xFF);
    delay_ms(500);

    g_manual_fet_override = false;
    g_fet_off_req[FET_OFF_OWNER_MANUAL] = 0;
    g_fet_off_applied = FET_OFF_APPLIED_UNKNOWN;
    if (fet_off_union() != 0) {
        (void)bq_controller_fets_off_resync();
    } else {
        g_fet_off_applied = 0;
    }

    uint8_t fet_ctrl = 0;
    bq7690x_read_battery_status(&bq76907, &batt_stat);
    bq7690x_read_fet_control(&bq76907, &fet_ctrl);
    hal_uart_printf("[FET] After: BatStat=0x%04X FET_EN=%s DSG=%s CHG=%s FETCtrl=0x%02X\r\n",
                      batt_stat,
                      (batt_stat & BQ_BATT_STAT_FET_EN) ? "ON" : "OFF",
                      (batt_stat & BQ_BATT_STAT_DSG_FET) ? "ON" : "OFF",
                      (batt_stat & BQ_BATT_STAT_CHG_FET) ? "ON" : "OFF",
                      fet_ctrl);

    if (!(batt_stat & BQ_BATT_STAT_FET_EN)) {
        hal_uart_printf("[FET] WARN: FET_EN still OFF after reset\r\n");
        return -1;
    }

    if (batt_stat & (BQ_BATT_STAT_DSG_FET | BQ_BATT_STAT_CHG_FET)) {
        hal_uart_printf("[FET] Autonomous FET control fully reset\r\n");
    } else {
        hal_uart_printf("[FET] FET_EN ON but FETs OFF — may need more time or fault active\r\n");
    }

    return 0;
}

/*
 * Reconfigure a chip that has lost its settings, without resetting it first.
 *
 * bq_controller_bare_reset() calls this after issuing CMD_RESET; the link
 * supervisor calls it on a chip that reset by itself, where another reset would
 * destroy PASSQ twice.
 *
 * g_inhibit_fet_en is held for the whole sequence so nothing re-enables the FETs
 * half-configured; whether they may come back on afterwards is the caller's
 * decision.
 */
int bq_controller_reconfigure(void) {
    g_inhibit_fet_en = true;

    {
        uint32_t cfg_err = 0;
        for (int attempt = 0; attempt <= CFG_VERIFY_MAX_RETRIES; attempt++) {
            if (bq_controller_configure_protections() != 0) {
                if (attempt == CFG_VERIFY_MAX_RETRIES) {
                    hal_uart_printf("[RECFG] FAIL: configure_protections\r\n");
                    g_inhibit_fet_en = false;
                    return -1;
                }
                continue;
            }
            /*
             * Apply the parameter cache over the top so a recovery does not
             * revert a runtime change. configure_protections() still writes the
             * CFG_* macros -- it owns the registers that are not parameterised
             * (cell count, current gain, alarm enable) -- but the cache has the
             * last word on everything it covers, including an active wire-fault
             * limp profile.
             *
             * Safe to call here: configure_protections() uses the per-register
             * _cfg wrappers, so no CONFIG_UPDATE is held open across this point.
             */
            if (param_apply_to_bq() != 0) {
                hal_uart_printf("[RECFG] WARN: param apply reported errors\r\n");
            }
            cfg_err = verify_bq_config();
            if (cfg_err == 0) break;
        }
        g_battery_status.config_verify_errors = cfg_err;
    }

    if (configure_alert_pin() != 0) {
        hal_uart_printf("[RECFG] FAIL: configure_alert_pin\r\n");
        g_inhibit_fet_en = false;
        return -1;
    }

    if (bq7690x_balance_init(&bq76907) == BQ_OK) {
        g_balancing_enabled = true;
    }

    g_inhibit_fet_en = false;

    /* PASSQ is gone after a reset or a CONFIG_UPDATE cycle; resync SOC so the
     * tracker does not read the zeroed counter as a huge jump. */
    bq7690x_reset_pass_q(&bq76907);
    battery_soc_soh_post_config_update();
    return 0;
}

int bq_controller_bare_reset(void) {
    /* Announce the self-inflicted POR so the supervisor does not treat it as a
     * fault and start a second recovery. */
    bq_link_supervisor_expect_por();
    hal_uart_printf("[RESET] Sending CMD_RESET (0x0012)...\r\n");

    /* Save pending PASSQ delta before RESET destroys the counter */
    update_battery_measurements();
    battery_soc_soh_pre_config_update();

    bq7690x_reset(&bq76907);
    delay_ms(100);

    BQ_Wake_TS_Pulse(30);
    delay_ms(20);

    uint16_t batt_stat = 0;
    bq_status_t st = bq_read_reg16(&bq76907, BatteryStatus, &batt_stat);
    if (st == BQ_OK) {
        hal_uart_printf("[RESET] BatStat=0x%04X POR=%d FET_EN=%s DSG=%s CHG=%s\r\n",
                          batt_stat,
                          (batt_stat & BQ_BATT_STAT_POR) ? 1 : 0,
                          (batt_stat & BQ_BATT_STAT_FET_EN) ? "ON" : "OFF",
                          (batt_stat & BQ_BATT_STAT_DSG_FET) ? "ON" : "OFF",
                          (batt_stat & BQ_BATT_STAT_CHG_FET) ? "ON" : "OFF");
    } else {
        hal_uart_printf("[RESET] Cannot read BatteryStatus (err=%d)\r\n", st);
    }

    if (bq_controller_reconfigure() != 0) {
        return -1;
    }

    /* PowerConfig writes SLEEP_EN=1 to DM; BQ SLEEP stays off at runtime */
    bq7690x_sleep_disable(&bq76907);

    hal_uart_printf("[RESET] Done. FETs OFF. Use OOO to enable FET_EN.\r\n");
    return 0;
}

int bq_controller_enter_sleep(void) {
    return (bq7690x_sleep_enable(&bq76907) == BQ_OK) ? 0 : -1;
}

int bq_controller_enter_shutdown(void) {
    return (bq7690x_shutdown(&bq76907) == BQ_OK) ? 0 : -1;
}

bool bq_controller_is_manual_override(void) {
    return g_manual_fet_override;
}

bool bq_controller_has_fault(void) {
    return (g_battery_status.safety_status_a != 0) ||
           (g_battery_status.safety_status_b != 0);
}

void bq_controller_print_fault_details(void) {
    if (!g_battery_status.is_connected) {
        hal_uart_printf("[ERROR] Device not connected\r\n");
        return;
    }

    bool has_fault = (g_battery_status.safety_status_a != 0) ||
                     (g_battery_status.safety_status_b != 0);

    if (!has_fault) {
        hal_uart_printf("[INFO] No faults detected - System OK\r\n");
        return;
    }

    hal_uart_printf("\r\n=====================================\r\n");
    hal_uart_printf("     FAULT DIAGNOSIS REPORT\r\n");
    hal_uart_printf("=====================================\r\n\r\n");

    // Parse Safety Status A (based on datasheet Table 6-3)
    if (g_battery_status.safety_status_a) {
        hal_uart_printf("[WARNING] Safety Status A: 0x%02X\r\n", g_battery_status.safety_status_a);

        if (g_battery_status.safety_status_a & BQ_SAFETY_STATUS_A_COV) {
            hal_uart_printf("  [FAULT] COV: Cell Overvoltage\r\n");
            hal_uart_printf("     Threshold: >%dmV\r\n", CFG_CELL_OV_THRESHOLD_MV);
            hal_uart_printf("     Recovery: reg=%d (0=none,1=50mV,2=100mV,3=200mV)\r\n", CFG_CELL_OV_RECOVERY_REG);
            hal_uart_printf("     Action: CHG FET disabled\r\n");
        }
        if (g_battery_status.safety_status_a & BQ_SAFETY_STATUS_A_CUV) {
            hal_uart_printf("  [FAULT] CUV: Cell Undervoltage\r\n");
            hal_uart_printf("     Threshold: <%dmV\r\n", CFG_CELL_UV_THRESHOLD_MV);
            hal_uart_printf("     Recovery: reg=%d (0=none,1=50mV,2=100mV,3=200mV)\r\n", CFG_CELL_UV_RECOVERY_REG);
            hal_uart_printf("     Action: DSG FET disabled\r\n");
        }
        if (g_battery_status.safety_status_a & BQ_SAFETY_STATUS_A_SCD) {
            hal_uart_printf("  [FAULT] SCD: Short Circuit in Discharge\r\n");
            hal_uart_printf("     Threshold: See CFG_SCD_THRESHOLD_REG\r\n");
            hal_uart_printf("     Action: DSG FET disabled immediately\r\n");
        }
        if (g_battery_status.safety_status_a & BQ_SAFETY_STATUS_A_OCD1) {
            hal_uart_printf("  [FAULT] OCD1: Overcurrent in Discharge 1 (Moderate)\r\n");
            hal_uart_printf("     Threshold: See CFG_OCD1_THRESHOLD_REG\r\n");
            hal_uart_printf("     Action: DSG FET disabled\r\n");
        }
        if (g_battery_status.safety_status_a & BQ_SAFETY_STATUS_A_OCD2) {
            hal_uart_printf("  [FAULT] OCD2: Overcurrent in Discharge 2 (High)\r\n");
            hal_uart_printf("     Threshold: See CFG_OCD2_THRESHOLD_REG\r\n");
            hal_uart_printf("     Action: DSG FET disabled\r\n");
        }
        if (g_battery_status.safety_status_a & BQ_SAFETY_STATUS_A_OCC) {
            hal_uart_printf("  [FAULT] OCC: Overcurrent in Charge\r\n");
            hal_uart_printf("     Threshold: See CFG_OCC_THRESHOLD_REG\r\n");
            hal_uart_printf("     Action: CHG FET disabled\r\n");
        }
        if (g_battery_status.safety_status_a & BQ_SAFETY_STATUS_A_CURLATCH) {
            hal_uart_printf("  [FAULT] CURLATCH: Current Protection Latch\r\n");
            hal_uart_printf("     Action: FET latched off\r\n");
        }
        if (g_battery_status.safety_status_a & BQ_SAFETY_STATUS_A_REGOUT) {
            hal_uart_printf("  [FAULT] REGOUT: REGOUT Fault\r\n");
        }
        hal_uart_printf("\r\n");
    }

    // Parse Safety Status B (based on datasheet Table 6-5)
    if (g_battery_status.safety_status_b) {
        hal_uart_printf("[WARNING] Safety Status B: 0x%02X\r\n", g_battery_status.safety_status_b);

        if (g_battery_status.safety_status_b & BQ_SAFETY_STATUS_B_OTD) {
            hal_uart_printf("  [FAULT] OTD: Overtemperature in Discharge\r\n");
            hal_uart_printf("     Threshold: >%dC\r\n", CFG_OTD_THRESHOLD_C);
            hal_uart_printf("     Recovery: <%dC\r\n", CFG_OTD_RECOVERY_C);
        }
        if (g_battery_status.safety_status_b & BQ_SAFETY_STATUS_B_OTC) {
            hal_uart_printf("  [FAULT] OTC: Overtemperature in Charge\r\n");
            hal_uart_printf("     Threshold: >%dC\r\n", CFG_OTC_THRESHOLD_C);
            hal_uart_printf("     Recovery: <%dC\r\n", CFG_OTC_RECOVERY_C);
        }
        if (g_battery_status.safety_status_b & BQ_SAFETY_STATUS_B_UTD) {
            hal_uart_printf("  [FAULT] UTD: Undertemperature in Discharge\r\n");
            hal_uart_printf("     Threshold: <%dC\r\n", CFG_UTD_THRESHOLD_C);
            hal_uart_printf("     Recovery: >%dC\r\n", CFG_UTD_RECOVERY_C);
        }
        if (g_battery_status.safety_status_b & BQ_SAFETY_STATUS_B_UTC) {
            hal_uart_printf("  [FAULT] UTC: Undertemperature in Charge\r\n");
            hal_uart_printf("     Threshold: <%dC\r\n", CFG_UTC_THRESHOLD_C);
            hal_uart_printf("     Recovery: >%dC\r\n", CFG_UTC_RECOVERY_C);
        }
        if (g_battery_status.safety_status_b & BQ_SAFETY_STATUS_B_OTINT) {
            hal_uart_printf("  [FAULT] OTINT: Internal Die Overtemperature\r\n");
            hal_uart_printf("     Threshold: >%dC\r\n", CFG_OTINT_THRESHOLD_C);
            hal_uart_printf("     Recovery: <%dC\r\n", CFG_OTINT_RECOVERY_C);
            hal_uart_printf("     Action: Both FETs disabled\r\n");
        }
        if (g_battery_status.safety_status_b & BQ_SAFETY_STATUS_B_HWD) {
            hal_uart_printf("  [FAULT] HWD: Host Watchdog\r\n");
        }
        if (g_battery_status.safety_status_b & BQ_SAFETY_STATUS_B_VREF) {
            hal_uart_printf("  [FAULT] VREF: VREF Measurement Diagnostic\r\n");
        }
        if (g_battery_status.safety_status_b & BQ_SAFETY_STATUS_B_VSS) {
            hal_uart_printf("  [FAULT] VSS: VSS Measurement Diagnostic\r\n");
        }
        hal_uart_printf("\r\n");
    }

    // Current Conditions
    hal_uart_printf("Current Conditions:\r\n");
    hal_uart_printf("  Current: %ldA (%ldmA)\r\n",
                      (long)g_battery_status.current_a,
                      (long)g_battery_status.current_ma);
    hal_uart_printf("  Temp Internal: %dC\r\n", g_battery_status.internal_temp_c);
    hal_uart_printf("  Temp TS1: %dC\r\n", g_battery_status.ts1_temp_c);

    // Cell Voltages
    hal_uart_printf("  Cell Voltages:\r\n");
    for (int i = 0; i < BATTERY_CELLS; i++) {
        char marker = ' ';
        if (g_battery_status.cell_voltage_mv[i] >= CFG_CELL_OV_THRESHOLD_MV)
            marker = 'H'; // High
        else if (g_battery_status.cell_voltage_mv[i] <= CFG_CELL_UV_THRESHOLD_MV)
            marker = 'L'; // Low

        hal_uart_printf("    Cell %d: %dmV %c\r\n",
                          i + 1,
                          g_battery_status.cell_voltage_mv[i],
                          marker);
    }
    hal_uart_printf("  Min: %dmV, Max: %dmV, Delta: %dmV\r\n",
                      g_battery_status.cell_min_mv,
                      g_battery_status.cell_max_mv,
                      g_battery_status.cell_delta_mv);
    hal_uart_printf("\r\n");

    // FET Status
    hal_uart_printf("FET Status (Battery Status: 0x%04X):\r\n", g_battery_status.battery_status);
    if (g_battery_status.battery_status & BQ_BATT_STAT_DSG_FET)
        hal_uart_printf("  [ON]  DSG FET\r\n");
    else
        hal_uart_printf("  [OFF] DSG FET\r\n");

    if (g_battery_status.battery_status & BQ_BATT_STAT_CHG_FET)
        hal_uart_printf("  [ON]  CHG FET\r\n");
    else
        hal_uart_printf("  [OFF] CHG FET\r\n");

    hal_uart_printf("\r\n");

    // Recommendations
    hal_uart_printf("Recommended Actions:\r\n");
    if (g_battery_status.safety_status_a & 0x07) { // SCD/OCD
        hal_uart_printf("  1. Check load connections\r\n");
        hal_uart_printf("  2. Verify current is within limits\r\n");
        hal_uart_printf("  3. Inspect wiring for shorts\r\n");
    }
    if (g_battery_status.safety_status_a & 0x30) { // COV/CUV
        hal_uart_printf("  1. Check charger settings\r\n");
        hal_uart_printf("  2. Verify cell balance\r\n");
        hal_uart_printf("  3. Test individual cell voltages\r\n");
    }
    if (g_battery_status.safety_status_b & 0x1F) { // Temperature
        hal_uart_printf("  1. Check ambient temperature\r\n");
        hal_uart_printf("  2. Improve ventilation/cooling\r\n");
        hal_uart_printf("  3. Reduce load current\r\n");
    }
    hal_uart_printf("  4. Use bq_controller_clear_faults() to reset\r\n");

    hal_uart_printf("\r\n=====================================\r\n\r\n");
}

int bq_controller_clear_faults(void) {
    hal_uart_printf("[INFO] Attempting to clear faults...\r\n");

    /* Clear TS1 open wire state so it can re-evaluate */
    g_battery_status.ts1_open_wire = false;
    g_ts1_ow_debounce = 0;
    g_ts1_ow_recovery = 0;
    g_ts1_prev_valid = false;

    // Refresh status from chip into g_battery_status
    update_status_flags();

    hal_uart_printf("  Before clear: SafetyA=0x%02X SafetyB=0x%02X BattStat=0x%04X\r\n",
                      g_battery_status.safety_status_a,
                      g_battery_status.safety_status_b,
                      g_battery_status.battery_status);
    hal_uart_printf("  DSG FET: %s, CHG FET: %s\r\n",
                      (g_battery_status.battery_status & BQ_BATT_STAT_DSG_FET) ? "ON" : "OFF",
                      (g_battery_status.battery_status & BQ_BATT_STAT_CHG_FET) ? "ON" : "OFF");

    // PROT_RECOVERY is read-only in SEALED mode — unseal first
    ensure_unsealed();

    uint8_t recovery_mask = 0xFF;
    bq_status_t st = bq7690x_write_prot_recovery(&bq76907, recovery_mask);

    if (st == BQ_OK) {
        hal_uart_printf("[OK] PROT_RECOVERY command sent (mask=0xFF)\r\n");
        delay_ms(500);

        update_status_flags();

        hal_uart_printf("  After clear:  SafetyA=0x%02X SafetyB=0x%02X BattStat=0x%04X\r\n",
                          g_battery_status.safety_status_a,
                          g_battery_status.safety_status_b,
                          g_battery_status.battery_status);
        hal_uart_printf("  DSG FET: %s, CHG FET: %s\r\n",
                          (g_battery_status.battery_status & BQ_BATT_STAT_DSG_FET) ? "ON" : "OFF",
                          (g_battery_status.battery_status & BQ_BATT_STAT_CHG_FET) ? "ON" : "OFF");

        if (bq_controller_has_fault()) {
            hal_uart_printf("[WARN] Faults still active after recovery attempt\r\n");
            return -1;
        } else {
            hal_uart_printf("[OK] Faults cleared successfully\r\n");

            if (!g_manual_fet_override) {
                ensure_fet_en();
                delay_ms(100);
            }

            update_status_flags();
            hal_uart_printf("  Final FET status: DSG=%s, CHG=%s\r\n",
                              (g_battery_status.battery_status & BQ_BATT_STAT_DSG_FET) ? "ON" : "OFF",
                              (g_battery_status.battery_status & BQ_BATT_STAT_CHG_FET) ? "ON" : "OFF");

            return 0;
        }
    } else {
        hal_uart_printf("[FAIL] Failed to write PROT_RECOVERY register\r\n");
        return -1;
    }
}

void bq_controller_print_status(void) {
    if (!g_battery_status.is_connected) {
        hal_uart_printf("[ERROR] Device not connected\r\n");
        return;
    }

    hal_uart_printf("\r\n=== Battery Status ===\r\n");

    // Cell voltages
    hal_uart_printf("Cells: ");
    for (int i = 0; i < BATTERY_CELLS; i++) {
        hal_uart_printf("%d=%dmV ", i+1, g_battery_status.cell_voltage_mv[i]);
    }
    hal_uart_printf("\r\n");

    // Voltage summary
    hal_uart_printf("Delta: %dmV, Stack: %d.%03dV\r\n",
                      g_battery_status.cell_delta_mv,
                      g_battery_status.stack_voltage_mv / 1000,
                      g_battery_status.stack_voltage_mv % 1000);

    // Current (display in A for high current applications)
    hal_uart_printf("Current: %ldA (%ldmA)\r\n",
                      (long)g_battery_status.current_a,
                      (long)g_battery_status.current_ma);

    // Power
    int32_t power_w = g_battery_status.power_mw / 1000;
    int32_t power_w_frac = abs((int)(g_battery_status.power_mw % 1000));
    hal_uart_printf("Power: %ld.%03dW (%ldmW)\r\n",
                      (long)power_w,
                      (int)power_w_frac,
                      (long)g_battery_status.power_mw);

    // Temperature
    hal_uart_printf("Temp: Int=%dC, TS1=%dC\r\n",
                      g_battery_status.internal_temp_c,
                      g_battery_status.ts1_temp_c);

    // Safety status
    hal_uart_printf("Safety: A=0x%02X/0x%02X, B=0x%02X/0x%02X\r\n",
                      g_battery_status.safety_alert_a,
                      g_battery_status.safety_status_a,
                      g_battery_status.safety_alert_b,
                      g_battery_status.safety_status_b);

    // Balancing
    hal_uart_printf("Balance: 0x%02X", g_battery_status.balance_active_cells);
    if (g_battery_status.balance_active_cells) {
        hal_uart_printf(" (Active: ");
        for (int i = 0; i < BATTERY_CELLS; i++) {
            if (g_battery_status.balance_active_cells & (1 << i)) {
                hal_uart_printf("%d ", i + 1);
            }
        }
        hal_uart_printf(")");
    }
    hal_uart_printf("\r\n");

    // FET Status
    hal_uart_printf("FET Status (Battery Status: 0x%04X):\r\n", g_battery_status.battery_status);
    if (g_battery_status.battery_status & BQ_BATT_STAT_DSG_FET)
        hal_uart_printf("  [ON]  DSG FET\r\n");
    else
        hal_uart_printf("  [OFF] DSG FET\r\n");

    if (g_battery_status.battery_status & BQ_BATT_STAT_CHG_FET)
        hal_uart_printf("  [ON]  CHG FET\r\n");
    else
        hal_uart_printf("  [OFF] CHG FET\r\n");

    hal_uart_printf("===================\r\n\r\n");
}

/* ========== ALERT Pin Functions ========== */

bool bq_controller_is_alert_active(void) {
    // ALERT pin is active LOW; hal_gpio_read_alert() returns 1 when the pin is
    // LOW (active) and 0 when HIGH (idle), so pass it through.
    return (hal_gpio_read_alert() != 0);
}

int bq_controller_handle_alert(void) {
    // Refresh all status registers into g_battery_status
    update_status_flags();

    if (g_battery_status.alarm_status == 0) {
        return 0;  // Spurious alert
    }

    uint16_t alarm = g_battery_status.alarm_status;

    // Log all alarm bits
    hal_uart_printf("[ALERT] Alarm: 0x%04X", alarm);
    if (alarm & BQ_ALARM_SSA)
        hal_uart_printf(" SSA(0x%02X)", g_battery_status.safety_status_a);
    if (alarm & BQ_ALARM_SSB)
        hal_uart_printf(" SSB(0x%02X)", g_battery_status.safety_status_b);
    if (alarm & BQ_ALARM_SAA)   hal_uart_printf(" SAA");
    if (alarm & BQ_ALARM_SAB)   hal_uart_printf(" SAB");
    if (alarm & BQ_ALARM_XDSG)  hal_uart_printf(" XDSG");
    if (alarm & BQ_ALARM_XCHG)  hal_uart_printf(" XCHG");
    if (alarm & BQ_ALARM_SHUTV) hal_uart_printf(" SHUTV");
    if (alarm & BQ_ALARM_WAKE)  hal_uart_printf(" WAKE");
    if (alarm & BQ_ALARM_SLEEP) hal_uart_printf(" SLEEP");
    if (alarm & BQ_ALARM_FULLSCAN) hal_uart_printf(" FULLSCAN");
    if (alarm & BQ_ALARM_CDTOGGLE) hal_uart_printf(" CDTOGGLE");
    if (alarm & BQ_ALARM_POR)   hal_uart_printf(" POR");
    hal_uart_printf("\r\n");

    // Log BatteryStatus for sleep/wake context
    if (alarm & (BQ_ALARM_WAKE | BQ_ALARM_SLEEP)) {
        hal_uart_printf("[ALERT] BatStat=0x%04X SLEEP=%d SLEEP_EN=%d ALERT=%d\r\n",
                          g_battery_status.battery_status,
                          (g_battery_status.battery_status & BQ_BATT_STAT_SLEEP) ? 1 : 0,
                          (g_battery_status.battery_status & BQ_BATT_STAT_SLEEP_EN) ? 1 : 0,
                          (g_battery_status.battery_status & BQ_BATT_STAT_ALERTPIN) ? 1 : 0);
    }

    // Clear the alarm status to deassert ALERT pin
    if (bq7690x_write_alarm_status(&bq76907, alarm) != BQ_OK) {
        return -1;
    }

    return 0;
}

/* ========== Sleep Wake Test ========== */

void bq_controller_sleep_test_start(void) {
    uint16_t bs = 0;
    bq7690x_read_battery_status(&bq76907, &bs);

    hal_uart_printf("[SLP_TEST] === BQ76907 Sleep/Wake Test ===\r\n");
    hal_uart_printf("[SLP_TEST] BatStat=0x%04X SLEEP=%d SLEEP_EN=%d\r\n",
                      bs, (bs >> 15) & 1, (bs >> 6) & 1);
    hal_uart_printf("[SLP_TEST] SleepCurr=%dmA VoltTime=%ds WakeComp=%d\r\n",
                      CFG_SLEEP_CURRENT_MA, CFG_SLEEP_VOLTAGE_TIME_S,
                      CFG_WAKE_COMPARATOR_CURR);
    hal_uart_printf("[SLP_TEST] ALERT pin: %s\r\n",
                      hal_gpio_read_alert() ? "ACTIVE (LOW)" : "idle (HIGH)");
    hal_uart_printf("[SLP_TEST] Suppressing I2C to let BQ enter SLEEP...\r\n");
    hal_uart_printf("[SLP_TEST] Expected: SLEEP alarm in ~%ds, then apply load for WAKE\r\n",
                      CFG_SLEEP_VOLTAGE_TIME_S);
    hal_uart_printf("[SLP_TEST] Type UUU again to stop test\r\n");

    g_sleep_test_active = 1;
}

void bq_controller_sleep_test_stop(void) {
    g_sleep_test_active = 0;
    hal_uart_printf("[SLP_TEST] Test stopped. Resuming normal I2C.\r\n");
}

int bq_controller_sleep_test_handle_alert(void) {
    uint16_t alarm = 0, bs = 0;
    bq7690x_read_alarm_status(&bq76907, &alarm);
    bq7690x_read_battery_status(&bq76907, &bs);

    hal_uart_printf("[SLP_TEST] ALERT! pin=LOW\r\n");
    hal_uart_printf("[SLP_TEST] AlarmStatus=0x%04X", alarm);
    if (alarm & BQ_ALARM_SLEEP) hal_uart_printf(" SLEEP");
    if (alarm & BQ_ALARM_WAKE)  hal_uart_printf(" WAKE");
    if (alarm & BQ_ALARM_SSA)   hal_uart_printf(" SSA");
    if (alarm & BQ_ALARM_SSB)   hal_uart_printf(" SSB");
    if (alarm & BQ_ALARM_XDSG)  hal_uart_printf(" XDSG");
    if (alarm & BQ_ALARM_XCHG)  hal_uart_printf(" XCHG");
    if (alarm & BQ_ALARM_CDTOGGLE) hal_uart_printf(" CDTOGGLE");
    if (alarm & BQ_ALARM_FULLSCAN) hal_uart_printf(" FULLSCAN");
    if (alarm & BQ_ALARM_POR)   hal_uart_printf(" POR");
    hal_uart_printf("\r\n");
    hal_uart_printf("[SLP_TEST] BatStat=0x%04X SLEEP=%d SLEEP_EN=%d DSG=%d CHG=%d\r\n",
                      bs,
                      (bs & BQ_BATT_STAT_SLEEP) ? 1 : 0,
                      (bs & BQ_BATT_STAT_SLEEP_EN) ? 1 : 0,
                      (bs & BQ_BATT_STAT_DSG_FET) ? 1 : 0,
                      (bs & BQ_BATT_STAT_CHG_FET) ? 1 : 0);

    if (alarm & BQ_ALARM_SLEEP) {
        hal_uart_printf("[SLP_TEST] >> BQ entered SLEEP mode\r\n");
        hal_uart_printf("[SLP_TEST] >> Apply load to test current wake (>0.2A)\r\n");
    }

    if (alarm & BQ_ALARM_WAKE) {
        hal_uart_printf("[SLP_TEST] >> *** WAKE FROM CURRENT DETECTED! ***\r\n");
        hal_uart_printf("[SLP_TEST] >> Test PASSED. Stopping.\r\n");
        g_sleep_test_active = 0;
    }

    // Clear alarm to deassert ALERT pin
    if (alarm != 0) {
        bq7690x_write_alarm_status(&bq76907, alarm);
    }

    return (alarm & BQ_ALARM_WAKE) ? 1 : 0;
}

bool bq_controller_is_sleep_test_active(void) {
    return g_sleep_test_active != 0;
}

/* ========== Coulomb Counter Function ========== */

int32_t bq_controller_get_passed_charge_uah(void)
{
    return g_battery_status.passed_charge_uah;
}

int bq_controller_reset_passq(void)
{
    if (bq7690x_reset_pass_q(&bq76907) != BQ_OK) {
        hal_uart_printf("[PASSQ] Reset FAILED\r\n");
        return -1;
    }
    g_battery_status.passed_charge_uah = 0;
    g_battery_status.dod0_uah = 0;
    g_battery_status.passq_elapsed_time_sec = 0;
    hal_uart_printf("[PASSQ] Counter reset OK\r\n");
    return 0;
}

/* ========== Calibration Debug Functions ========== */

void bq_controller_print_calibration(void)
{
    uint16_t cc1_gain = 0, cc2_gain = 0;
    int16_t cc1_offset = 0, cc2_offset = 0;

    hal_uart_printf("\r\n========================================\r\n");
    hal_uart_printf("    BQ7690x CALIBRATION VALUES\r\n");
    hal_uart_printf("========================================\r\n\r\n");

    // Read CC1 calibration (used for PASSQ integration) - static config, direct read OK
    if (bq7690x_read_cc1_gain(&bq76907, &cc1_gain) == BQ_OK) {
        hal_uart_printf("CC1 Gain:       %u (0x%04X)\r\n", cc1_gain, cc1_gain);
    } else {
        hal_uart_printf("CC1 Gain:       [READ FAILED]\r\n");
    }

    if (bq7690x_read_cc1_offset(&bq76907, &cc1_offset) == BQ_OK) {
        hal_uart_printf("CC1 Offset:     %d (0x%04X)\r\n", cc1_offset, (uint16_t)cc1_offset);
    } else {
        hal_uart_printf("CC1 Offset:     [READ FAILED]\r\n");
    }

    // Read CC2 calibration (used for current measurement 0x3A) - static config, direct read OK
    if (bq7690x_read_curr_gain(&bq76907, &cc2_gain) == BQ_OK) {
        hal_uart_printf("CC2 Gain:       %u (0x%04X)\r\n", cc2_gain, cc2_gain);
    } else {
        hal_uart_printf("CC2 Gain:       [READ FAILED]\r\n");
    }

    if (bq7690x_read_curr_offset(&bq76907, &cc2_offset) == BQ_OK) {
        hal_uart_printf("CC2 Offset:     %d (0x%04X)\r\n", cc2_offset, (uint16_t)cc2_offset);
    } else {
        hal_uart_printf("CC2 Offset:     [READ FAILED]\r\n");
    }

    hal_uart_printf("\r\n");
    hal_uart_printf("========================================\r\n");
    hal_uart_printf("    CURRENT MEASUREMENTS\r\n");
    hal_uart_printf("========================================\r\n\r\n");

    // Use cached values from g_battery_status (updated every 100ms by bq_controller_run)
    hal_uart_printf("Current:        %ld mA (CC2 gain-scaled)\r\n",
                      (long)g_battery_status.current_ma);
    hal_uart_printf("CC1 LSB:        %lu uA   CC2 LSB: %lu uA\r\n",
                      (unsigned long)g_cc1_ua_per_lsb, (unsigned long)g_cc2_ua_per_lsb);

    hal_uart_printf("\r\n");
    hal_uart_printf("========================================\r\n");
    hal_uart_printf("    NOTES\r\n");
    hal_uart_printf("========================================\r\n");
    hal_uart_printf("Rsense Config:  %u uOhm (0.%u mOhm)\r\n",
                      CFG_RSENSE_MICRO_OHM, CFG_RSENSE_MICRO_OHM/100);
    hal_uart_printf("CC1: Used for PASSQ integration\r\n");
    hal_uart_printf("CC2: Used for 0x3A Current() register\r\n");
    hal_uart_printf("Raw: 32-bit raw ADC, no gain applied\r\n");
    hal_uart_printf("\r\n");
    hal_uart_printf("Default calibration for 1 mOhm:\r\n");
    hal_uart_printf("  - 1 LSB = 1 mA\r\n");
    hal_uart_printf("  - Range: ±32.768 A\r\n");
    hal_uart_printf("\r\n");
    hal_uart_printf("For 0.1 mOhm (100 uOhm) with 10mA/LSB:\r\n");
    hal_uart_printf("  - 1 LSB = 10 mA\r\n");
    hal_uart_printf("  - Range: ±327.68 A\r\n");
    hal_uart_printf("  - Gain should be ~1/10 of default\r\n");
    hal_uart_printf("========================================\r\n\r\n");
}

/* ========== FET Diagnostic Function ========== */

void bq_controller_print_fet_diag(void) {
    uint16_t batt_stat = 0;
    uint8_t  fet_ctrl = 0, fet_opts = 0;
    int16_t  body_diode = 0;

    bq7690x_read_battery_status(&bq76907, &batt_stat);
    bq7690x_read_fet_control(&bq76907, &fet_ctrl);
    bq7690x_read_fet_options(&bq76907, &fet_opts);
    bq7690x_read_body_diode_threshold(&bq76907, &body_diode);

    hal_uart_printf("[FET DIAG] BattStat=0x%04X FETCtrl=0x%02X FETOpts=0x%02X BodyDiode=%dmV\r\n",
                      batt_stat, fet_ctrl, fet_opts, body_diode);
    hal_uart_printf("  FET_EN=%d DSG=%s CHG=%s SFET=%d HOST_ON=%d HOST_OFF=%d\r\n",
                      (batt_stat & BQ_BATT_STAT_FET_EN) ? 1 : 0,
                      (batt_stat & BQ_BATT_STAT_DSG_FET) ? "ON" : "OFF",
                      (batt_stat & BQ_BATT_STAT_CHG_FET) ? "ON" : "OFF",
                      (fet_opts & BQ_FETOPT_SFET) ? 1 : 0,
                      (fet_opts & BQ_FETOPT_HOST_FETON) ? 1 : 0,
                      (fet_opts & BQ_FETOPT_HOST_FETOFF) ? 1 : 0);
}

/* ====================================================================
 * Runtime parameter-to-register conversions (for param_apply_to_bq)
 * ==================================================================== */

#define BQ_VOLT_PROT_DELAY_MAX_ADSCAN 255u
/* ms → ADSCAN count using CFG_ADSCAN_US (cell count × CVADCSPEED slot width). */
static inline uint8_t bq_ms_to_adscan(uint16_t ms)
{
    uint32_t n = BQ_MS_TO_ADSCAN(ms);
    if (n < 1u)  n = 1u;
    if (n > BQ_VOLT_PROT_DELAY_MAX_ADSCAN) n = BQ_VOLT_PROT_DELAY_MAX_ADSCAN;
    return (uint8_t)n;
}

static uint8_t runtime_occ_reg(uint16_t current_a, uint32_t rsense_uohm)
{
    uint32_t reg = ((uint32_t)current_a * rsense_uohm + 1000) / 2000;
    if (reg < 2)  reg = 2;   /* TRM min valid value */
    if (reg > 62) reg = 62;  /* TRM max for OCC threshold */
    return (uint8_t)reg;
}

static uint8_t runtime_ocd_reg(uint16_t current_a, uint32_t rsense_uohm)
{
    uint32_t reg = ((uint32_t)current_a * rsense_uohm) / 2000;
    if (reg < 2)   reg = 2;   /* TRM min valid value */
    if (reg > 100) reg = 100; /* TRM max for OCD threshold */
    return (uint8_t)reg;
}

static uint8_t runtime_scd_reg(uint16_t current_a, uint32_t rsense_uohm)
{
    uint32_t mv = ((uint32_t)current_a * rsense_uohm) / 1000;
    if (mv <= 10)  return 0;
    if (mv <= 20)  return 1;
    if (mv <= 40)  return 2;
    if (mv <= 60)  return 3;
    if (mv <= 80)  return 4;
    if (mv <= 100) return 5;
    if (mv <= 125) return 6;
    if (mv <= 150) return 7;
    if (mv <= 175) return 8;
    if (mv <= 200) return 9;
    if (mv <= 250) return 10;
    if (mv <= 300) return 11;
    if (mv <= 350) return 12;
    if (mv <= 400) return 13;
    if (mv <= 450) return 14;
    return 15;
}

/* Runtime temperature-to-resistance conversion: piecewise linear NTC 10k
 * approximation. */
static int32_t runtime_temp_to_resistance(int16_t temp_c)
{
    if (temp_c <= 0)
        return 27500L - ((int32_t)temp_c * 700L);
    else if (temp_c <= 25)
        return 10000L + ((int32_t)(25 - temp_c) * 450L);
    else if (temp_c <= 60)
        return 10000L - ((int32_t)(temp_c - 25) * 200L);
    else
        return 3000L - ((int32_t)(temp_c - 60) * 50L);
}

static int32_t runtime_resistance_to_adc(int32_t r_ohm)
{
    return (r_ohm * 19565L) / (20000L + r_ohm);
}

static uint8_t runtime_ot_reg(int16_t temp_c)
{
    return (uint8_t)(runtime_resistance_to_adc(runtime_temp_to_resistance(temp_c)) / 54);
}

static uint8_t runtime_ut_reg(int16_t temp_c)
{
    return (uint8_t)(runtime_resistance_to_adc(runtime_temp_to_resistance(temp_c)) / 77);
}

/* ====================================================================
 * param_apply_to_bq() — write all cached params to BQ76907
 * ==================================================================== */

int param_apply_to_bq(void)
{
    hal_uart_printf("[PARAM] Applying params to BQ IC...\r\n");

    /* Read sense resistor for current conversions */
    uint32_t rsense = param_cache[PARAM_RSENSE_MICRO_OHM].u32;

    /* Refresh PASSQ, then save the accumulator before CONFIG_UPDATE destroys it. */
    update_battery_measurements();
    battery_soc_soh_pre_config_update();

    /* Enter CONFIG_UPDATE once for all writes */
    bq_status_t st = bq7690x_set_config_update(&bq76907);
    if (st != BQ_OK) {
        hal_uart_printf("[PARAM] ERROR: CONFIG_UPDATE enter failed (%d)\r\n", st);
        return -1;
    }

    int errors = 0;

    /* --- Cell Voltage Protection --- */
    if (bq_dm_write_16(&bq76907, CellOvervoltageProtectionThreshold,
                        param_cache[PARAM_CELL_OV_THRESHOLD_MV].u16) != BQ_OK) errors++;
    if (bq_dm_write_8(&bq76907, CellOvervoltageProtectionDelay,
                       bq_ms_to_adscan(param_cache[PARAM_CELL_OV_DELAY_MS].u16)) != BQ_OK) errors++;
    /* Convert mV -> register encoding: 0=none, 1=50mV, 2=100mV, 3=200mV */
    if (bq_dm_write_8(&bq76907, CellOvervoltageProtectionRecoveryHysteresis,
                       hyst_mv_to_reg(param_cache[PARAM_CELL_OV_RECOVERY_MV].u16)) != BQ_OK) errors++;

    if (bq_dm_write_16(&bq76907, CellUndervoltageProtectionThreshold,
                        param_cache[PARAM_CELL_UV_THRESHOLD_MV].u16) != BQ_OK) errors++;
    if (bq_dm_write_8(&bq76907, CellUndervoltageProtectionDelay,
                       bq_ms_to_adscan(param_cache[PARAM_CELL_UV_DELAY_MS].u16)) != BQ_OK) errors++;
    if (bq_dm_write_8(&bq76907, CellUndervoltageProtectionRecoveryHysteresis,
                       hyst_mv_to_reg(param_cache[PARAM_CELL_UV_RECOVERY_MV].u16)) != BQ_OK) errors++;

    /* --- Current Protection --- */
    if (bq_dm_write_8(&bq76907, OvercurrentinChargeProtectionThreshold,
                       runtime_occ_reg(param_cache[PARAM_OCC_THRESHOLD_A].u16, rsense)) != BQ_OK) errors++;
    if (bq_dm_write_8(&bq76907, OvercurrentinChargeProtectionDelay,
                       param_cache[PARAM_OCC_DELAY_REG].u8) != BQ_OK) errors++;
    if (bq_dm_write_8(&bq76907, OvercurrentinDischarge1ProtectionThreshold,
                       runtime_ocd_reg(param_cache[PARAM_OCD1_THRESHOLD_A].u16, rsense)) != BQ_OK) errors++;
    if (bq_dm_write_8(&bq76907, OvercurrentinDischarge1ProtectionDelay,
                       param_cache[PARAM_OCD1_DELAY_REG].u8) != BQ_OK) errors++;
    if (bq_dm_write_8(&bq76907, OvercurrentinDischarge2ProtectionThreshold,
                       runtime_ocd_reg(param_cache[PARAM_OCD2_THRESHOLD_A].u16, rsense)) != BQ_OK) errors++;
    if (bq_dm_write_8(&bq76907, OvercurrentinDischarge2ProtectionDelay,
                       param_cache[PARAM_OCD2_DELAY_REG].u8) != BQ_OK) errors++;
    if (bq_dm_write_8(&bq76907, ShortCircuitinDischargeProtectionThreshold,
                       runtime_scd_reg(param_cache[PARAM_SCD_THRESHOLD_A].u16, rsense)) != BQ_OK) errors++;
    if (bq_dm_write_8(&bq76907, ShortCircuitinDischargeProtectionDelay,
                       param_cache[PARAM_SCD_DELAY_REG].u8) != BQ_OK) errors++;
    if (bq_dm_write_8(&bq76907, LatchLimit,
                       param_cache[PARAM_LATCH_LIMIT].u8) != BQ_OK) errors++;
    if (bq_dm_write_8(&bq76907, RecoveryTime,
                       param_cache[PARAM_CURRENT_RECOVERY_TIME_S].u8) != BQ_OK) errors++;

    /* --- Temperature Protection --- */
    if (bq_dm_write_8(&bq76907, OvertemperatureinChargeProtectionThreshold,
                       runtime_ot_reg(param_cache[PARAM_OTC_THRESHOLD_C].i16)) != BQ_OK) errors++;
    if (bq_dm_write_8(&bq76907, OvertemperatureinChargeProtectionDelay,
                       param_cache[PARAM_OTC_DELAY_S].u8) != BQ_OK) errors++;
    if (bq_dm_write_8(&bq76907, OvertemperatureinChargeProtectionRecovery,
                       runtime_ot_reg(param_cache[PARAM_OTC_RECOVERY_C].i16)) != BQ_OK) errors++;
    if (bq_dm_write_8(&bq76907, UndertemperatureinChargeProtectionThreshold,
                       runtime_ut_reg(param_cache[PARAM_UTC_THRESHOLD_C].i16)) != BQ_OK) errors++;
    if (bq_dm_write_8(&bq76907, UndertemperatureinChargeProtectionDelay,
                       param_cache[PARAM_UTC_DELAY_S].u8) != BQ_OK) errors++;
    if (bq_dm_write_8(&bq76907, UndertemperatureinChargeProtectionRecovery,
                       runtime_ut_reg(param_cache[PARAM_UTC_RECOVERY_C].i16)) != BQ_OK) errors++;
    if (bq_dm_write_8(&bq76907, OvertemperatureinDischargeProtectionThreshold,
                       runtime_ot_reg(param_cache[PARAM_OTD_THRESHOLD_C].i16)) != BQ_OK) errors++;
    if (bq_dm_write_8(&bq76907, OvertemperatureinDischargeProtectionDelay,
                       param_cache[PARAM_OTD_DELAY_S].u8) != BQ_OK) errors++;
    if (bq_dm_write_8(&bq76907, OvertemperatureinDischargeProtectionRecovery,
                       runtime_ot_reg(param_cache[PARAM_OTD_RECOVERY_C].i16)) != BQ_OK) errors++;
    if (bq_dm_write_8(&bq76907, UndertemperatureinDischargeProtectionThreshold,
                       runtime_ut_reg(param_cache[PARAM_UTD_THRESHOLD_C].i16)) != BQ_OK) errors++;
    if (bq_dm_write_8(&bq76907, UndertemperatureinDischargeProtectionDelay,
                       param_cache[PARAM_UTD_DELAY_S].u8) != BQ_OK) errors++;
    if (bq_dm_write_8(&bq76907, UndertemperatureinDischargeProtectionRecovery,
                       runtime_ut_reg(param_cache[PARAM_UTD_RECOVERY_C].i16)) != BQ_OK) errors++;
    if (bq_dm_write_8(&bq76907, InternalOvertemperatureProtectionThreshold,
                       param_cache[PARAM_OTINT_THRESHOLD_C].u8) != BQ_OK) errors++;
    if (bq_dm_write_8(&bq76907, InternalOvertemperatureProtectionDelay,
                       param_cache[PARAM_OTINT_DELAY_S].u8) != BQ_OK) errors++;
    if (bq_dm_write_8(&bq76907, InternalOvertemperatureProtectionRecovery,
                       param_cache[PARAM_OTINT_RECOVERY_C].u8) != BQ_OK) errors++;

    /* --- Sleep/Wake/Shutdown --- */
    /* SleepCurrent written after CC1 gain scaling (needs g_cc1_ua_per_lsb for mA->userA) */
    if (bq_dm_write_8(&bq76907, VoltageTime,
                       param_cache[PARAM_SLEEP_VOLTAGE_TIME_S].u8) != BQ_OK) errors++;
    if (bq_dm_write_8(&bq76907, WakeComparatorCurrent,
                       param_cache[PARAM_WAKE_COMPARATOR_CURR].u8) != BQ_OK) errors++;
    if (bq_dm_write_16(&bq76907, ShutdownCellVoltage,
                        (uint16_t)param_cache[PARAM_SHUTDOWN_CELL_MV].i16) != BQ_OK) errors++;
    if (bq_dm_write_16(&bq76907, ShutdownStackVoltage,
                        param_cache[PARAM_SHUTDOWN_STACK_MV].u16) != BQ_OK) errors++;
    if (bq_dm_write_8(&bq76907, ShutdownTemperature,
                       param_cache[PARAM_SHUTDOWN_TEMP_C].u8) != BQ_OK) errors++;
    if (bq_dm_write_8(&bq76907, AutoShutdownTime,
                       param_cache[PARAM_AUTO_SHUTDOWN_TIME_MIN].u8) != BQ_OK) errors++;

    /* --- FET/Config --- */
    /* fet_options_now(), not the raw cache value: FET_EN is firmware-owned and
     * must stay clear while g_inhibit_fet_en is raised. */
    if (bq_dm_write_8(&bq76907, FETOptions,
                       fet_options_now()) != BQ_OK) errors++;
    if (bq_dm_write_8(&bq76907, ChargeDetectorTime,
                       param_cache[PARAM_CHARGE_DETECTOR_TIME_100MS].u8) != BQ_OK) errors++;
    /* Body diode threshold is in userA (current), NOT mV! min=0, max=32767 */
    if (bq_dm_write_16(&bq76907, BodyDiodeThreshold,
                        param_cache[PARAM_BODY_DIODE_THRESHOLD].u16) != BQ_OK) errors++;

    /* --- Watchdog --- */
    if (bq_dm_write_8(&bq76907, HostWatchdogTimeout,
                       param_cache[PARAM_HOST_WATCHDOG_TIMEOUT_S].u8) != BQ_OK) errors++;

    /* --- Protection Enable Masks --- */
    if (bq_dm_write_8(&bq76907, EnabledProtectionsA,
                       param_cache[PARAM_ENABLED_PROTECTIONS_A].u8) != BQ_OK) errors++;
    if (bq_dm_write_8(&bq76907, EnabledProtectionsB,
                       param_cache[PARAM_ENABLED_PROTECTIONS_B].u8) != BQ_OK) errors++;
    if (bq_dm_write_8(&bq76907, DSGFETProtectionsA,
                       param_cache[PARAM_DSG_FET_PROTECTIONS_A].u8) != BQ_OK) errors++;
    if (bq_dm_write_8(&bq76907, CHGFETProtectionsA,
                       param_cache[PARAM_CHG_FET_PROTECTIONS_A].u8) != BQ_OK) errors++;
    if (bq_dm_write_8(&bq76907, BothFETProtectionsB,
                       param_cache[PARAM_BOTH_FET_PROTECTIONS_B].u8) != BQ_OK) errors++;

    /* The chip's own cell open-wire stimulus: a ~55 uA sink per used VC pin,
     * enabled once per this many measurement loops. 0 = off (factory default).
     * Without it, detection timing depends on board leakage. */
    if (bq_dm_write_8(&bq76907, CellOpenWireNORMALCheckTime,
                       param_cache[PARAM_OW_CHECK_TIME].u8) != BQ_OK) errors++;

    /* --- ADC/Coulomb Config --- */
    if (bq_dm_write_16(&bq76907, DAConfig,
                        param_cache[PARAM_DA_CONFIG].u16) != BQ_OK) errors++;
    if (bq_dm_write_8(&bq76907, PowerConfig,
                       param_cache[PARAM_POWER_CONFIG].u8) != BQ_OK) errors++;
    if (bq_dm_write_16(&bq76907, DefaultAlarmMask,
                        param_cache[PARAM_DEFAULT_ALARM_MASK].u16) != BQ_OK) errors++;

    /* --- CC1/CC2 Gain Scaling --- */
    {
        uint32_t max_i_a = param_cache[PARAM_MAX_CURRENT_A].u16;
        uint32_t factory_range_ma = 32767UL * 1000 / rsense;
        uint32_t desired_range_ma = max_i_a * 1000;
        uint16_t cc1_new = g_cc1_gain_factory ? g_cc1_gain_factory : 1;
        uint16_t cc2_new = g_curr_gain_factory ? g_curr_gain_factory : 1;
        if (factory_range_ma < desired_range_ma && g_cc1_gain_factory > 0) {
            cc1_new = (uint16_t)((uint32_t)g_cc1_gain_factory * factory_range_ma / desired_range_ma);
            cc2_new = (uint16_t)((uint32_t)g_curr_gain_factory * factory_range_ma / desired_range_ma);
            if (cc1_new == 0) cc1_new = 1;
            if (cc2_new == 0) cc2_new = 1;
        }
        if (bq_dm_write_16(&bq76907, CC1Gain, cc1_new) != BQ_OK) errors++;
        if (bq_dm_write_16(&bq76907, CurrGain, cc2_new) != BQ_OK) errors++;
        g_cc1_ua_per_lsb = (uint32_t)((uint64_t)1000000 * g_cc1_gain_factory / ((uint64_t)rsense * cc1_new));
        g_cc2_ua_per_lsb = (uint32_t)((uint64_t)1000000 * g_curr_gain_factory / ((uint64_t)rsense * cc2_new));
        hal_uart_printf("[PARAM] CC1Gain=%u CurrGain=%u cc1_ua/LSB=%lu cc2_ua/LSB=%lu\r\n",
                          cc1_new, cc2_new, (unsigned long)g_cc1_ua_per_lsb, (unsigned long)g_cc2_ua_per_lsb);
    }

    /* --- SleepCurrent: register is in userA, convert from mA using scaled gain --- */
    {
        uint32_t sleep_raw = (uint32_t)param_cache[PARAM_SLEEP_CURRENT_MA].u16 * 1000 / g_cc1_ua_per_lsb;
        if (sleep_raw == 0) sleep_raw = 1;
        if (sleep_raw > 32767) sleep_raw = 32767;
        uint16_t sleep_userA = (uint16_t)sleep_raw;
        if (bq_dm_write_16(&bq76907, SleepCurrent, sleep_userA) != BQ_OK) errors++;
        hal_uart_printf("[PARAM] SleepCurrent=%u userA (%u mA)\r\n",
                          sleep_userA, param_cache[PARAM_SLEEP_CURRENT_MA].u16);
    }

    /* Exit CONFIG_UPDATE — IC applies all written values.
     * Retry up to 3 times: leaving IC in CONFIG_UPDATE disables protection! */
    for (int retry = 0; retry < 3; retry++) {
        st = bq7690x_exit_config_update(&bq76907);
        if (st == BQ_OK) break;
        hal_uart_printf("[PARAM] WARNING: CONFIG_UPDATE exit retry %d (%d)\r\n", retry + 1, st);
        delay_ms(10);
    }
    if (st != BQ_OK) {
        hal_uart_printf("[PARAM] CRITICAL: CONFIG_UPDATE exit failed — IC stuck in config mode!\r\n");
        return -2;
    }

    /* Reset PASSQ (destroyed by CONFIG_UPDATE) and resync SOC tracking */
    bq7690x_reset_pass_q(&bq76907);
    battery_soc_soh_post_config_update();

    /* PowerConfig writes SLEEP_EN=1 to DM; BQ SLEEP stays off at runtime */
    bq7690x_sleep_disable(&bq76907);

    /* Diagnostic: verify DAConfig runtime value (no extra CONFIG_UPDATE) */
    {
        uint16_t da_runtime = 0;
        bq_dm_read_16(&bq76907, DAConfig, &da_runtime);
        hal_uart_printf("[PARAM] DAConfig runtime=0x%04X\r\n", (unsigned)da_runtime);
    }

    /* AlarmEnable (0x66) is a direct register, NOT data memory — write after CONFIG_UPDATE */
    if (bq7690x_write_alarm_enable(&bq76907, param_cache[PARAM_ALARM_ENABLE_MASK].u16) != BQ_OK)
        errors++;

    if (errors > 0) {
        hal_uart_printf("[PARAM] WARNING: %d register writes failed\r\n", errors);
        return -1;
    }

    hal_uart_printf("[PARAM] Applied %d params: OCD1=%uA SCD=%uA "
                      "COV=%umV CUV=%umV rsense=%luuOhm\r\n",
                      (int)PARAM_COUNT,
                      param_cache[PARAM_OCD1_THRESHOLD_A].u16,
                      param_cache[PARAM_SCD_THRESHOLD_A].u16,
                      param_cache[PARAM_CELL_OV_THRESHOLD_MV].u16,
                      param_cache[PARAM_CELL_UV_THRESHOLD_MV].u16,
                      (unsigned long)rsense);
    return 0;
}

