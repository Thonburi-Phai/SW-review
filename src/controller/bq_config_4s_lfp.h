/**
 * @file bq_config_4s_lfp.h
 * @brief BQ7690x configuration for a 4S LiFePO4 (LFP) battery pack:
 *        protection settings, register conversion macros and SOC defaults.
 */

#pragma once
#include <stdint.h>

/* ========== Battery Selection ========== */

#define BATT_15AH                   1
#define BATT_120AH                  2
#define BATT_50AH                   3
#define BATT_100AH                  4
#define BATTERY_CONFIG              BATT_100AH

/* ========== Battery Specifications (4S LiFePO4) ========== */

#if BATTERY_CONFIG == BATT_15AH
#define BATTERY_CAPACITY_MAH        15000
#define CFG_SOC_CELL_R_CHG_X10MOHM  30      /* 3.0 mΩ */
#elif BATTERY_CONFIG == BATT_50AH
#define BATTERY_CAPACITY_MAH        50000
#define CFG_SOC_CELL_R_CHG_X10MOHM  10      /* 1.0 mΩ */
#elif BATTERY_CONFIG == BATT_100AH
#define BATTERY_CAPACITY_MAH        100000
#define CFG_SOC_CELL_R_CHG_X10MOHM  27      /* 2.7 mΩ, provisional */

#elif BATTERY_CONFIG == BATT_120AH
#define BATTERY_CAPACITY_MAH        120000
#define CFG_SOC_CELL_R_CHG_X10MOHM  27      /* 2.7 mΩ */
/* Feeds cell_ocv_est(): the resting voltage a cell would return to if current stopped.
 * The termination anchor uses CFG_SOC_TERM_R_X10MOHM instead. */
#endif

#define BATTERY_CELLS               4       // Number of cells in series
#define BATTERY_NOMINAL_VOLTAGE_MV  3200    // Nominal cell voltage (mV)
#define BATTERY_MAX_VOLTAGE_MV      3600    // Maximum safe cell voltage (mV)
#define BATTERY_MIN_VOLTAGE_MV      2500    // Minimum safe cell voltage (mV)

/* ========== Current Sense Configuration ========== */

/**
 * Sense resistor value in µΩ
 *
 * Production: 200µΩ (0.2mΩ), max ±1000A (AFE ±200mV limit)
 * TEST:      5000µΩ (5mΩ),   max ±40A
 *
 * All conversion macros use this value automatically.
 * OCC valid range: 3-123mV → min 15A, max 615A @ 200µΩ
 * OCD valid range: 4-200mV → min 20A, max 1000A @ 200µΩ (10A/step)
 * SCD valid range: 10-500mV → min 50A, max 2500A @ 200µΩ
 */
#define CFG_RSENSE_MICRO_OHM        200     // Production: 0.2mΩ (200µΩ)
#define CFG_MAX_CURRENT_A           250     // CC1/CC2 full-scale ±250A @ 200µΩ

// CC2 (Current register 0x3A) used for display, CC1 used for PASSQ coulomb counting.
#define CFG_USE_RAW_CURRENT         0       // 0 = use gain-scaled CC2 (recommended)

/* ========== DA Config (0x9019) — ADC Speed Configuration ========== */

/**
 * BQ76907 DA Config register (H2, 0x9019) — NOT the same as BQ76952!
 * BQ76907 has NO USER_AMPS field. Bit layout (TRM Table 12-5, page 94):
 *
 *   Bit 8:    TSMODE      — 0=thermistor mode (Vref=VREG18), 1=ADCIN mode (Vref=VREF1)
 *   Bits 7-6: CCMODE[1:0] — CC1 power mode: 0=full, 1=low, 2=low (same)
 *   Bits 5-4: CVADCSPEED  — Cell voltage ADC speed (0=slow/16bit .. 3=fast/12bit)
 *   Bits 3-2: IADCSPEED   — Current ADC speed (0=2.93ms/16bit, 1=1.46ms, 2=732µs, 3=366µs)
 *   Bits 1-0: SSADCSPEED  — Shared-slot ADC speed (0=slow .. 3=fast)
 *
 * CC1/CC2 LSB is ~1mA @ 1mΩ OTP calibration (factory per-chip).
 * CC1/CC2 Gains are scaled at runtime in configure_current_gain() to
 * cover MAX_CURRENT_A without clipping. PASSQ integrates CC1.
 */
#define CFG_TSMODE                  0       /* 0=thermistor, 1=ADCIN */
#define CFG_CCMODE                  0       /* 0=full power CC1 */
#define CFG_CVADCSPEED              0       /* 0=slowest/best resolution */
#define CFG_IADCSPEED               0       /* 0=2.93ms/16-bit (best resolution) */
#define CFG_SSADCSPEED              0       /* 0=slowest/best resolution */
#define CFG_DA_CONFIG               ((CFG_TSMODE << 8) | (CFG_CCMODE << 6) | \
                                     (CFG_CVADCSPEED << 4) | (CFG_IADCSPEED << 2) | \
                                     CFG_SSADCSPEED)
/* Result: 0x0000 (all defaults — slowest/highest resolution) */

/* ========== Power Config (0x9014) — ADC Loop Speed ========== */

/**
 * BQ76907 Power Config register (H1, 0x9014) — NOT the same as BQ76952!
 * Bit layout (TRM Table 12-1, page 91):
 *
 *   Bits 7-6: LOOP_SLOW[1:0]    — ADC scan loop: 0=Full, 1=2x, 2=4x, 3=8x slower
 *   Bits 5-4: CB_LOOP_SLOW[1:0] — Balancing loop: 0=skip 1/2 ADSCAN, 1=3/4, 2=7/8, 3=15/16
 *   Bit 3:    OTSD              — 1=die overtemp causes SHUTDOWN
 *   Bit 2:    LFOWD             — 1=LFO watchdog failure causes SHUTDOWN
 *   Bit 1:    DPSLP_LFO         — 1=keep LFO running in DEEPSLEEP
 *   Bit 0:    SLEEP_EN          — 1=enable SLEEP mode (TRM default=1)
 *
 * SLEEP_EN=1 required for low-power automotive standby.
 */
#define CFG_LOOP_SLOW               0       /* [7:6] 0=Full speed */
#define CFG_CB_LOOP_SLOW            0       /* [5:4] 0=skip 1/2 ADSCAN (default) */
#define CFG_OTSD                    0       /* [3] 0=OT does not auto-shutdown */
#define CFG_LFOWD                   0       /* [2] 0=LFO watchdog does not shutdown */
#define CFG_DPSLP_LFO               0       /* [1] 0=LFO off in DEEPSLEEP */
#define CFG_SLEEP_EN                1       /* [0] 1=enable SLEEP mode (REQUIRED) */
#define CFG_POWER_CONFIG            ((CFG_LOOP_SLOW << 6) | (CFG_CB_LOOP_SLOW << 4) | \
                                     (CFG_OTSD << 3) | (CFG_LFOWD << 2) | \
                                     (CFG_DPSLP_LFO << 1) | CFG_SLEEP_EN)
/* Result: 0x01 (TRM default — full speed, sleep enabled) */

/* ========== Cell Voltage Protection Settings ========== */

/* COV/CUV delay registers count ADSCAN loops, not milliseconds.  One ADSCAN =
 * one conversion slot per active cell + one shared slot (TRM §5.2), each slot
 * 2.93 ms at CVADCSPEED=0 (halving per speed step).  4S → 5 × 2.93 ≈ 14.65 ms.
 * NOTE: with CB_LOOP_SLOW=0 every other ADSCAN is idle while balancing is
 * active, so the effective delay roughly doubles during balancing. */
#define CFG_CV_SLOT_US              (2930u >> CFG_CVADCSPEED)                 /* 2930/1465/732/366 µs */
#define CFG_ADSCAN_US               ((BATTERY_CELLS + 1u) * CFG_CV_SLOT_US)   /* 14650 µs for 4S */
#define BQ_MS_TO_ADSCAN(ms)         (((uint32_t)(ms) * 1000u + CFG_ADSCAN_US / 2u) / CFG_ADSCAN_US)
#define CFG_VOLT_PROT_DELAY_MAX_MS  ((255u * CFG_ADSCAN_US) / 1000u)          /* 3735 ms = register max 255 */

#define _BQ_CLAMP(x, lo, hi)  ((x) < (lo) ? (lo) : ((x) > (hi) ? (hi) : (x)))
#define _BQ_CLAMP_U8(x)  _BQ_CLAMP(x, 1u, 255u)

// Overvoltage Protection (OV)
#define CFG_CELL_OV_THRESHOLD_MV    3600    // LFP max
// Human-readable equivalents (parameter defaults and firmware calculations):
#define CFG_CELL_OV_DELAY_MS        3735    // Register max: 255 ADSCAN ≈ 3.7 s
#define CFG_CELL_OV_RECOVERY_MV     200     // 200mV widest hysteresis
// Compile-time register values (written by configure_voltage_protection):
#define CFG_CELL_OV_DELAY_ADSCAN    _BQ_CLAMP_U8(BQ_MS_TO_ADSCAN(CFG_CELL_OV_DELAY_MS))  // 255
#define CFG_CELL_OV_RECOVERY_REG    3       // Recovery hysteresis: 0=none, 1=50mV, 2=100mV, 3=200mV

// Undervoltage Protection (UV)
#define CFG_CELL_UV_THRESHOLD_MV    2500    // LFP deep discharge cutoff
// Human-readable equivalents:
#define CFG_CELL_UV_DELAY_MS        2535    // 173 ADSCAN ≈ 2.5 s
#define CFG_CELL_UV_RECOVERY_MV     100     // 100mV
// Compile-time register values:
#define CFG_CELL_UV_DELAY_ADSCAN    _BQ_CLAMP_U8(BQ_MS_TO_ADSCAN(CFG_CELL_UV_DELAY_MS))  // 173
#define CFG_CELL_UV_RECOVERY_REG    2       // Recovery hysteresis: 0=none, 1=50mV, 2=100mV, 3=200mV

/* ========== Current Protection Settings ========== */

/**
 * Current protection conversion macros for BQ76907
 * All macros use CFG_RSENSE_MICRO_OHM automatically.
 *
 * Formula: V(mV) = I(A) × R(µΩ) / 1000
 */

/* ========== Current to Register Conversion Macros ========== */

/**
 * OCC Threshold Conversion (Overcurrent in Charge)
 *   V_threshold = 2mV × register - 1mV
 *   Range: 3mV - 123mV (register 2 - 62)
 *   register = (V + 1) / 2 = (I × R_uΩ / 1000 + 1) / 2 = (I × R_uΩ + 1000) / 2000
 */
#define CURRENT_TO_OCC_REG(current_a) ( \
    (uint8_t)_BQ_CLAMP(((current_a) * CFG_RSENSE_MICRO_OHM + 1000) / 2000, 2u, 62u) \
)

/**
 * OCD1/OCD2 Threshold Conversion (Overcurrent in Discharge)
 *   V_threshold = 2mV × register
 *   Range: 4mV - 200mV (register 2 - 100)
 *   register = V / 2 = I × R_uΩ / 2000
 */
#define CURRENT_TO_OCD_REG(current_a) ( \
    (uint8_t)_BQ_CLAMP(((current_a) * CFG_RSENSE_MICRO_OHM) / 2000, 2u, 100u) \
)

/**
 * SCD Threshold Conversion (Short Circuit in Discharge)
 * Non-linear lookup by sense voltage: V(mV) = I(A) × R(µΩ) / 1000
 *
 * Setting | Threshold
 * --------|----------
 *    0    |   10 mV
 *    1    |   20 mV
 *    2    |   40 mV
 *    3    |   60 mV
 *    4    |   80 mV
 *    5    |  100 mV
 *    6    |  125 mV
 *    7    |  150 mV
 *    8    |  175 mV
 *    9    |  200 mV
 *   10    |  250 mV
 *   11    |  300 mV
 *   12    |  350 mV
 *   13    |  400 mV
 *   14    |  450 mV
 *   15    |  500 mV
 */
#define _SCD_MV(current_a) (((current_a) * CFG_RSENSE_MICRO_OHM) / 1000)

#define CURRENT_TO_SCD_REG(current_a) ( \
    (_SCD_MV(current_a) <= 10)  ? 0  : \
    (_SCD_MV(current_a) <= 20)  ? 1  : \
    (_SCD_MV(current_a) <= 40)  ? 2  : \
    (_SCD_MV(current_a) <= 60)  ? 3  : \
    (_SCD_MV(current_a) <= 80)  ? 4  : \
    (_SCD_MV(current_a) <= 100) ? 5  : \
    (_SCD_MV(current_a) <= 125) ? 6  : \
    (_SCD_MV(current_a) <= 150) ? 7  : \
    (_SCD_MV(current_a) <= 175) ? 8  : \
    (_SCD_MV(current_a) <= 200) ? 9  : \
    (_SCD_MV(current_a) <= 250) ? 10 : \
    (_SCD_MV(current_a) <= 300) ? 11 : \
    (_SCD_MV(current_a) <= 350) ? 12 : \
    (_SCD_MV(current_a) <= 400) ? 13 : \
    (_SCD_MV(current_a) <= 450) ? 14 : 15 \
)

/* ========== Current Protection Configuration ========== */

// Overcurrent in Charge (OCC)
// Charge-side only. @ 200µΩ: reg=(200×200+1000)/2000=20, and 20×2000/200 reads
// back 200 A with no rounding. Register ceiling is reg 62 = 620 A @ 200 µΩ.
#define CFG_OCC_THRESHOLD_A         200     // 2x the software charge limit
#define CFG_OCC_THRESHOLD_REG       CURRENT_TO_OCC_REG(CFG_OCC_THRESHOLD_A)
#define CFG_OCC_DELAY_REG           0x85    // ~201ms, as OCD1/OCD2

// Overcurrent in Discharge 1 (OCD1) - Moderate level
// @ 200µΩ: reg=(700×200)/2000=70 → V=140mV → actual 700A
#define CFG_OCD1_THRESHOLD_A        700     // 700A discharge overcurrent
#define CFG_OCD1_THRESHOLD_REG      CURRENT_TO_OCD_REG(CFG_OCD1_THRESHOLD_A)
#define CFG_OCD1_DELAY_REG          0x85    // ~201ms (range 3: 181.5 + 4×4.88ms)

// Overcurrent in Discharge 2 (OCD2) - High level / burst
// @ 200µΩ: reg=(850×200)/2000=85 → V=170mV → actual 850A
#define CFG_OCD2_THRESHOLD_A        850     // 850A burst overcurrent
#define CFG_OCD2_THRESHOLD_REG      CURRENT_TO_OCD_REG(CFG_OCD2_THRESHOLD_A)
#define CFG_OCD2_DELAY_REG          0x85    // ~201ms (range 3: 181.5 + 4×4.88ms)

// Short Circuit in Discharge (SCD)
// @ 200µΩ: V=2000×200/1000=400mV → setting 13 (400mV) → actual 2000A
#define CFG_SCD_THRESHOLD_A         2000    // 2000A target
#define CFG_SCD_THRESHOLD_REG       CURRENT_TO_SCD_REG(CFG_SCD_THRESHOLD_A)
#define CFG_SCD_DELAY_REG           0x05    // 488µs (mid-range, setting 5 of 0-10)

// Latch Limit - permanent lockout after N overcurrent recovery cycles
// CURRLAT[2:0] encoding: 0=disabled, 1=2, 2=4, 3=8, 4=16, 5=32, 6=48, 7=96 retries
// 0 = disabled. A latch clears only on a host PROT_RECOVERY, for which this
// board has no working route, so a latch would be unrecoverable in the field.
#define CFG_LATCH_LIMIT             0

// Current Protection Recovery -- shared by OCC, OCD1, OCD2 and SCD.
// With CFG_LATCH_LIMIT at 0 this is the only bound on repeated fault cycling.
#define CFG_CURRENT_RECOVERY_TIME_S 5       // 5s

/* ========== Temperature Protection Settings ========== */

/**
 * External Thermistor (TS pin) Temperature Protection
 *
 * IMPORTANT: OTC/UTC/OTD/UTD use REGISTER VALUES (0-255), not °C!
 * - Overtemperature (OTC/OTD): Register value × 54 = ADC threshold
 * - Undertemperature (UTC/UTD): Register value × 77 = ADC threshold
 *
 * For NTC 10kΩ @ 25°C with B=3950K:
 * Temp → Resistance → Voltage → ADC counts → Register value
 *
 * Lookup table (approximate for NTC 10kΩ):
 * -20°C → 9779 ADC / 77 = 127 (UTC/UTD)
 * -15°C → 8855 ADC / 77 = 115
 *   0°C → 7478 ADC / 77 = 97
 *   5°C → 6545 ADC / 77 = 85
 *  25°C → 6522 ADC / 54 = 121 (OTC/OTD)
 *  45°C → 3500 ADC / 54 = 65
 *  50°C → 3186 ADC / 54 = 59
 *  55°C → 2800 ADC / 54 = 52
 *  60°C → 2543 ADC / 54 = 47
 */

/* ========== Temperature Conversion Helper Macros (NTC 10kΩ) ========== */

/**
 * Convert temperature (°C) to resistance (Ω) for NTC 10kΩ @ 25°C, B=3950K
 * Using simplified Steinhart-Hart: R = R0 × exp(B × (1/T - 1/T0))
 *
 * Piecewise linear approximation for speed:
 * - Uses different coefficients for different temperature ranges
 * - Accurate to ±2°C across -20°C to 80°C range
 */
#define TEMP_TO_RESISTANCE(temp_c) ( \
    ((temp_c) <= 0)  ? (27500L - ((temp_c) * 700L)) : \
    ((temp_c) <= 25) ? (10000L + ((25 - (temp_c)) * 450L)) : \
    ((temp_c) <= 60) ? (10000L - (((temp_c) - 25) * 200L)) : \
                       (3000L - (((temp_c) - 60) * 50L)) \
)

/**
 * Convert resistance (Ω) to ADC counts
 * ADC_counts = (V_TS / 92µV) where V_TS = 1.8V × R_NTC / (20000 + R_NTC)
 * ADC = (1800 × R_NTC × 1000) / ((20000 + R_NTC) × 92)
 * Simplified: ADC ≈ (R_NTC × 19565) / (20000 + R_NTC)
 */
#define RESISTANCE_TO_ADC(r_ohm) ( \
    ((int32_t)(r_ohm) * 19565L) / (20000L + (int32_t)(r_ohm)) \
)

/**
 * Convert temperature (°C) to register value for OVERTEMPERATURE protection (OTC/OTD)
 * Register value = ADC_counts / 54
 *
 * Usage: TEMP_TO_OT_REG(50) → 59 (for 50°C threshold)
 */
#define TEMP_TO_OT_REG(temp_c) ( \
    (uint8_t)(RESISTANCE_TO_ADC(TEMP_TO_RESISTANCE(temp_c)) / 54) \
)

/**
 * Convert temperature (°C) to register value for UNDERTEMPERATURE protection (UTC/UTD)
 * Register value = ADC_counts / 77
 *
 * Usage: TEMP_TO_UT_REG(0) → 97 (for 0°C threshold)
 */
#define TEMP_TO_UT_REG(temp_c) ( \
    (uint8_t)(RESISTANCE_TO_ADC(TEMP_TO_RESISTANCE(temp_c)) / 77) \
)

// Overtemperature in Charge (OTC) - Uses TS pin (external thermistor)
#define CFG_OTC_THRESHOLD_C         60      // Trip when charging > 60°C
#define CFG_OTC_THRESHOLD_REG       TEMP_TO_OT_REG(CFG_OTC_THRESHOLD_C)
#define CFG_OTC_DELAY_MEAS          4       // 4 measurements (not seconds; actual delay depends on ADC loop)
#define CFG_OTC_RECOVERY_C          55      // Recover at 55°C
#define CFG_OTC_RECOVERY_REG        TEMP_TO_OT_REG(CFG_OTC_RECOVERY_C)

// Undertemperature in Charge (UTC) - Uses TS pin (external thermistor)
// AUTOMOTIVE: LiFePO4 MUST NOT charge below 0°C (lithium plating risk!)
#define CFG_UTC_THRESHOLD_C         0       // Trip when charging < 0°C (CRITICAL!)
#define CFG_UTC_THRESHOLD_REG       TEMP_TO_UT_REG(CFG_UTC_THRESHOLD_C)
#define CFG_UTC_DELAY_MEAS          4       // 4 measurements
#define CFG_UTC_RECOVERY_C          5       // Recover at 5°C
#define CFG_UTC_RECOVERY_REG        TEMP_TO_UT_REG(CFG_UTC_RECOVERY_C)

// Overtemperature in Discharge (OTD) - Uses TS pin (external thermistor)
#define CFG_OTD_THRESHOLD_C         60      // Trip when discharging > 60°C
#define CFG_OTD_THRESHOLD_REG       TEMP_TO_OT_REG(CFG_OTD_THRESHOLD_C)
#define CFG_OTD_DELAY_MEAS          4       // 4 measurements
#define CFG_OTD_RECOVERY_C          55      // Recover at 55°C
#define CFG_OTD_RECOVERY_REG        TEMP_TO_OT_REG(CFG_OTD_RECOVERY_C)

// Undertemperature in Discharge (UTD) - Uses TS pin (external thermistor)
#define CFG_UTD_THRESHOLD_C         -30     // Trip when discharging < -30°C
#define CFG_UTD_THRESHOLD_REG       TEMP_TO_UT_REG(CFG_UTD_THRESHOLD_C)
#define CFG_UTD_DELAY_MEAS          4       // 4 measurements
#define CFG_UTD_RECOVERY_C          -25     // Recover at -25°C
#define CFG_UTD_RECOVERY_REG        TEMP_TO_UT_REG(CFG_UTD_RECOVERY_C)

/* ========== MCU-side NTC safety (ADC channels, see ntc_fault.h) ========== */

/* Master enable for the MCU temperature bands AND the NTC open-wire force-off.
 * 0 = both layers measure, classify and report; no FET is ever touched. */
#define CFG_NTC_SAFETY_EN           0

/* Charge ladder, CH1-CH3, four adjustable points. Below the first point and at
 * or above the last, charging is blocked; between the first two the main charge
 * FET is off and the pre-charge FET is on. */
#define CFG_NTC_CHG_MIN_C           (-10)   // CHG OFF below this
#define CFG_NTC_CHG_PRECHG_END_C    0       // pre-charge band ends here
#define CFG_NTC_CHG_NORMAL_MAX_C    55      // normal charge ends, extended begins
#define CFG_NTC_CHG_MAX_C           100     // CHG OFF at or above this

/* Discharge ladder, CH1-CH3, three adjustable points. */
#define CFG_NTC_DSG_MIN_C           (-20)   // DSG OFF below this
#define CFG_NTC_DSG_NORMAL_MAX_C    60      // normal discharge ends, extended begins
#define CFG_NTC_DSG_MAX_C           100     // DSG OFF at or above this

/* MOSFET channel, CH4: outside this range both FETs are held open. */
#define CFG_NTC_FET_MIN_C           (-20)
#define CFG_NTC_FET_MAX_C           100

/* A band already entered is held until the temperature falls this far back
 * through the point that was crossed. Must stay smaller than the smallest gap
 * between two adjacent points of any ladder. */
#define CFG_NTC_HYST_C              3

/* A band must forbid a FET for this long before the request is raised. */
#define CFG_NTC_TRIP_DELAY_MS       2000

// Internal Die Temperature Protection (uses °C directly!)
#define CFG_OTINT_THRESHOLD_C       85      // Trip when die > 85°C
#define CFG_OTINT_DELAY_MEAS        1       // 1 measurement
#define CFG_OTINT_RECOVERY_C        80      // Recover at 80°C

/* ========== Cell Balancing Settings ========== */

#define CFG_BAL_START_MV            5       // Start balancing when delta >= 5mV
#define CFG_BAL_STOP_MV             2       // Stop when delta < 2mV
#define CFG_BAL_MIN_CELL_MV         2800    // Don't balance if cell < 2.8V (safety floor)
#define CFG_BAL_MIN_START_CELL_MV   3350    // Only balance when vmax >= 3.35V (LFP absorption region)
#define CFG_BAL_MAX_TEMP_C          60      // Don't balance if temp > 60°C
#define CFG_BAL_MAX_DISCHARGE_A     1       // Don't balance if discharge > 1A (charge any value OK)
#define CFG_BAL_ALLOW_ADJACENT      0       // 0=no, 1=yes (adjacent cell balancing)

/* Balance duty-cycle timing.
 * Each "tick" = one call to run_balancing_task() = 600ms.
 * Cycle phases (percentage of CYCLE_TICKS):
 *   0%  → balance OFF (CB_ACTIVE_CELLS=0, cells settle)
 *   20% → measure+decide (voltages clean, run cell selection)
 *   ON% → balance ON (apply decided mask, refresh until cycle wraps)
 * Duty cycle = (100 - ON_PCT)%.
 * Default: 10 ticks (6s cycle), ON at 30% → 70% duty, 4.2s balance / 1.8s off.
 */
#define CFG_BAL_CYCLE_TICKS         10      // Total ticks per cycle (10 × 600ms = 6s)
#define CFG_BAL_ON_PCT              30      // % into cycle when balance turns ON (duty = 100-this)

/* ========== Sleep/Shutdown Settings ========== */

#define CFG_SLEEP_CURRENT_MA        10      // BQ enters SLEEP when |current| < 10mA for VoltageTime
#define CFG_SLEEP_VOLTAGE_TIME_S    10      // 10s idle before entering BQ SLEEP
#define CFG_WAKE_COMPARATOR_CURR    1       // TRM: threshold = 500µV × setting. At 200µΩ: 500µV/200µΩ = 2.5A

/* ALL autonomous SHUTDOWN paths DISABLED — PERMANENT PROJECT RULE, production
 * as well as bench builds.  Do NOT re-enable any of these four registers, nor
 * PowerConfig[OTSD]/[LFOWD] (see CFG_POWER_CONFIG).
 *
 * A chip-initiated SHUTDOWN kills I2C and REGOUT and erases all RAM config.
 * The chip evaluates a single sample with no debounce and the 10-second commit
 * cannot be aborted by voltage recovery (TRM §8.5) — only by host RESET() /
 * EXIT_DEEPSLEEP(), which also wipe config and clear FET_EN.
 *
 * What replaces it: cell protection is CUV opening the DSG FET (autonomous);
 * open-wire detection is MCU software on cell-voltage signatures; storage
 * shutdown is an MCU decision via bq_controller_enter_shutdown() /
 * BPCMD_ENTER_SHUTDOWN.  0 = disabled per TRM §12.4.2 and is the chip
 * factory default.  See docs/bq76907_balance_wire_open_detection.md §11. */
#define CFG_SHUTDOWN_CELL_MV        0       // 0 = cell-voltage shutdown disabled
#define CFG_SHUTDOWN_STACK_MV       0       // 0 = stack-voltage shutdown disabled
#define CFG_SHUTDOWN_TEMP_C         0       // 0 = internal-temp shutdown disabled
#define CFG_AUTO_SHUTDOWN_TIME_MIN  0       // 0 = auto re-shutdown timer disabled

/* ========== FET Settings ========== */

// FET Options register (0x901E)
// Bit 0: PROTRCVR - Allow PROT_RECOVERY command in SEALED mode
// Bit 2: FET_EN - Autonomous FET enable (protection-based FET control)
// Bit 4: SLEEPCHG - Allow charging in SLEEP mode
// Bit 5: HOST_FETON - Allow host to manually turn ON FETs
// Bit 6: HOST_FETOFF - Allow host to manually turn OFF FETs
#define CFG_FET_OPTIONS             (BQ_FETOPT_PROTRCVR | BQ_FETOPT_SLEEPCHG | \
                                     BQ_FETOPT_HOST_FETON | BQ_FETOPT_HOST_FETOFF | \
                                     BQ_FETOPT_FET_EN)
#define CFG_CHARGE_DETECTOR_TIME_100MS 5    // Charge detection time: 5 × 100ms = 500ms (TRM unit=100ms)

/* ========== Watchdog Settings ========== */

#define CFG_HOST_WATCHDOG_TIMEOUT_S 0       // 0 = disabled, or set timeout in seconds

/* ========== ALERT Pin Configuration ========== */

/**
 * AlarmEnable register (0x66) selects which events drive the ALERT pin.
 * ALERT is active LOW. Read SafetyStatus/SafetyAlert registers after it asserts.
 *
 * AlarmStatus uses aggregate flags: SSA/SSB/SAA/SAB trigger when ANY
 * corresponding safety/alert bit is set.
 *
 * XCHG/XDSG are NOT included: they trigger whenever FETs are disabled (even
 * after clearing the root cause), causing infinite loops.
 */
#define CFG_ALARM_ENABLE_MASK   (BQ_ALARM_SSA | BQ_ALARM_SSB | \
                                 BQ_ALARM_SAA | BQ_ALARM_SAB | \
                                 BQ_ALARM_SHUTV | BQ_ALARM_WAKE | \
                                 BQ_ALARM_SLEEP)

// Optional additional alarms (uncomment if needed):
// | BQ_ALARM_CB           // Cell balancing active
// | BQ_ALARM_FULLSCAN     // Full ADC scan complete
// | BQ_ALARM_ADSCAN       // ADC scan complete
// | BQ_ALARM_TIMER        // Timer expired
// | BQ_ALARM_INITCOMP     // Init complete
// | BQ_ALARM_POR          // Power-on reset

// Note: BQ_ALARM_* bitmasks are defined in bq7690x/bq7690x_status.h

/* ========== Default Alarm Mask (0x901C) — ALERT at Power-On ========== */

/**
 * DefaultAlarmMask takes effect immediately after POR/CMD_RESET, before
 * firmware runs configure_alert_pin(), so ALERT fires during the boot window.
 */
#define CFG_DEFAULT_ALARM_MASK      CFG_ALARM_ENABLE_MASK

/* ========== Protection Enable Masks ========== */

// Use definitions from bq7690x_protection.h instead of redefining them
// Import: BQ_PROTA_*, BQ_PROTB_*, BQ_DSGPROTA_*, BQ_CHGPROTA_*, BQ_BOTHPROTB_*

// Enabled Protections A - Which protections are enabled
// CURLATCH omitted: a latch cannot be cleared on this board (see CFG_LATCH_LIMIT).
#define CFG_ENABLED_PROTECTIONS_A   (BQ_PROTA_SCD | BQ_PROTA_OCD1 | BQ_PROTA_OCD2 | \
                                     BQ_PROTA_OCC | BQ_PROTA_COV | BQ_PROTA_CUV)

// Enabled Protections B - Which protections are enabled
// All temperature protections enabled: OTC/UTC critical for LFP safety
#define CFG_ENABLED_PROTECTIONS_B   (BQ_PROTB_OTD | BQ_PROTB_OTC | \
                                     BQ_PROTB_UTC | BQ_PROTB_UTD | \
                                     BQ_PROTB_OTINT)

/* ========== FET Protection Actions ========== */

// DSG FET Protections A - Which faults turn off discharge FET
#define CFG_DSG_FET_PROTECTIONS_A   (BQ_DSGPROTA_CUV | BQ_DSGPROTA_SCD | \
                                     BQ_DSGPROTA_OCD1 | BQ_DSGPROTA_OCD2 | \
                                     BQ_DSGPROTA_OTD | BQ_DSGPROTA_OTINT)

// CHG FET Protections A - Which faults turn off charge FET
// UTC: prevents charging LFP below 0°C (lithium plating!)
// OTC/OTINT: stop charging on overtemperature
#define CFG_CHG_FET_PROTECTIONS_A   (BQ_CHGPROTA_COV | BQ_CHGPROTA_OCC | \
                                     BQ_CHGPROTA_UTC | BQ_CHGPROTA_OTC | \
                                     BQ_CHGPROTA_OTINT)

// Both FET Protections B - Which faults turn off both FETs
// VREF/VSS: hardware reference faults → unsafe to run FETs
#define CFG_BOTH_FET_PROTECTIONS_B  (BQ_BOTHPROTB_VREF | BQ_BOTHPROTB_VSS)

/* ========== Body Diode Threshold ========== */

// Body diode detection threshold in userA (current, NOT mV!)
// TRM: I2, min=0, max=32767, default=64 userA
// At 1mΩ calibration: 64 userA = 64mA. Detects body diode conduction when
// FET is off and current exceeds this threshold.
#define CFG_BODY_DIODE_THRESHOLD    64      // Default: 64 userA

/* ========== Current Conversion Helper Macros ========== */

/**
 * RawCurrent to Amperes conversion (parametric)
 *
 * V_LSB = Vref2 / (5 × 2^23) = 1.227V / 41943040 ≈ 29.23 nV
 * I(mA) = (RawCurrent × 1227000) / (41943 × CFG_RSENSE_MICRO_OHM)
 *
 * Uses int64_t to avoid overflow with large raw values.
 */
#define RAW_CURRENT_TO_MA(raw)  ((int32_t)(((int64_t)(raw) * 1227000LL) / (41943LL * CFG_RSENSE_MICRO_OHM)))
#define RAW_CURRENT_TO_A(raw)   ((int32_t)(((int64_t)(raw) * 1227LL) / (41943LL * CFG_RSENSE_MICRO_OHM)))

/**
 * Current (mA) to RawCurrent conversion (for testing)
 */
#define MA_TO_RAW_CURRENT(ma)   ((int32_t)(((int64_t)(ma) * 41943LL * CFG_RSENSE_MICRO_OHM) / 1227000LL))

/* ========== NTC 10kΩ Resistance-to-Temperature Lookup Table ========== */

/**
 * NTC 10kΩ @ 25°C, B=3950K
 * Verified against Steinhart-Hart: 1/T = 1/298.15 + (1/3950) × ln(R/10000)
 * Resistance descending (temperature ascending).
 * Linear interpolation between entries gives ±1°C accuracy.
 */
typedef struct {
    int32_t resistance_ohm;
    int16_t temp_c;
} ntc_lut_entry_t;

static const ntc_lut_entry_t NTC_LUT[] = {
    { 195652,  -40 },
    { 120000,  -30 },
    {  75000,  -20 },
    {  47810,  -10 },
    {  30000,    0 },
    {  20000,   10 },
    {  14120,   15 },
    {  10000,   25 },
    {   7150,   35 },
    {   5325,   40 },
    {   3600,   50 },
    {   2490,   60 },
    {   1750,   70 },
    {   1250,   80 },
    {    900,   90 },
    {    800,  100 },
};

#define NTC_LUT_SIZE (sizeof(NTC_LUT) / sizeof(NTC_LUT[0]))

/* ========== SOC Estimation — Coulomb Counting Config ========== */

/* Endpoint anchoring — bottom */
#define CFG_SOC_STEEP_ENTRY_MV          3000    /* Below this: steep bottom region, voltage corrections allowed */
#define CFG_SOC_HARD_BOTTOM_MV          2500    /* Absolute empty: SoC = 0% */
#define CFG_SOC_BOTTOM_ANCHOR_RECOVER_MV 2900   /* Re-arm hard anchor when cell_min recovers */
#define CFG_SOC_JUMP_DETECT_C_X10        50    /* Flag if implied current > 5.0C (data corruption) */

/* Rate-limited soft correction — bottom (steep region) */
#define CFG_SOC_SOFT_CORR_STEP_X10      5       /* Max correction per tick: 0.5% (units of 0.1%) */
#define CFG_SOC_SOFT_CORR_HOLD_MS       8000    /* Voltage must stay in steep for 8s before correcting (crank 1-5 s) */
#define CFG_SOC_SOFT_CORR_MIN_MA        200     /* Minimum discharge current for correction (avoids noise) */
#define CFG_SOC_GROSS_ERROR_SOC         40      /* Hard snap if SOC > this but voltage in steep region */
#define CFG_SOC_CELL_R_MOHM             10      /* retired; bottom comp uses PARAM_SOC_CELL_R_CHG_X10MOHM + hold time */
/* CFG_SOC_CELL_R_CHG_X10MOHM — defined in Battery Selection above */

/* Rate-limited soft correction — top (steep upper region during charge) */
#define CFG_SOC_STEEP_TOP_ENTRY_MV      3450    /* Above this: steep top region, upward corrections allowed */
#define CFG_SOC_TOP_SOFT_CORR_STEP_X10  5       /* Max upward correction per tick: 0.5% */
#define CFG_SOC_TOP_SOFT_CORR_HOLD_MS   3000    /* Voltage must stay in steep top for 3s before correcting */
#define CFG_SOC_TOP_SOFT_CORR_MIN_MA    200     /* Minimum charge current for top correction */

/* ========== SOC anchoring v2 — docs/soc_anchoring_redesign_plan.md ==========
 * Graded anchors, per-cell offsets, TI-style capacity learning, dV/dt rest qualification,
 * convergence engine, current-sign guard.  All are runtime parameters; these are defaults. */
#define CFG_SOC_DISQ_LO_MV              3270    /* rest OCV inside [LO,HI] carries no SOC info (TI band) */
#define CFG_SOC_DISQ_HI_MV              3350
#define CFG_SOC_REST_C_X100             5       /* rest = |I| <= C/20 (6 A @120Ah), IR-corrected OCV */
#define CFG_SOC_REST_MIN_MIN            10      /* minimum rest before an anchor (min) */
#define CFG_SOC_REST_MAX_MIN            300     /* fallback ceiling if dV/dt never settles (min) */
#define CFG_SOC_REST_DVDT_X10UV_S       40      /* settled when |dV/dt| < 4.0 µV/s over 100 s windows */
#define CFG_SOC_TOP_REST_MV             3400    /* top rest anchor: highest cell OCV >= this */
#define CFG_SOC_BOT_REST_MV             3000    /* bottom rest anchor: lowest cell OCV <= this */
#define CFG_SOC_SPREAD_HIGH_MV          30      /* balanced: all offsets 0, pack = 100 at top */
#define CFG_SOC_SPREAD_MED_MV           50      /* taper anchor refused above this spread */
#define CFG_SOC_TAPER_CELL_MV           3450    /* taper: all cells IR-corrected >= this */
#define CFG_SOC_TAPER_ENTRY_C_X100      4       /* taper: 0 < I <= C/25 (4.8 A @120Ah) */
#define CFG_SOC_TAPER_EXIT_C_X100       6       /* taper hysteresis exit: I >= C/16 */
#define CFG_SOC_TAPER_HOLD_S            180     /* taper hold: 3 min */
#define CFG_SOC_CONV_REST_PCT_PER_MIN   6       /* convergence at rest: 0.1 %/s */
#define CFG_SOC_CONV_LOAD_MIN_X10_PCT_PER_MIN 5 /* convergence floor under load: 0.5 %/min */
#define CFG_SOC_CONV_LOAD_MAX_PCT_PER_MIN     5 /* convergence ceiling under load: 5 %/min */

/* Endpoint pull, replacing the rate-limited soft corrections and the gross-error snap.
 *
 * Outer zone: the correction removes a fraction of the error per amp-hour moved rather
 * than per minute, so it is scaled by C-rate and stops at zero current.  Direction
 * gated: down only while discharging, up only while charging.
 *
 * Inner zone: within the last stretch before the cell's limit the current gate is
 * dropped and the correction runs at its own fixed rate.  It must NOT use the
 * convergence engine's load formula, which is error over time-to-empty: time-to-empty
 * comes from the gauge's own remaining capacity, so a badly wrong gauge believes there
 * are hours left and crawls while the pack empties. */
#define CFG_SOC_PULL_Q_TAU_PCT          5       /* error decays 1/e per this much of capacity moved */
#define CFG_SOC_BOT_PULL_ENTRY_MV       3100    /* corrected lowest cell: outer zone below this */
#define CFG_SOC_BOT_HARD_MV             2850    /* and inner, current-blind, below this (~1 %) */
#define CFG_SOC_TOP_PULL_ENTRY_MV       3400    /* corrected highest cell: outer zone above this */
#define CFG_SOC_TOP_HARD_MV             3520    /* and inner, current-blind, above this */
#define CFG_SOC_PULL_HARD_PCT_PER_MIN   30      /* inner-zone rate: 0.5 %/s */

/* A learned capacity is eased in over this long rather than applied in one step. */
#define CFG_SOC_CAP_RAMP_S              300
#define CFG_SOC_LEARN_MIN_DSOC_PCT      37      /* TI: two qualified points >= 37 % apart */
#define CFG_SOC_LEARN_TEMP_MIN_C        10      /* retired; kept so ids do not shift */
#define CFG_SOC_LEARN_TEMP_MAX_C        40      /* retired */

/* How stale a stored learning point may be before it stops being pairable.
 *
 * The millisecond clock wraps every 49.7 days, so learning points are aged out
 * continuously rather than only being tested when a pair is attempted; otherwise a
 * point older than the wrap would alias to a young one and be accepted. */
#define CFG_SOC_LEARN_MAX_AGE_H         336     /* 14 days */

/* How much of the FIRST qualified measurement is taken, against the nameplate it
 * replaces: 70 means the measurement carries 70 % of the decision and the nameplate
 * 30 %. */
#define CFG_SOC_LEARN_FIRST_W_PCT       70
/* Per-learn step bounds, CEDV-style asymmetric.  Percentages of the LEARNED capacity,
 * not of the nameplate. */
#define CFG_SOC_LEARN_STEP_DOWN_PCT     2
#define CFG_SOC_LEARN_STEP_UP_PCT       4
#define CFG_SOC_SIGN_GUARD_MV           40      /* pack dV over 10 s against the current sign, steady I */

/* Termination anchor (T4): a termination is evidence about the battery only if the cell
 * was at its knee rather than pushed there by the rate.  Error bound:
 *
 *     error in % SOC = (|I| * R_cell) / (dV/dSOC at the terminating voltage)
 */
#define CFG_SOC_TERM_C_X100             100     /* 1C: a sanity backstop, not the deciding test */
#define CFG_SOC_TERM_ERR_X10_PCT        100     /* give up if the correction would exceed 10.0 % SOC */

/* Cell resistance for the DISCHARGE side of the termination bound, separate from the
 * charge figure and larger: a cell's resistance climbs as it empties. */
#define CFG_SOC_TERM_R_DSG_X10MOHM      30      /* 3.0 mΩ, unmeasured */
/* Resistance for the termination error bound only: resistance at the sampling interval,
 * which is the right figure here even though it is the wrong one for OCV estimation. */
#define CFG_SOC_TERM_R_X10MOHM          10      /* 1.0 mΩ, measured */
/* How far back to look for the current that caused a protection trip.  The chip's own
 * detection delays are 2.7 to 3.7 s (COV/CUV count ADSCAN loops), and the MCU then polls
 * the status, so the causing current is several seconds in the past by the time the
 * anchor runs.  The largest magnitude in the window is taken. */
#define CFG_SOC_TERM_PRETRIP_MS         6000
#define CFG_SOC_TERM_PRETRIP_SAMPLES    5       /* median of this many samples */

/* Drift compensation */
#define CFG_SOC_INVISIBLE_DRAIN_UA      1100    /* MCU+BQ ~1.1mA idle */

/* Feature enables */
#define CFG_SOC_TEMP_COMPENSATION_EN    0       /* 1=derate capacity by temperature */
#define CFG_SOC_CAPACITY_LEARNING_EN    1       /* default for PARAM_SOC_LEARN_EN */

/**
 * Convert NTC resistance (Ω) to temperature (°C) using LUT + linear interpolation.
 * Returns clamped values at table boundaries.
 */
static inline int16_t ntc_resistance_to_temp(int32_t r_ohm)
{
    // Above table range → clamp to coldest
    if (r_ohm >= NTC_LUT[0].resistance_ohm) {
        return NTC_LUT[0].temp_c;
    }
    // Below table range → clamp to hottest
    if (r_ohm <= NTC_LUT[NTC_LUT_SIZE - 1].resistance_ohm) {
        return NTC_LUT[NTC_LUT_SIZE - 1].temp_c;
    }

    // Find bracketing entries (resistance is descending)
    for (uint16_t i = 0; i < NTC_LUT_SIZE - 1; i++) {
        if (r_ohm <= NTC_LUT[i].resistance_ohm && r_ohm >= NTC_LUT[i + 1].resistance_ohm) {
            int32_t r_hi = NTC_LUT[i].resistance_ohm;
            int32_t r_lo = NTC_LUT[i + 1].resistance_ohm;
            int16_t t_hi = NTC_LUT[i].temp_c;      // colder (lower temp)
            int16_t t_lo = NTC_LUT[i + 1].temp_c;  // warmer (higher temp)

            // Linear interpolation: t = t_hi + (t_lo - t_hi) × (r_hi - r) / (r_hi - r_lo)
            int32_t dt = t_lo - t_hi;
            int32_t dr = r_hi - r_lo;
            if (dr == 0) return t_hi;
            return (int16_t)(t_hi + (dt * (r_hi - r_ohm)) / dr);
        }
    }

    return NTC_LUT[NTC_LUT_SIZE - 1].temp_c;
}
