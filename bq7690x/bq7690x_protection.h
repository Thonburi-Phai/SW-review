/**
 * @file bq7690x_protection.h
 * @brief BQ7690x protection enable, FET-action and threshold/delay registers.
 */

#pragma once
#include "bq7690x.h"
#include "bq7690x_reg.h"

/* ========== Enabled Protections Bitmasks ========== */

/* Enabled Protections A (Table 12-10) */
#define BQ_PROTA_COV        (1u << 7)  /* Cell Overvoltage */
#define BQ_PROTA_CUV        (1u << 6)  /* Cell Undervoltage */
#define BQ_PROTA_SCD        (1u << 5)  /* Short-Circuit in Discharge */
#define BQ_PROTA_OCD1       (1u << 4)  /* Overcurrent Discharge 1 */
#define BQ_PROTA_OCD2       (1u << 3)  /* Overcurrent Discharge 2 */
#define BQ_PROTA_OCC        (1u << 2)  /* Overcurrent Charge */
#define BQ_PROTA_CURLATCH   (1u << 1)  /* Current latch protection */
#define BQ_PROTA_REGOUT     (1u << 0)  /* REGOUT safety check */

/* Enabled Protections B (Table 12-11) */
#define BQ_PROTB_OTD        (1u << 5)  /* Overtemperature in Discharge */
#define BQ_PROTB_OTC        (1u << 4)  /* Overtemperature in Charge */
#define BQ_PROTB_UTD        (1u << 3)  /* Undertemperature in Discharge */
#define BQ_PROTB_UTC        (1u << 2)  /* Undertemperature in Charge */
#define BQ_PROTB_OTINT      (1u << 1)  /* Internal Overtemperature */
#define BQ_PROTB_HWD        (1u << 0)  /* Host Watchdog */

/* DSG FET Protections A (Table 12-12) */
#define BQ_DSGPROTA_CUV     (1u << 7)  /* Cell Undervoltage disables DSG */
#define BQ_DSGPROTA_SCD     (1u << 6)  /* Short-Circuit Discharge disables DSG */
#define BQ_DSGPROTA_OCD1    (1u << 5)  /* Overcurrent Discharge 1 disables DSG */
#define BQ_DSGPROTA_OCD2    (1u << 4)  /* Overcurrent Discharge 2 disables DSG */
#define BQ_DSGPROTA_HWD     (1u << 3)  /* Host Watchdog disables DSG */
#define BQ_DSGPROTA_OTD     (1u << 2)  /* Over-Temp Discharge disables DSG */
#define BQ_DSGPROTA_UTD     (1u << 1)  /* Under-Temp Discharge disables DSG */
#define BQ_DSGPROTA_OTINT   (1u << 0)  /* Internal Over-Temp disables DSG */

/* CHG FET Protections A (Table 12-13) */
#define BQ_CHGPROTA_COV     (1u << 7)  /* Cell Overvoltage disables CHG */
#define BQ_CHGPROTA_SCD     (1u << 6)  /* Short-Circuit Discharge disables CHG */
#define BQ_CHGPROTA_OCC     (1u << 5)  /* Overcurrent Charge disables CHG */
#define BQ_CHGPROTA_HWD     (1u << 3)  /* Host Watchdog disables CHG */
#define BQ_CHGPROTA_OTC     (1u << 2)  /* Over-Temp Charge disables CHG */
#define BQ_CHGPROTA_UTC     (1u << 1)  /* Under-Temp Charge disables CHG */
#define BQ_CHGPROTA_OTINT   (1u << 0)  /* Internal Over-Temp disables CHG */

/* Both FET Protections B (Table 12-14) */
#define BQ_BOTHPROTB_VREF   (1u << 2)  /* VREF check disables both FETs */
#define BQ_BOTHPROTB_VSS    (1u << 1)  /* VSS check disables both FETs */
#define BQ_BOTHPROTB_REGOUT (1u << 0)  /* REGOUT flag disables both FETs */

/* ========== Enabled Protections ========== */

bq_status_t bq7690x_read_enabled_protections_a(bq7690x_t *dev, uint8_t *value);
bq_status_t bq7690x_write_enabled_protections_a(bq7690x_t *dev, uint8_t value);

bq_status_t bq7690x_read_enabled_protections_b(bq7690x_t *dev, uint8_t *value);
bq_status_t bq7690x_write_enabled_protections_b(bq7690x_t *dev, uint8_t value);

/* ========== FET Protections ========== */

bq_status_t bq7690x_read_dsg_fet_protections_a(bq7690x_t *dev, uint8_t *value);
bq_status_t bq7690x_write_dsg_fet_protections_a(bq7690x_t *dev, uint8_t value);

bq_status_t bq7690x_read_chg_fet_protections_a(bq7690x_t *dev, uint8_t *value);
bq_status_t bq7690x_write_chg_fet_protections_a(bq7690x_t *dev, uint8_t value);

bq_status_t bq7690x_read_both_fet_protections_b(bq7690x_t *dev, uint8_t *value);
bq_status_t bq7690x_write_both_fet_protections_b(bq7690x_t *dev, uint8_t value);

/* ========== Protection Settings ========== */

bq_status_t bq7690x_read_body_diode_threshold(bq7690x_t *dev, int16_t *threshold_userA);
bq_status_t bq7690x_write_body_diode_threshold(bq7690x_t *dev, int16_t threshold_userA);

bq_status_t bq7690x_read_cell_open_wire_normal_time(bq7690x_t *dev, uint8_t *time_sec);
bq_status_t bq7690x_write_cell_open_wire_normal_time(bq7690x_t *dev, uint8_t time_sec);

bq_status_t bq7690x_read_cell_open_wire_sleep_time(bq7690x_t *dev, uint8_t *time_sec);
bq_status_t bq7690x_write_cell_open_wire_sleep_time(bq7690x_t *dev, uint8_t time_sec);

bq_status_t bq7690x_read_host_watchdog_timeout(bq7690x_t *dev, uint8_t *timeout_sec);
bq_status_t bq7690x_write_host_watchdog_timeout(bq7690x_t *dev, uint8_t timeout_sec);

/* ========== Cell Voltage Protection ========== */

bq_status_t bq7690x_read_cell_uv_threshold(bq7690x_t *dev, int16_t *threshold_userA);
bq_status_t bq7690x_write_cell_uv_threshold(bq7690x_t *dev, int16_t threshold_userA);

bq_status_t bq7690x_read_cell_uv_delay(bq7690x_t *dev, uint8_t *delay_adscan);
bq_status_t bq7690x_write_cell_uv_delay(bq7690x_t *dev, uint8_t delay_adscan);

bq_status_t bq7690x_read_cell_uv_recovery(bq7690x_t *dev, uint8_t *recovery_reg);
bq_status_t bq7690x_write_cell_uv_recovery(bq7690x_t *dev, uint8_t recovery_reg);

bq_status_t bq7690x_read_cell_ov_threshold(bq7690x_t *dev, int16_t *threshold_userA);
bq_status_t bq7690x_write_cell_ov_threshold(bq7690x_t *dev, int16_t threshold_userA);

bq_status_t bq7690x_read_cell_ov_delay(bq7690x_t *dev, uint8_t *delay_adscan);
bq_status_t bq7690x_write_cell_ov_delay(bq7690x_t *dev, uint8_t delay_adscan);

bq_status_t bq7690x_read_cell_ov_recovery(bq7690x_t *dev, uint8_t *recovery_reg);
bq_status_t bq7690x_write_cell_ov_recovery(bq7690x_t *dev, uint8_t recovery_reg);

/* ========== Current Protection ========== */

bq_status_t bq7690x_read_occ_threshold(bq7690x_t *dev, uint8_t *threshold);
bq_status_t bq7690x_write_occ_threshold(bq7690x_t *dev, uint8_t threshold);

bq_status_t bq7690x_read_occ_delay(bq7690x_t *dev, uint8_t *delay_ms);
bq_status_t bq7690x_write_occ_delay(bq7690x_t *dev, uint8_t delay_ms);

bq_status_t bq7690x_read_ocd1_threshold(bq7690x_t *dev, uint8_t *threshold);
bq_status_t bq7690x_write_ocd1_threshold(bq7690x_t *dev, uint8_t threshold);

bq_status_t bq7690x_read_ocd1_delay(bq7690x_t *dev, uint8_t *delay_ms);
bq_status_t bq7690x_write_ocd1_delay(bq7690x_t *dev, uint8_t delay_ms);

bq_status_t bq7690x_read_ocd2_threshold(bq7690x_t *dev, uint8_t *threshold);
bq_status_t bq7690x_write_ocd2_threshold(bq7690x_t *dev, uint8_t threshold);

bq_status_t bq7690x_read_ocd2_delay(bq7690x_t *dev, uint8_t *delay_ms);
bq_status_t bq7690x_write_ocd2_delay(bq7690x_t *dev, uint8_t delay_ms);

bq_status_t bq7690x_read_scd_threshold(bq7690x_t *dev, uint8_t *threshold);
bq_status_t bq7690x_write_scd_threshold(bq7690x_t *dev, uint8_t threshold);

bq_status_t bq7690x_read_scd_delay(bq7690x_t *dev, uint8_t *delay_us);
bq_status_t bq7690x_write_scd_delay(bq7690x_t *dev, uint8_t delay_us);

bq_status_t bq7690x_read_latch_limit(bq7690x_t *dev, uint8_t *limit);
bq_status_t bq7690x_write_latch_limit(bq7690x_t *dev, uint8_t limit);

bq_status_t bq7690x_read_recovery_time(bq7690x_t *dev, uint8_t *time_sec);
bq_status_t bq7690x_write_recovery_time(bq7690x_t *dev, uint8_t time_sec);

/* ========== Temperature Protection (Charge) ========== */

bq_status_t bq7690x_read_otc_threshold(bq7690x_t *dev, uint8_t *threshold_reg);
bq_status_t bq7690x_write_otc_threshold(bq7690x_t *dev, uint8_t threshold_reg);

bq_status_t bq7690x_read_otc_delay(bq7690x_t *dev, uint8_t *delay_meas);
bq_status_t bq7690x_write_otc_delay(bq7690x_t *dev, uint8_t delay_meas);

bq_status_t bq7690x_read_otc_recovery(bq7690x_t *dev, uint8_t *recovery_reg);
bq_status_t bq7690x_write_otc_recovery(bq7690x_t *dev, uint8_t recovery_reg);

bq_status_t bq7690x_read_utc_threshold(bq7690x_t *dev, uint8_t *threshold_reg);
bq_status_t bq7690x_write_utc_threshold(bq7690x_t *dev, uint8_t threshold_reg);

bq_status_t bq7690x_read_utc_delay(bq7690x_t *dev, uint8_t *delay_meas);
bq_status_t bq7690x_write_utc_delay(bq7690x_t *dev, uint8_t delay_meas);

bq_status_t bq7690x_read_utc_recovery(bq7690x_t *dev, uint8_t *recovery_reg);
bq_status_t bq7690x_write_utc_recovery(bq7690x_t *dev, uint8_t recovery_reg);

/* ========== Temperature Protection (Discharge) ========== */

bq_status_t bq7690x_read_otd_threshold(bq7690x_t *dev, uint8_t *threshold_reg);
bq_status_t bq7690x_write_otd_threshold(bq7690x_t *dev, uint8_t threshold_reg);

bq_status_t bq7690x_read_otd_delay(bq7690x_t *dev, uint8_t *delay_meas);
bq_status_t bq7690x_write_otd_delay(bq7690x_t *dev, uint8_t delay_meas);

bq_status_t bq7690x_read_otd_recovery(bq7690x_t *dev, uint8_t *recovery_reg);
bq_status_t bq7690x_write_otd_recovery(bq7690x_t *dev, uint8_t recovery_reg);

bq_status_t bq7690x_read_utd_threshold(bq7690x_t *dev, uint8_t *threshold_reg);
bq_status_t bq7690x_write_utd_threshold(bq7690x_t *dev, uint8_t threshold_reg);

bq_status_t bq7690x_read_utd_delay(bq7690x_t *dev, uint8_t *delay_meas);
bq_status_t bq7690x_write_utd_delay(bq7690x_t *dev, uint8_t delay_meas);

bq_status_t bq7690x_read_utd_recovery(bq7690x_t *dev, uint8_t *recovery_reg);
bq_status_t bq7690x_write_utd_recovery(bq7690x_t *dev, uint8_t recovery_reg);

/* ========== Internal Temperature Protection ========== */

bq_status_t bq7690x_read_otint_threshold(bq7690x_t *dev, uint8_t *temp_c);
bq_status_t bq7690x_write_otint_threshold(bq7690x_t *dev, uint8_t temp_c);

bq_status_t bq7690x_read_otint_delay(bq7690x_t *dev, uint8_t *delay_meas);
bq_status_t bq7690x_write_otint_delay(bq7690x_t *dev, uint8_t delay_meas);

bq_status_t bq7690x_read_otint_recovery(bq7690x_t *dev, uint8_t *recovery_reg);
bq_status_t bq7690x_write_otint_recovery(bq7690x_t *dev, uint8_t recovery_reg);
