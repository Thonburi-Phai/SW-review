/**
 * @file bq7690x_protection.c
 * @brief BQ7690x Protection Configuration Implementation
 */

#include "bq7690x_protection.h"

/* ========== Enabled Protections ========== */

bq_status_t bq7690x_read_enabled_protections_a(bq7690x_t *dev, uint8_t *value)
{
    return bq_dm_read_8_cfg(dev, EnabledProtectionsA, value);
}

bq_status_t bq7690x_write_enabled_protections_a(bq7690x_t *dev, uint8_t value)
{
    return bq_dm_write_8_cfg(dev, EnabledProtectionsA, value);
}

bq_status_t bq7690x_read_enabled_protections_b(bq7690x_t *dev, uint8_t *value)
{
    return bq_dm_read_8_cfg(dev, EnabledProtectionsB, value);
}

bq_status_t bq7690x_write_enabled_protections_b(bq7690x_t *dev, uint8_t value)
{
    return bq_dm_write_8_cfg(dev, EnabledProtectionsB, value);
}

/* ========== FET Protections ========== */

bq_status_t bq7690x_read_dsg_fet_protections_a(bq7690x_t *dev, uint8_t *value)
{
    return bq_dm_read_8_cfg(dev, DSGFETProtectionsA, value);
}

bq_status_t bq7690x_write_dsg_fet_protections_a(bq7690x_t *dev, uint8_t value)
{
    return bq_dm_write_8_cfg(dev, DSGFETProtectionsA, value);
}

bq_status_t bq7690x_read_chg_fet_protections_a(bq7690x_t *dev, uint8_t *value)
{
    return bq_dm_read_8_cfg(dev, CHGFETProtectionsA, value);
}

bq_status_t bq7690x_write_chg_fet_protections_a(bq7690x_t *dev, uint8_t value)
{
    return bq_dm_write_8_cfg(dev, CHGFETProtectionsA, value);
}

bq_status_t bq7690x_read_both_fet_protections_b(bq7690x_t *dev, uint8_t *value)
{
    return bq_dm_read_8_cfg(dev, BothFETProtectionsB, value);
}

bq_status_t bq7690x_write_both_fet_protections_b(bq7690x_t *dev, uint8_t value)
{
    return bq_dm_write_8_cfg(dev, BothFETProtectionsB, value);
}

/* ========== Protection Settings ========== */

bq_status_t bq7690x_read_body_diode_threshold(bq7690x_t *dev, int16_t *value)
{
    return bq_dm_read_16_cfg(dev, BodyDiodeThreshold, (uint16_t *)value);
}

bq_status_t bq7690x_write_body_diode_threshold(bq7690x_t *dev, int16_t value)
{
    return bq_dm_write_16_cfg(dev, BodyDiodeThreshold, (uint16_t)value);
}

bq_status_t bq7690x_read_cell_open_wire_normal_time(bq7690x_t *dev, uint8_t *value)
{
    return bq_dm_read_8_cfg(dev, CellOpenWireNORMALCheckTime, value);
}

bq_status_t bq7690x_write_cell_open_wire_normal_time(bq7690x_t *dev, uint8_t value)
{
    return bq_dm_write_8_cfg(dev, CellOpenWireNORMALCheckTime, value);
}

bq_status_t bq7690x_read_cell_open_wire_sleep_time(bq7690x_t *dev, uint8_t *value)
{
    return bq_dm_read_8_cfg(dev, CellOpenWireSLEEPCheckTime, value);
}

bq_status_t bq7690x_write_cell_open_wire_sleep_time(bq7690x_t *dev, uint8_t value)
{
    return bq_dm_write_8_cfg(dev, CellOpenWireSLEEPCheckTime, value);
}

bq_status_t bq7690x_read_host_watchdog_timeout(bq7690x_t *dev, uint8_t *value)
{
    return bq_dm_read_8_cfg(dev, HostWatchdogTimeout, value);
}

bq_status_t bq7690x_write_host_watchdog_timeout(bq7690x_t *dev, uint8_t value)
{
    return bq_dm_write_8_cfg(dev, HostWatchdogTimeout, value);
}

/* ========== Cell Voltage Protection ========== */

bq_status_t bq7690x_read_cell_uv_threshold(bq7690x_t *dev, int16_t *value)
{
    return bq_dm_read_16_cfg(dev, CellUndervoltageProtectionThreshold, (uint16_t *)value);
}

bq_status_t bq7690x_write_cell_uv_threshold(bq7690x_t *dev, int16_t value)
{
    return bq_dm_write_16_cfg(dev, CellUndervoltageProtectionThreshold, (uint16_t)value);
}

bq_status_t bq7690x_read_cell_uv_delay(bq7690x_t *dev, uint8_t *value)
{
    return bq_dm_read_8_cfg(dev, CellUndervoltageProtectionDelay, value);
}

bq_status_t bq7690x_write_cell_uv_delay(bq7690x_t *dev, uint8_t value)
{
    return bq_dm_write_8_cfg(dev, CellUndervoltageProtectionDelay, value);
}

bq_status_t bq7690x_read_cell_uv_recovery(bq7690x_t *dev, uint8_t *value)
{
    return bq_dm_read_8_cfg(dev, CellUndervoltageProtectionRecoveryHysteresis, value);
}

bq_status_t bq7690x_write_cell_uv_recovery(bq7690x_t *dev, uint8_t value)
{
    return bq_dm_write_8_cfg(dev, CellUndervoltageProtectionRecoveryHysteresis, value);
}

bq_status_t bq7690x_read_cell_ov_threshold(bq7690x_t *dev, int16_t *value)
{
    return bq_dm_read_16_cfg(dev, CellOvervoltageProtectionThreshold, (uint16_t *)value);
}

bq_status_t bq7690x_write_cell_ov_threshold(bq7690x_t *dev, int16_t value)
{
    return bq_dm_write_16_cfg(dev, CellOvervoltageProtectionThreshold, (uint16_t)value);
}

bq_status_t bq7690x_read_cell_ov_delay(bq7690x_t *dev, uint8_t *value)
{
    return bq_dm_read_8_cfg(dev, CellOvervoltageProtectionDelay, value);
}

bq_status_t bq7690x_write_cell_ov_delay(bq7690x_t *dev, uint8_t value)
{
    return bq_dm_write_8_cfg(dev, CellOvervoltageProtectionDelay, value);
}

bq_status_t bq7690x_read_cell_ov_recovery(bq7690x_t *dev, uint8_t *value)
{
    return bq_dm_read_8_cfg(dev, CellOvervoltageProtectionRecoveryHysteresis, value);
}

bq_status_t bq7690x_write_cell_ov_recovery(bq7690x_t *dev, uint8_t value)
{
    return bq_dm_write_8_cfg(dev, CellOvervoltageProtectionRecoveryHysteresis, value);
}

/* ========== Current Protection ========== */

bq_status_t bq7690x_read_occ_threshold(bq7690x_t *dev, uint8_t *value)
{
    return bq_dm_read_8_cfg(dev, OvercurrentinChargeProtectionThreshold, value);
}

bq_status_t bq7690x_write_occ_threshold(bq7690x_t *dev, uint8_t value)
{
    return bq_dm_write_8_cfg(dev, OvercurrentinChargeProtectionThreshold, value);
}

bq_status_t bq7690x_read_occ_delay(bq7690x_t *dev, uint8_t *value)
{
    return bq_dm_read_8_cfg(dev, OvercurrentinChargeProtectionDelay, value);
}

bq_status_t bq7690x_write_occ_delay(bq7690x_t *dev, uint8_t value)
{
    return bq_dm_write_8_cfg(dev, OvercurrentinChargeProtectionDelay, value);
}

bq_status_t bq7690x_read_ocd1_threshold(bq7690x_t *dev, uint8_t *value)
{
    return bq_dm_read_8_cfg(dev, OvercurrentinDischarge1ProtectionThreshold, value);
}

bq_status_t bq7690x_write_ocd1_threshold(bq7690x_t *dev, uint8_t value)
{
    return bq_dm_write_8_cfg(dev, OvercurrentinDischarge1ProtectionThreshold, value);
}

bq_status_t bq7690x_read_ocd1_delay(bq7690x_t *dev, uint8_t *value)
{
    return bq_dm_read_8_cfg(dev, OvercurrentinDischarge1ProtectionDelay, value);
}

bq_status_t bq7690x_write_ocd1_delay(bq7690x_t *dev, uint8_t value)
{
    return bq_dm_write_8_cfg(dev, OvercurrentinDischarge1ProtectionDelay, value);
}

bq_status_t bq7690x_read_ocd2_threshold(bq7690x_t *dev, uint8_t *value)
{
    return bq_dm_read_8_cfg(dev, OvercurrentinDischarge2ProtectionThreshold, value);
}

bq_status_t bq7690x_write_ocd2_threshold(bq7690x_t *dev, uint8_t value)
{
    return bq_dm_write_8_cfg(dev, OvercurrentinDischarge2ProtectionThreshold, value);
}

bq_status_t bq7690x_read_ocd2_delay(bq7690x_t *dev, uint8_t *value)
{
    return bq_dm_read_8_cfg(dev, OvercurrentinDischarge2ProtectionDelay, value);
}

bq_status_t bq7690x_write_ocd2_delay(bq7690x_t *dev, uint8_t value)
{
    return bq_dm_write_8_cfg(dev, OvercurrentinDischarge2ProtectionDelay, value);
}

bq_status_t bq7690x_read_scd_threshold(bq7690x_t *dev, uint8_t *value)
{
    return bq_dm_read_8_cfg(dev, ShortCircuitinDischargeProtectionThreshold, value);
}

bq_status_t bq7690x_write_scd_threshold(bq7690x_t *dev, uint8_t value)
{
    return bq_dm_write_8_cfg(dev, ShortCircuitinDischargeProtectionThreshold, value);
}

bq_status_t bq7690x_read_scd_delay(bq7690x_t *dev, uint8_t *value)
{
    return bq_dm_read_8_cfg(dev, ShortCircuitinDischargeProtectionDelay, value);
}

bq_status_t bq7690x_write_scd_delay(bq7690x_t *dev, uint8_t value)
{
    return bq_dm_write_8_cfg(dev, ShortCircuitinDischargeProtectionDelay, value);
}

bq_status_t bq7690x_read_latch_limit(bq7690x_t *dev, uint8_t *value)
{
    return bq_dm_read_8_cfg(dev, LatchLimit, value);
}

bq_status_t bq7690x_write_latch_limit(bq7690x_t *dev, uint8_t value)
{
    return bq_dm_write_8_cfg(dev, LatchLimit, value);
}

bq_status_t bq7690x_read_recovery_time(bq7690x_t *dev, uint8_t *value)
{
    return bq_dm_read_8_cfg(dev, RecoveryTime, value);
}

bq_status_t bq7690x_write_recovery_time(bq7690x_t *dev, uint8_t value)
{
    return bq_dm_write_8_cfg(dev, RecoveryTime, value);
}

/* ========== Temperature Protection (Charge) ========== */

bq_status_t bq7690x_read_otc_threshold(bq7690x_t *dev, uint8_t *value)
{
    return bq_dm_read_8_cfg(dev, OvertemperatureinChargeProtectionThreshold, value);
}

bq_status_t bq7690x_write_otc_threshold(bq7690x_t *dev, uint8_t value)
{
    return bq_dm_write_8_cfg(dev, OvertemperatureinChargeProtectionThreshold, value);
}

bq_status_t bq7690x_read_otc_delay(bq7690x_t *dev, uint8_t *value)
{
    return bq_dm_read_8_cfg(dev, OvertemperatureinChargeProtectionDelay, value);
}

bq_status_t bq7690x_write_otc_delay(bq7690x_t *dev, uint8_t value)
{
    return bq_dm_write_8_cfg(dev, OvertemperatureinChargeProtectionDelay, value);
}

bq_status_t bq7690x_read_otc_recovery(bq7690x_t *dev, uint8_t *value)
{
    return bq_dm_read_8_cfg(dev, OvertemperatureinChargeProtectionRecovery, value);
}

bq_status_t bq7690x_write_otc_recovery(bq7690x_t *dev, uint8_t value)
{
    return bq_dm_write_8_cfg(dev, OvertemperatureinChargeProtectionRecovery, value);
}

bq_status_t bq7690x_read_utc_threshold(bq7690x_t *dev, uint8_t *value)
{
    return bq_dm_read_8_cfg(dev, UndertemperatureinChargeProtectionThreshold, value);
}

bq_status_t bq7690x_write_utc_threshold(bq7690x_t *dev, uint8_t value)
{
    return bq_dm_write_8_cfg(dev, UndertemperatureinChargeProtectionThreshold, value);
}

bq_status_t bq7690x_read_utc_delay(bq7690x_t *dev, uint8_t *value)
{
    return bq_dm_read_8_cfg(dev, UndertemperatureinChargeProtectionDelay, value);
}

bq_status_t bq7690x_write_utc_delay(bq7690x_t *dev, uint8_t value)
{
    return bq_dm_write_8_cfg(dev, UndertemperatureinChargeProtectionDelay, value);
}

bq_status_t bq7690x_read_utc_recovery(bq7690x_t *dev, uint8_t *value)
{
    return bq_dm_read_8_cfg(dev, UndertemperatureinChargeProtectionRecovery, value);
}

bq_status_t bq7690x_write_utc_recovery(bq7690x_t *dev, uint8_t value)
{
    return bq_dm_write_8_cfg(dev, UndertemperatureinChargeProtectionRecovery, value);
}

/* ========== Temperature Protection (Discharge) ========== */

bq_status_t bq7690x_read_otd_threshold(bq7690x_t *dev, uint8_t *value)
{
    return bq_dm_read_8_cfg(dev, OvertemperatureinDischargeProtectionThreshold, value);
}

bq_status_t bq7690x_write_otd_threshold(bq7690x_t *dev, uint8_t value)
{
    return bq_dm_write_8_cfg(dev, OvertemperatureinDischargeProtectionThreshold, value);
}

bq_status_t bq7690x_read_otd_delay(bq7690x_t *dev, uint8_t *value)
{
    return bq_dm_read_8_cfg(dev, OvertemperatureinDischargeProtectionDelay, value);
}

bq_status_t bq7690x_write_otd_delay(bq7690x_t *dev, uint8_t value)
{
    return bq_dm_write_8_cfg(dev, OvertemperatureinDischargeProtectionDelay, value);
}

bq_status_t bq7690x_read_otd_recovery(bq7690x_t *dev, uint8_t *value)
{
    return bq_dm_read_8_cfg(dev, OvertemperatureinDischargeProtectionRecovery, value);
}

bq_status_t bq7690x_write_otd_recovery(bq7690x_t *dev, uint8_t value)
{
    return bq_dm_write_8_cfg(dev, OvertemperatureinDischargeProtectionRecovery, value);
}

bq_status_t bq7690x_read_utd_threshold(bq7690x_t *dev, uint8_t *value)
{
    return bq_dm_read_8_cfg(dev, UndertemperatureinDischargeProtectionThreshold, value);
}

bq_status_t bq7690x_write_utd_threshold(bq7690x_t *dev, uint8_t value)
{
    return bq_dm_write_8_cfg(dev, UndertemperatureinDischargeProtectionThreshold, value);
}

bq_status_t bq7690x_read_utd_delay(bq7690x_t *dev, uint8_t *value)
{
    return bq_dm_read_8_cfg(dev, UndertemperatureinDischargeProtectionDelay, value);
}

bq_status_t bq7690x_write_utd_delay(bq7690x_t *dev, uint8_t value)
{
    return bq_dm_write_8_cfg(dev, UndertemperatureinDischargeProtectionDelay, value);
}

bq_status_t bq7690x_read_utd_recovery(bq7690x_t *dev, uint8_t *value)
{
    return bq_dm_read_8_cfg(dev, UndertemperatureinDischargeProtectionRecovery, value);
}

bq_status_t bq7690x_write_utd_recovery(bq7690x_t *dev, uint8_t value)
{
    return bq_dm_write_8_cfg(dev, UndertemperatureinDischargeProtectionRecovery, value);
}

/* ========== Internal Temperature Protection ========== */

bq_status_t bq7690x_read_otint_threshold(bq7690x_t *dev, uint8_t *value)
{
    return bq_dm_read_8_cfg(dev, InternalOvertemperatureProtectionThreshold, value);
}

bq_status_t bq7690x_write_otint_threshold(bq7690x_t *dev, uint8_t value)
{
    return bq_dm_write_8_cfg(dev, InternalOvertemperatureProtectionThreshold, value);
}

bq_status_t bq7690x_read_otint_delay(bq7690x_t *dev, uint8_t *value)
{
    return bq_dm_read_8_cfg(dev, InternalOvertemperatureProtectionDelay, value);
}

bq_status_t bq7690x_write_otint_delay(bq7690x_t *dev, uint8_t value)
{
    return bq_dm_write_8_cfg(dev, InternalOvertemperatureProtectionDelay, value);
}

bq_status_t bq7690x_read_otint_recovery(bq7690x_t *dev, uint8_t *value)
{
    return bq_dm_read_8_cfg(dev, InternalOvertemperatureProtectionRecovery, value);
}

bq_status_t bq7690x_write_otint_recovery(bq7690x_t *dev, uint8_t value)
{
    return bq_dm_write_8_cfg(dev, InternalOvertemperatureProtectionRecovery, value);
}
