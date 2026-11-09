/*
 * ntc_bands.h -- temperature bands for the MCU-side NTC protection.
 *
 * Charge, discharge and MOSFET temperature each have their own ladder of
 * adjustable points; a temperature lands in exactly one band of each ladder and
 * the band says what the FETs may do. The points come from parameters, not from
 * the BQ76907's own temperature thresholds, so the two layers are independent.
 *
 * Pure arithmetic: no hardware, no parameter access, no I/O, so the host test
 * harness compiles this file on its own.
 */

#ifndef NTC_BANDS_H
#define NTC_BANDS_H

#include <stdint.h>

/* Charge ladder: four points, five bands. */
typedef enum {
    NTC_CHG_OFF_COLD = 0,   /* below chg_min_c                                */
    NTC_CHG_PRECHARGE,      /* chg_min_c .. prechg_end_c: CHG off, no PRECHG FET */
    NTC_CHG_NORMAL,         /* prechg_end_c .. normal_max_c                   */
    NTC_CHG_EXTENDED,       /* normal_max_c .. chg_max_c                      */
    NTC_CHG_OFF_HOT         /* at or above chg_max_c                          */
} ntc_chg_band_t;

/* Discharge ladder: three points, four bands. */
typedef enum {
    NTC_DSG_OFF_COLD = 0,   /* below dsg_min_c                                */
    NTC_DSG_NORMAL,         /* dsg_min_c .. normal_max_c                      */
    NTC_DSG_EXTENDED,       /* normal_max_c .. dsg_max_c                      */
    NTC_DSG_OFF_HOT         /* at or above dsg_max_c                          */
} ntc_dsg_band_t;

/* MOSFET ladder: two points, three bands. */
typedef enum {
    NTC_FET_OFF_COLD = 0,   /* below fet_min_c                                */
    NTC_FET_OK,             /* fet_min_c .. fet_max_c                         */
    NTC_FET_OFF_HOT         /* at or above fet_max_c                          */
} ntc_fet_band_t;

typedef struct {
    /* Charge, ascending. */
    int16_t chg_min_c;          /* CHG off below this                         */
    int16_t chg_prechg_end_c;   /* pre-charge band ends, normal charge begins */
    int16_t chg_normal_max_c;   /* normal charge ends, extended begins        */
    int16_t chg_max_c;          /* CHG off at or above this                   */

    /* Discharge, ascending. */
    int16_t dsg_min_c;          /* DSG off below this                         */
    int16_t dsg_normal_max_c;   /* normal discharge ends, extended begins     */
    int16_t dsg_max_c;          /* DSG off at or above this                   */

    /* MOSFET channel. */
    int16_t fet_min_c;
    int16_t fet_max_c;

    /* A band already entered is held until the temperature falls this far back
     * through the point that was crossed. 0 disables hysteresis. */
    int16_t hyst_c;
} ntc_bands_t;

/*
 * Band for a temperature, given the band it was in last time.
 *
 * A point is crossed upward at the point itself and downward only hyst_c below
 * it, so a reading sitting on a point cannot oscillate. `prev` carries that
 * state; pass the band these returned last tick for the same channel.
 */
ntc_chg_band_t ntc_band_charge(int16_t temp_c, ntc_chg_band_t prev, const ntc_bands_t *b);
ntc_dsg_band_t ntc_band_discharge(int16_t temp_c, ntc_dsg_band_t prev, const ntc_bands_t *b);
ntc_fet_band_t ntc_band_mosfet(int16_t temp_c, ntc_fet_band_t prev, const ntc_bands_t *b);

/* true when the band forbids the corresponding FET. The pre-charge band
 * forbids charging too: this board has no pre-charge FET to hand the current
 * to, so the band differs from the one below it only in what it reports. */
uint8_t ntc_band_blocks_charge(ntc_chg_band_t band);
uint8_t ntc_band_blocks_discharge(ntc_dsg_band_t band);

/* Names for logs and telemetry. */
const char *ntc_band_charge_name(ntc_chg_band_t band);
const char *ntc_band_discharge_name(ntc_dsg_band_t band);
const char *ntc_band_mosfet_name(ntc_fet_band_t band);

#endif /* NTC_BANDS_H */
