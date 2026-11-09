/*
 * ntc_fault.h -- what the BMS does about the MCU's own NTC readings.
 *
 * external_temp_adc.c measures the four ADC channels and classifies open
 * wires; this decides what that means for the power path. Two independent
 * duties, both behind PARAM_NTC_SAFETY_EN, which ships 0:
 *
 *   Temperature bands (ntc_bands.h). CH1-CH3 watch the cells and are judged
 *   against the charge and discharge ladders; CH4 watches the MOSFETs and has
 *   a single operating range. A band that forbids a FET raises a request on
 *   the FET-off arbiter, which suspends no protection and survives
 *   CONFIG_UPDATE. This board has no pre-charge FET, so the pre-charge band
 *   holds the charge FET open like the band below it; the point between them
 *   stays adjustable and the two are reported apart.
 *
 *   Open wire. An open channel is ignored and the remaining ones carry the
 *   judgement, because an open channel rails the ADC and so cannot present a
 *   plausible temperature. CH4 falls back to the BQ76907's own TS1 sensor.
 *   When no battery channel is left, there is nothing to judge temperature
 *   with and both FETs are held open.
 *
 * With the parameter at 0 every reading is still converted, classified,
 * printed and broadcast; no FET is touched.
 */

#ifndef NTC_FAULT_H
#define NTC_FAULT_H

#include <stdint.h>
#include <stdbool.h>
#include "ntc_bands.h"

typedef enum {
    NTC_STATE_OK = 0,   /* every channel present, every band permissive      */
    NTC_STATE_DEGRADED, /* a channel is open or on a fallback; FETs untouched */
    NTC_STATE_TRIPPED,  /* a band forbids a FET and that FET is held open    */
    NTC_STATE_SERVICE   /* no battery channel left: both FETs held open      */
} ntc_state_t;

typedef struct {
    bool           enabled;             /* PARAM_NTC_SAFETY_EN                */
    ntc_state_t    state;
    uint8_t        usable_battery_mask; /* CH1-3 present and not open-wire    */
    bool           ch4_open;            /* CH4 open, MOSFET temp from TS1     */
    bool           fet_temp_valid;      /* a MOSFET temperature source exists */
    int16_t        fet_temp_c;
    int16_t        battery_min_c;       /* over usable battery channels       */
    int16_t        battery_max_c;
    ntc_chg_band_t chg_band;            /* most restrictive over those channels */
    ntc_dsg_band_t dsg_band;
    ntc_fet_band_t fet_band;
    uint8_t        want_fets_off;       /* BQ_FET_* bits the bands ask for    */
    uint8_t        applied_fets_off;    /* bits actually requested, after the delay */
    uint32_t       entered_ms;          /* when the current state began       */
} ntc_fault_status_t;

void ntc_fault_init(void);

/*
 * Drive the response from the latest ADC sweep. Call once per controller tick,
 * immediately after ext_temp_adc_update(), so the bands see this tick's
 * readings and the open-wire masks they produced.
 */
void ntc_fault_tick(void);

const ntc_fault_status_t *ntc_fault_status(void);

/** @brief Print the band ladders, the per-channel bands, and the FET request. */
void ntc_fault_print_status(void);

#endif /* NTC_FAULT_H */
