/* ntc_fault.c -- response to the MCU's NTC readings; see ntc_fault.h. */

#include "ntc_fault.h"
#include "external_temp_adc.h"
#include "bq_controller.h"
#include "bq7690x_fet.h"
#include "bq_config_4s_lfp.h"
#include "parameter.h"
#include "hal_systime.h"
#include "hal_uart.h"

static ntc_fault_status_t s_st;

/* Per-channel band history, so each ladder's hysteresis has something to work
 * from. A channel that goes unusable is reset to its permissive band, so it
 * rejoins without carrying a stale edge. */
static ntc_chg_band_t s_chg_prev[EXT_TEMP_SENSOR_COUNT];
static ntc_dsg_band_t s_dsg_prev[EXT_TEMP_SENSOR_COUNT];
static ntc_fet_band_t s_fet_prev;

/* Trip delay: the request must stand this long before it reaches the chip. */
static uint8_t  s_pending_fets_off;
static uint32_t s_pending_since_ms;

static void bands_from_params(ntc_bands_t *b)
{
    b->chg_min_c        = param_cache[PARAM_NTC_CHG_MIN_C].i16;
    b->chg_prechg_end_c = param_cache[PARAM_NTC_CHG_PRECHG_END_C].i16;
    b->chg_normal_max_c = param_cache[PARAM_NTC_CHG_NORMAL_MAX_C].i16;
    b->chg_max_c        = param_cache[PARAM_NTC_CHG_MAX_C].i16;
    b->dsg_min_c        = param_cache[PARAM_NTC_DSG_MIN_C].i16;
    b->dsg_normal_max_c = param_cache[PARAM_NTC_DSG_NORMAL_MAX_C].i16;
    b->dsg_max_c        = param_cache[PARAM_NTC_DSG_MAX_C].i16;
    b->fet_min_c        = param_cache[PARAM_NTC_FET_MIN_C].i16;
    b->fet_max_c        = param_cache[PARAM_NTC_FET_MAX_C].i16;
    b->hyst_c           = (int16_t)param_cache[PARAM_NTC_HYST_C].u8;
}

/* How restrictive a charge band is, so the worst channel decides. A band that
 * blocks charging outranks the pre-charge band, which outranks the two that
 * permit charging. */
static uint8_t chg_severity(ntc_chg_band_t b)
{
    if ((b == NTC_CHG_OFF_COLD) || (b == NTC_CHG_OFF_HOT)) { return 2u; }
    if (b == NTC_CHG_PRECHARGE)                            { return 1u; }
    return 0u;
}

static uint8_t dsg_severity(ntc_dsg_band_t b)
{
    return ntc_band_blocks_discharge(b) ? 1u : 0u;
}

static void set_state(ntc_state_t next)
{
    if (s_st.state != next) {
        s_st.state = next;
        s_st.entered_ms = systime_milli();
    }
}

void ntc_fault_init(void)
{
    for (int i = 0; i < EXT_TEMP_SENSOR_COUNT; i++) {
        s_chg_prev[i] = NTC_CHG_NORMAL;
        s_dsg_prev[i] = NTC_DSG_NORMAL;
    }
    s_fet_prev = NTC_FET_OK;

    s_st.enabled             = false;
    s_st.state               = NTC_STATE_OK;
    s_st.usable_battery_mask = 0;
    s_st.ch4_open            = false;
    s_st.fet_temp_valid      = false;
    s_st.fet_temp_c          = EXT_TEMP_INVALID_C;
    s_st.battery_min_c       = EXT_TEMP_INVALID_C;
    s_st.battery_max_c       = EXT_TEMP_INVALID_C;
    s_st.chg_band            = NTC_CHG_NORMAL;
    s_st.dsg_band            = NTC_DSG_NORMAL;
    s_st.fet_band            = NTC_FET_OK;
    s_st.want_fets_off       = 0;
    s_st.applied_fets_off    = 0;
    s_st.entered_ms          = systime_milli();

    s_pending_fets_off = 0;
    s_pending_since_ms = systime_milli();
}

void ntc_fault_tick(void)
{
    const ext_temp_module_status_t *ext = ext_temp_adc_get_status();
    const bq_battery_status_t *bq = bq_controller_get_status();
    ntc_bands_t bands;
    uint32_t now = systime_milli();
    uint8_t usable_batt;
    uint8_t worst_chg_sev = 0;
    uint8_t worst_dsg_sev = 0;
    uint8_t want = 0;
    uint8_t fault_mask = 0;

    if (ext == NULL) {
        return;
    }

    s_st.enabled = (param_cache[PARAM_NTC_SAFETY_EN].u8 != 0u);
    bands_from_params(&bands);

    usable_batt = (uint8_t)(ext->usable_mask & EXT_TEMP_BATTERY_MASK);
    s_st.usable_battery_mask = usable_batt;
    s_st.battery_min_c = EXT_TEMP_INVALID_C;
    s_st.battery_max_c = EXT_TEMP_INVALID_C;
    s_st.chg_band = NTC_CHG_NORMAL;
    s_st.dsg_band = NTC_DSG_NORMAL;

    /* Battery channels: an open or out-of-range channel contributes nothing,
     * and its history is reset so it rejoins on a clean edge. */
    for (int i = 0; i < EXT_TEMP_SENSOR_COUNT; i++) {
        uint8_t bit = (uint8_t)(1u << i);
        int16_t t;
        ntc_chg_band_t cb;
        ntc_dsg_band_t db;

        if ((bit & EXT_TEMP_BATTERY_MASK) == 0u) {
            continue;
        }
        if ((usable_batt & bit) == 0u) {
            s_chg_prev[i] = NTC_CHG_NORMAL;
            s_dsg_prev[i] = NTC_DSG_NORMAL;
            continue;
        }

        t = ext->sensors[i].temp_c;
        if ((s_st.battery_min_c == EXT_TEMP_INVALID_C) || (t < s_st.battery_min_c)) {
            s_st.battery_min_c = t;
        }
        if ((s_st.battery_max_c == EXT_TEMP_INVALID_C) || (t > s_st.battery_max_c)) {
            s_st.battery_max_c = t;
        }

        cb = ntc_band_charge(t, s_chg_prev[i], &bands);
        db = ntc_band_discharge(t, s_dsg_prev[i], &bands);
        s_chg_prev[i] = cb;
        s_dsg_prev[i] = db;

        if (chg_severity(cb) > worst_chg_sev) {
            worst_chg_sev = chg_severity(cb);
            s_st.chg_band = cb;
        }
        if (dsg_severity(db) > worst_dsg_sev) {
            worst_dsg_sev = dsg_severity(db);
            s_st.dsg_band = db;
        }
        if (ntc_band_blocks_charge(cb) || ntc_band_blocks_discharge(db)) {
            fault_mask |= bit;
        }
    }

    /* MOSFET channel: CH4, or the BQ's own TS1 sensor when CH4 is open. With
     * neither source the band is left permissive -- the MOSFET ladder has no
     * force-off for a missing sensor, only for a temperature outside it. */
    s_st.ch4_open = ((ext->usable_mask & EXT_TEMP_MOSFET_MASK) == 0u);
    if (!s_st.ch4_open) {
        s_st.fet_temp_c = ext->sensors[EXT_TEMP_SENSOR_4].temp_c;
        s_st.fet_temp_valid = true;
    } else if (!bq->ts1_open_wire && (bq->ts1_temp_c != EXT_TEMP_INVALID_C)) {
        s_st.fet_temp_c = bq->ts1_temp_c;
        s_st.fet_temp_valid = true;
    } else {
        s_st.fet_temp_c = EXT_TEMP_INVALID_C;
        s_st.fet_temp_valid = false;
    }

    if (s_st.fet_temp_valid) {
        s_fet_prev = ntc_band_mosfet(s_st.fet_temp_c, s_fet_prev, &bands);
    } else {
        s_fet_prev = NTC_FET_OK;
    }
    s_st.fet_band = s_fet_prev;
    if (s_st.fet_band != NTC_FET_OK) {
        fault_mask |= EXT_TEMP_MOSFET_MASK;
    }

    /* What the bands ask for. */
    if (usable_batt == 0u) {
        /* Nothing left to judge cell temperature with. */
        want = (uint8_t)(BQ_FET_DSG_OFF | BQ_FET_CHG_OFF);
    } else {
        if (ntc_band_blocks_charge(s_st.chg_band))    { want |= BQ_FET_CHG_OFF; }
        if (ntc_band_blocks_discharge(s_st.dsg_band)) { want |= BQ_FET_DSG_OFF; }
        if (s_st.fet_band != NTC_FET_OK) {
            want |= (uint8_t)(BQ_FET_DSG_OFF | BQ_FET_CHG_OFF);
        }
    }
    s_st.want_fets_off = want;

    /* Trip delay: a change has to stand before it reaches the chip. Applied in
     * both directions; the band hysteresis already stops edge chatter, and this
     * keeps a single noisy sweep from moving the power path. */
    if (want != s_pending_fets_off) {
        s_pending_fets_off = want;
        s_pending_since_ms = now;
    }
    if ((want != s_st.applied_fets_off) &&
        ((now - s_pending_since_ms) >= (uint32_t)CFG_NTC_TRIP_DELAY_MS)) {
        if (s_st.enabled) {
            hal_uart_printf("[NTC] FET request 0x%02X -> 0x%02X (%s / %s / %s, usable 0x%02X)\r\n",
                              s_st.applied_fets_off, want,
                              ntc_band_charge_name(s_st.chg_band),
                              ntc_band_discharge_name(s_st.dsg_band),
                              ntc_band_mosfet_name(s_st.fet_band),
                              usable_batt);
        }
        s_st.applied_fets_off = want;
    }

    /* Act, or stay out of the way. Withdrawing on a disable is deliberate: the
     * parameter is the one switch that takes this layer off the power path. */
    if (s_st.enabled) {
        (void)bq_controller_request_fets_off(FET_OFF_OWNER_NTC, s_st.applied_fets_off);
    } else {
        (void)bq_controller_request_fets_off(FET_OFF_OWNER_NTC, 0);
    }

    if (usable_batt == 0u) {
        set_state(NTC_STATE_SERVICE);
    } else if (s_st.applied_fets_off != 0u) {
        set_state(NTC_STATE_TRIPPED);
    } else if ((usable_batt != EXT_TEMP_BATTERY_MASK) || s_st.ch4_open ||
               !s_st.fet_temp_valid) {
        set_state(NTC_STATE_DEGRADED);
    } else {
        set_state(NTC_STATE_OK);
    }

    ext_temp_adc_set_fault_state(fault_mask,
                                 s_st.enabled && (s_st.applied_fets_off != 0u));
}

const ntc_fault_status_t *ntc_fault_status(void)
{
    return &s_st;
}

void ntc_fault_print_status(void)
{
    static const char *const state_name[] = { "OK", "DEGRADED", "TRIPPED", "SERVICE" };
    ntc_bands_t b;

    bands_from_params(&b);

    hal_uart_printf("\r\n=== NTC Safety ===\r\n");
    hal_uart_printf("Enabled: %s, state: %s\r\n",
                      s_st.enabled ? "YES" : "NO",
                      state_name[(int)s_st.state]);
    hal_uart_printf("Charge points: %dC %dC %dC %dC, discharge: %dC %dC %dC, "
                      "MOSFET: %dC..%dC, hysteresis %dC\r\n",
                      b.chg_min_c, b.chg_prechg_end_c, b.chg_normal_max_c, b.chg_max_c,
                      b.dsg_min_c, b.dsg_normal_max_c, b.dsg_max_c,
                      b.fet_min_c, b.fet_max_c, b.hyst_c);
    hal_uart_printf("Battery channels usable 0x%02X, %dC..%dC -> %s / %s\r\n",
                      s_st.usable_battery_mask, s_st.battery_min_c, s_st.battery_max_c,
                      ntc_band_charge_name(s_st.chg_band),
                      ntc_band_discharge_name(s_st.dsg_band));
    hal_uart_printf("MOSFET temp %dC (%s) -> %s\r\n",
                      s_st.fet_temp_c,
                      !s_st.fet_temp_valid ? "no source" :
                        (s_st.ch4_open ? "BQ TS1 fallback" : "NTC_CH4"),
                      ntc_band_mosfet_name(s_st.fet_band));
    hal_uart_printf("FET off request 0x%02X (want 0x%02X)\r\n",
                      s_st.applied_fets_off, s_st.want_fets_off);
    hal_uart_printf("==================\r\n\r\n");
}
