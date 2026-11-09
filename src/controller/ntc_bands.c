/* ntc_bands.c -- temperature band ladders; see ntc_bands.h. */

#include "ntc_bands.h"

/*
 * Band index for a temperature against an ascending ladder of points.
 *
 * The band is the number of points the temperature has reached. Each point
 * already passed is pulled down by hyst_c, so leaving a band downward costs
 * that much more than entering it did. The pulled-down points stay ascending
 * as long as hyst_c is smaller than the gap between two points, which the
 * parameter bounds enforce.
 */
static uint8_t band_index(int16_t temp_c, const int16_t *points, uint8_t npoints,
                          uint8_t prev, int16_t hyst_c)
{
    uint8_t band = 0;

    for (uint8_t i = 0; i < npoints; i++) {
        int16_t edge = points[i];
        if (prev > i) {
            edge = (int16_t)(edge - hyst_c);
        }
        if (temp_c >= edge) {
            band = (uint8_t)(i + 1u);
        }
    }
    return band;
}

ntc_chg_band_t ntc_band_charge(int16_t temp_c, ntc_chg_band_t prev, const ntc_bands_t *b)
{
    int16_t points[4];

    if (b == 0) {
        return NTC_CHG_OFF_COLD;
    }
    points[0] = b->chg_min_c;
    points[1] = b->chg_prechg_end_c;
    points[2] = b->chg_normal_max_c;
    points[3] = b->chg_max_c;

    return (ntc_chg_band_t)band_index(temp_c, points, 4u, (uint8_t)prev, b->hyst_c);
}

ntc_dsg_band_t ntc_band_discharge(int16_t temp_c, ntc_dsg_band_t prev, const ntc_bands_t *b)
{
    int16_t points[3];

    if (b == 0) {
        return NTC_DSG_OFF_COLD;
    }
    points[0] = b->dsg_min_c;
    points[1] = b->dsg_normal_max_c;
    points[2] = b->dsg_max_c;

    return (ntc_dsg_band_t)band_index(temp_c, points, 3u, (uint8_t)prev, b->hyst_c);
}

ntc_fet_band_t ntc_band_mosfet(int16_t temp_c, ntc_fet_band_t prev, const ntc_bands_t *b)
{
    int16_t points[2];

    if (b == 0) {
        return NTC_FET_OFF_COLD;
    }
    points[0] = b->fet_min_c;
    points[1] = b->fet_max_c;

    return (ntc_fet_band_t)band_index(temp_c, points, 2u, (uint8_t)prev, b->hyst_c);
}

uint8_t ntc_band_blocks_charge(ntc_chg_band_t band)
{
    return (uint8_t)((band == NTC_CHG_OFF_COLD) ||
                     (band == NTC_CHG_PRECHARGE) ||
                     (band == NTC_CHG_OFF_HOT));
}

uint8_t ntc_band_blocks_discharge(ntc_dsg_band_t band)
{
    return (uint8_t)((band == NTC_DSG_OFF_COLD) || (band == NTC_DSG_OFF_HOT));
}

const char *ntc_band_charge_name(ntc_chg_band_t band)
{
    switch (band) {
        case NTC_CHG_OFF_COLD:  return "chg-off-cold";
        case NTC_CHG_PRECHARGE: return "pre-charge";
        case NTC_CHG_NORMAL:    return "chg-normal";
        case NTC_CHG_EXTENDED:  return "chg-extended";
        case NTC_CHG_OFF_HOT:   return "chg-off-hot";
        default:                return "chg-?";
    }
}

const char *ntc_band_discharge_name(ntc_dsg_band_t band)
{
    switch (band) {
        case NTC_DSG_OFF_COLD: return "dsg-off-cold";
        case NTC_DSG_NORMAL:   return "dsg-normal";
        case NTC_DSG_EXTENDED: return "dsg-extended";
        case NTC_DSG_OFF_HOT:  return "dsg-off-hot";
        default:               return "dsg-?";
    }
}

const char *ntc_band_mosfet_name(ntc_fet_band_t band)
{
    switch (band) {
        case NTC_FET_OFF_COLD: return "fet-off-cold";
        case NTC_FET_OK:       return "fet-ok";
        case NTC_FET_OFF_HOT:  return "fet-off-hot";
        default:               return "fet-?";
    }
}
