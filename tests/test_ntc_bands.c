/*
 * test_ntc_bands.c -- host test for the NTC temperature band ladders.
 *
 * The points below are the shipped defaults from bq_config_4s_lfp.h, written
 * out so a change to either file shows up here as a failure rather than
 * silently agreeing with itself.
 *
 * Build and run:  make -C .. test
 */

#include "../src/controller/ntc_bands.h"
#include <stdio.h>

static int failures;
static int checks;

static const ntc_bands_t B = {
    .chg_min_c = -10, .chg_prechg_end_c = 0, .chg_normal_max_c = 55, .chg_max_c = 100,
    .dsg_min_c = -20, .dsg_normal_max_c = 60, .dsg_max_c = 100,
    .fet_min_c = -20, .fet_max_c = 100,
    .hyst_c = 3,
};

static void chg(const char *what, int16_t t, ntc_chg_band_t prev, ntc_chg_band_t want)
{
    ntc_chg_band_t got = ntc_band_charge(t, prev, &B);
    checks++;
    if (got != want) {
        failures++;
        printf("  FAIL  %-52s %dC -> %s, want %s\n", what, t,
               ntc_band_charge_name(got), ntc_band_charge_name(want));
    } else {
        printf("  ok    %-52s %dC -> %s\n", what, t, ntc_band_charge_name(got));
    }
}

static void dsg(const char *what, int16_t t, ntc_dsg_band_t prev, ntc_dsg_band_t want)
{
    ntc_dsg_band_t got = ntc_band_discharge(t, prev, &B);
    checks++;
    if (got != want) {
        failures++;
        printf("  FAIL  %-52s %dC -> %s, want %s\n", what, t,
               ntc_band_discharge_name(got), ntc_band_discharge_name(want));
    } else {
        printf("  ok    %-52s %dC -> %s\n", what, t, ntc_band_discharge_name(got));
    }
}

static void fet(const char *what, int16_t t, ntc_fet_band_t prev, ntc_fet_band_t want)
{
    ntc_fet_band_t got = ntc_band_mosfet(t, prev, &B);
    checks++;
    if (got != want) {
        failures++;
        printf("  FAIL  %-52s %dC -> %s, want %s\n", what, t,
               ntc_band_mosfet_name(got), ntc_band_mosfet_name(want));
    } else {
        printf("  ok    %-52s %dC -> %s\n", what, t, ntc_band_mosfet_name(got));
    }
}

static void blocks(const char *what, int got, int want)
{
    checks++;
    if (got != want) {
        failures++;
        printf("  FAIL  %-52s got %d, want %d\n", what, got, want);
    } else {
        printf("  ok    %-52s %d\n", what, got);
    }
}

int main(void)
{
    printf("charge ladder: -10 / 0 / 55 / 100\n");
    chg("deep cold",            -25, NTC_CHG_NORMAL,    NTC_CHG_OFF_COLD);
    chg("on the cold point",    -10, NTC_CHG_OFF_COLD,  NTC_CHG_PRECHARGE);
    chg("inside pre-charge",     -5, NTC_CHG_PRECHARGE, NTC_CHG_PRECHARGE);
    chg("on the pre-charge end",  0, NTC_CHG_PRECHARGE, NTC_CHG_NORMAL);
    chg("room temperature",      25, NTC_CHG_NORMAL,    NTC_CHG_NORMAL);
    chg("on the normal ceiling", 55, NTC_CHG_NORMAL,    NTC_CHG_EXTENDED);
    chg("inside extended",       80, NTC_CHG_EXTENDED,  NTC_CHG_EXTENDED);
    chg("on the hot point",     100, NTC_CHG_EXTENDED,  NTC_CHG_OFF_HOT);

    printf("\ncharge hysteresis: a band holds until %dC back through its point\n", B.hyst_c);
    chg("just under the pre-charge end, coming down",   -1, NTC_CHG_NORMAL,    NTC_CHG_NORMAL);
    chg("2C under, still held",                         -2, NTC_CHG_NORMAL,    NTC_CHG_NORMAL);
    chg("past the hysteresis, drops to pre-charge",     -4, NTC_CHG_NORMAL,    NTC_CHG_PRECHARGE);
    chg("1C under the hot point, coming down",          99, NTC_CHG_OFF_HOT,   NTC_CHG_OFF_HOT);
    chg("4C under the hot point, releases",             96, NTC_CHG_OFF_HOT,   NTC_CHG_EXTENDED);
    chg("rising to the same 99C does NOT trip",         99, NTC_CHG_EXTENDED,  NTC_CHG_EXTENDED);

    printf("\ndischarge ladder: -20 / 60 / 100\n");
    dsg("deep cold",            -30, NTC_DSG_NORMAL,   NTC_DSG_OFF_COLD);
    dsg("on the cold point",    -20, NTC_DSG_OFF_COLD, NTC_DSG_NORMAL);
    dsg("cold but permitted",   -10, NTC_DSG_NORMAL,   NTC_DSG_NORMAL);
    dsg("on the normal ceiling", 60, NTC_DSG_NORMAL,   NTC_DSG_EXTENDED);
    dsg("inside extended",       90, NTC_DSG_EXTENDED, NTC_DSG_EXTENDED);
    dsg("on the hot point",     100, NTC_DSG_EXTENDED, NTC_DSG_OFF_HOT);
    dsg("hot band holds at 98C", 98, NTC_DSG_OFF_HOT,  NTC_DSG_OFF_HOT);

    printf("\ncharge is colder than discharge: -15C stops charging, not discharging\n");
    chg("-15C charge",          -15, NTC_CHG_NORMAL,   NTC_CHG_OFF_COLD);
    dsg("-15C discharge",       -15, NTC_DSG_NORMAL,   NTC_DSG_NORMAL);

    printf("\nMOSFET ladder: -20 / 100\n");
    fet("too cold",             -25, NTC_FET_OK,       NTC_FET_OFF_COLD);
    fet("in range",              40, NTC_FET_OK,       NTC_FET_OK);
    fet("on the hot point",     100, NTC_FET_OK,       NTC_FET_OFF_HOT);
    fet("hot band holds at 98C", 98, NTC_FET_OFF_HOT,  NTC_FET_OFF_HOT);

    printf("\nwhich bands forbid a FET\n");
    blocks("off-cold blocks charge",    ntc_band_blocks_charge(NTC_CHG_OFF_COLD),  1);
    blocks("pre-charge blocks charge",   ntc_band_blocks_charge(NTC_CHG_PRECHARGE), 1);
    blocks("normal permits charge",     ntc_band_blocks_charge(NTC_CHG_NORMAL),    0);
    blocks("extended permits charge",   ntc_band_blocks_charge(NTC_CHG_EXTENDED),  0);
    blocks("off-hot blocks charge",     ntc_band_blocks_charge(NTC_CHG_OFF_HOT),   1);
    blocks("off-cold blocks discharge", ntc_band_blocks_discharge(NTC_DSG_OFF_COLD), 1);
    blocks("normal permits discharge",  ntc_band_blocks_discharge(NTC_DSG_NORMAL),   0);
    blocks("extended permits discharge", ntc_band_blocks_discharge(NTC_DSG_EXTENDED), 0);
    blocks("off-hot blocks discharge",  ntc_band_blocks_discharge(NTC_DSG_OFF_HOT),  1);

    printf("\nhysteresis disabled: every point is a clean edge\n");
    {
        ntc_bands_t nh = B;
        nh.hyst_c = 0;
        checks++;
        if (ntc_band_charge(-1, NTC_CHG_NORMAL, &nh) != NTC_CHG_PRECHARGE) {
            failures++; printf("  FAIL  hyst 0: -1C drops straight to pre-charge\n");
        } else printf("  ok    hyst 0: -1C drops straight to pre-charge\n");
    }

    printf("\n%d checks, %d failures\n", checks, failures);
    return failures ? 1 : 0;
}
