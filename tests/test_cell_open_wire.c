/*
 * test_cell_open_wire.c -- host test for the open-wire classifier.
 *
 * Every case below is a real measurement from docs/openwire_bench_test_journal.md
 * (dap-004, 2026-08-14 per-tap table; dap-017 resting values, 2026-11-01), not a
 * number someone liked the look of. The point is that the classifier can be
 * checked against the hardware's actual behaviour without a board attached --
 * which matters because the open-wire tests themselves need hands on the tap
 * harness and cannot be run remotely.
 *
 * Build and run:  make -C .. test
 */

#include "../src/controller/cell_open_wire.h"
#include <stdio.h>
#include <string.h>

static int failures;
static int checks;

static void expect(const char *what, const ow_sample_t *s, const int16_t *base,
                   ow_kind_t want_kind, uint8_t want_tap)
{
    ow_config_t cfg;
    ow_config_defaults(&cfg);
    ow_verdict_t v = ow_classify(s, base, &cfg);
    checks++;

    bool ok = (v.kind == want_kind) && (want_tap == 0 || v.tap == want_tap);
    if (!ok) {
        failures++;
        printf("  FAIL  %-46s got %s/tap%u, want %s/tap%u  (sum-stack %d mV)\n",
               what, ow_kind_name(v.kind), v.tap,
               ow_kind_name(want_kind), want_tap, v.sum_minus_stack_mv);
    } else {
        printf("  ok    %-46s %s", what, ow_kind_name(v.kind));
        if (v.tap) printf(" tap%u", v.tap);
        printf("%s\n", v.saturated ? " (settled)" : " (drift)");
    }
}

int main(void)
{
    /* dap-017 resting, 2026-11-01. Stack is the measured 13127, which is 7 mV
     * off the cell sum -- the healthy residual the stack tolerance must accept. */
    const int16_t rest[OW_CELLS] = { 3278, 3281, 3281, 3280 };
    ow_sample_t healthy = { { 3278, 3281, 3281, 3280 }, 13127, 0 };

    printf("healthy pack\n");
    expect("resting, balanced", &healthy, rest, OW_NONE, 0);

    /* A load step moves every cell the same way; must not read as drift. */
    ow_sample_t loaded = { { 3180, 3183, 3183, 3182 }, 12728, -20000 };
    expect("40 A discharge, all cells sag together", &loaded, rest, OW_NONE, 0);

    printf("\nsettled open-wire signatures (journal 2026-08-14)\n");

    /* Test 1: tap 3 open. c3 = -0.24 V, c4 = 5.98 V (clamped from ~6.6 V).
     * Note the measured pair-sum is 5.74 V, NOT 6.56 -- a classifier keying on
     * a tight pair-sum here would miss the fault entirely. */
    ow_sample_t tap3 = { { 3278, 3281, -240, 5980 }, 13128, 0 };
    expect("tap 3: c3 -240 mV, c4 5980 mV", &tap3, rest, OW_MID_TAP, 3);

    /* Test 3: tap 2 open. c2 = -0.22 V, c3 = 5.98 V. */
    ow_sample_t tap2 = { { 3278, -220, 5980, 3280 }, 13128, 0 };
    expect("tap 2: c2 -220 mV, c3 5980 mV", &tap2, rest, OW_MID_TAP, 2);

    /* Test 4: tap 1 open, caught settled. c1 ~1.5 V, c2 ~5.05 V. c1 is still
     * inside the plausible window, so only c2 is impossible -- the pair rule
     * must still fire off the high side plus its low neighbour. */
    ow_sample_t tap1_settled = { { 1520, 5050, 3281, 3280 }, 13128, 0 };
    expect("tap 1 settled: c1 1520 mV, c2 5050 mV", &tap1_settled, rest,
           OW_MID_TAP, 1);

    /* Test 2: top tap open. c4 = -0.6 V, NO mirror, and the stack stayed at
     * 13.12 V -- which is the only thing separating this from a dead cell 4. */
    ow_sample_t toptap = { { 3278, 3281, 3281, -600 }, 13120, 0 };
    expect("top tap: c4 -600 mV, stack unchanged", &toptap, rest, OW_TOP_TAP, 0);

    printf("\nfail-closed cases\n");

    /* The dangerous twin of the above: a genuinely dead cell 4. Cell reads low
     * AND the stack falls with it. Must NOT be called an open wire. Using a
     * plausible-but-low 1.8 V, which is what a failing cell looks like. */
    ow_sample_t deadcell = { { 3278, 3281, 3281, 1800 }, 11640, 0 };
    expect("real dead cell 4, stack agrees", &deadcell, rest, OW_NONE, 0);

    /* Same but driven below zero, which no real cell does, while the stack
     * still agrees. Impossible reading plus consistent stack = unexplained.
     * Must be AMBIGUOUS, never TOP_TAP: that is the fail-closed rule. */
    ow_sample_t weird = { { 3278, 3281, 3281, -600 }, 9240, 0 };
    expect("c4 negative but stack agrees", &weird, rest, OW_AMBIGUOUS, 0);

    /* Two impossible readings that are not an adjacent low/high pair: present,
     * but refuse to localise. */
    ow_sample_t multi = { { -200, 3281, -240, 5980 }, 13128, 0 };
    expect("two collapsed nodes", &multi, rest, OW_MULTI, 0);

    printf("\ndrift path, before anything saturates\n");

    /* Tap 3 mid-collapse: c3 down 400 mV, c4 up 400 mV, pair-sum preserved.
     * This is the window pre-emption would have to act in. */
    ow_sample_t drift = { { 3278, 3281, 2881, 3681 }, 13127, 0 };
    expect("tap 3 drifting, pair-sum held", &drift, rest, OW_MID_TAP, 3);

    /* Same magnitude of movement but the pair-sum did NOT hold, so it is not a
     * floating node -- must not be claimed as one. */
    ow_sample_t notpair = { { 3278, 3281, 2881, 3281 }, 12727, 0 };
    expect("one cell down, neighbour flat", &notpair, rest, OW_NONE, 0);

    /* dap-017, 2026-11-02: a bare CMD_RESET leaves the chip on its factory
     * 7-cell map, which reads cells 2 and 4 as 0 mV with the stack still
     * correct. The first version of this classifier called that a top-tap open
     * and the wire-fault module applied the limp profile to a healthy pack. */
    ow_sample_t factory_default = { { 3277, 0, 3280, 0 }, 13125, 0 };
    expect("post-reset factory 7-cell map (c2,c4 = 0)", &factory_default, rest,
           OW_MULTI, 0);

    /* Accidental double pull, dap-017 2026-11-02: tap 1 and tap 2 open together
     * produced chaotic readings where every cell was individually plausible but
     * volts of pack voltage were unaccounted for. The classifier returned
     * OW_NONE for these two before the stack-divergence catch-all was added. */
    ow_sample_t chaos1 = { { 706, 1634, 3438, 3276 }, 13121, 0 };
    expect("multi-open chaos, sum-stack -4067", &chaos1, rest, OW_AMBIGUOUS, 0);
    ow_sample_t chaos2 = { { 2523, 2408, 3342, 3277 }, 13121, 0 };
    expect("multi-open chaos, sum-stack -1571", &chaos2, rest, OW_AMBIGUOUS, 0);

    printf("\npair-sum reconstruction (the UV substitute's only input)\n");
    {
        /*
         * dap-017, 2026-11-02, the regression that matters. Tap 1 open settles at
         * c1 = 1485 mV -- a plausible cell voltage -- so the first uv_substitute()
         * identified "intact" cells by testing 0 < v < 4000, counted the collapsed
         * c1 as intact, and subtracted it from the stack. What came back was c2's
         * share alone, 5078 mV, under the 5300 mV floor: the substitute tripped and
         * latched SERVICE REQUIRED 700 ms into a 10-minute grace window.
         *
         * The pair is two healthy cells at 6563 mV. Nothing should trip.
         */
        const int16_t tap1_real[OW_CELLS] = { 1485, 5084, 3280, 3279 };
        const int32_t floor_mv = 2 * (2500 + 150);      /* CUV default + margin  */

        struct { const char *what; uint8_t tap; const int16_t *c; int16_t stack;
                 int32_t want; bool want_trip; } cases[] = {
          /* The bug, as measured. */
          { "tap 1 open, collapsed cell reads plausible", 1, tap1_real, 13122, 6563, false },
          /* The other taps, where the old heuristic happened to agree. */
          { "tap 3 open, c3 negative c4 clamped",         3,
            (const int16_t[]){ 3278, 3281, -240, 5980 }, 13128, 6569, false },
          { "tap 2 open, c2 negative c3 clamped",         2,
            (const int16_t[]){ 3278, -220, 5980, 3280 }, 13128, 6570, false },
          /* A pair that has genuinely sagged must still trip: this is the whole
           * point of the substitute, so prove it did not get disarmed. */
          { "tap 1 open AND the pair truly flat at 2.4 V", 1,
            (const int16_t[]){ 1100, 3700, 3280, 3279 }, 11359, 4800, true },
          /* Out of range -- no opinion, not a pair at 0 mV. */
          { "tap 0 (not localised) yields no opinion",     0, tap1_real, 13122, 0, false },
        };

        for (unsigned i = 0; i < sizeof(cases)/sizeof(cases[0]); i++) {
            int32_t got = ow_pair_sum_mv(cases[i].tap, cases[i].c, cases[i].stack);
            bool trip = (cases[i].tap != 0) && (got < floor_mv);
            checks++;
            if (got != cases[i].want || trip != cases[i].want_trip) {
                failures++;
                printf("  FAIL  %-46s got %ld mV (%s), want %ld mV (%s)\n",
                       cases[i].what, (long)got, trip ? "trip" : "ok",
                       (long)cases[i].want, cases[i].want_trip ? "trip" : "ok");
            } else {
                printf("  ok    %-46s %ld mV, %s\n", cases[i].what, (long)got,
                       trip ? "trips correctly" : "no trip");
            }
        }
    }

    printf("\nstate machine: debounce and latch\n");
    {
        ow_config_t cfg; ow_config_defaults(&cfg);
        ow_ctx_t ctx; ow_init(&ctx);

        /* Settle a baseline. */
        for (int i = 0; i < 20; i++) ow_tick(&ctx, &healthy, &cfg, true);
        checks++;
        if (ctx.state != OW_ST_IDLE) { failures++; printf("  FAIL  baseline settles to IDLE\n"); }
        else printf("  ok    baseline settles to IDLE\n");

        /* One bad sample must not latch. */
        ow_tick(&ctx, &tap3, &cfg, true);
        checks++;
        if (ctx.state != OW_ST_SUSPECT) { failures++; printf("  FAIL  single bad sample -> SUSPECT only\n"); }
        else printf("  ok    single bad sample -> SUSPECT only\n");

        /* Three consecutive do. */
        bool ch = ow_tick(&ctx, &tap3, &cfg, true);
        ch |= ow_tick(&ctx, &tap3, &cfg, true);
        checks++;
        if (ctx.state != OW_ST_CONFIRMED || !ch) { failures++; printf("  FAIL  three bad samples -> CONFIRMED\n"); }
        else printf("  ok    three bad samples -> CONFIRMED (tap %u)\n", ctx.verdict.tap);

        /* A glitch back to healthy must not immediately clear the latch. */
        ow_tick(&ctx, &healthy, &cfg, true);
        checks++;
        if (ctx.state != OW_ST_RECOVERY || ctx.verdict.kind == OW_NONE) {
            failures++; printf("  FAIL  one clean sample does not clear the latch\n");
        } else printf("  ok    one clean sample does not clear the latch\n");

        /* Ten do. */
        for (int i = 0; i < 10; i++) ow_tick(&ctx, &healthy, &cfg, true);
        checks++;
        if (ctx.state != OW_ST_IDLE || ctx.verdict.kind != OW_NONE) {
            failures++; printf("  FAIL  ten clean samples clear the latch\n");
        } else printf("  ok    ten clean samples clear the latch\n");

        /* ...and clear the evidence with it, or packet 131 keeps broadcasting
         * "none tap=1 cell=1" from the fault that just went away. */
        checks++;
        if (ctx.verdict.tap != 0 || ctx.verdict.cell != 0 ||
            ctx.verdict.sum_minus_stack_mv != 0) {
            failures++;
            printf("  FAIL  clearing wipes tap/cell/evidence (got tap%u cell%u %d mV)\n",
                   ctx.verdict.tap, ctx.verdict.cell, ctx.verdict.sum_minus_stack_mv);
        } else printf("  ok    clearing wipes tap/cell/evidence\n");

        /* Balancing active: must hold, not re-latch, and must not poison the
         * baseline. This is the g_balance_prev_mask freeze hazard in test form. */
        ow_init(&ctx);
        for (int i = 0; i < 20; i++) ow_tick(&ctx, &healthy, &cfg, true);
        int16_t saved[OW_CELLS];
        memcpy(saved, ctx.baseline_mv, sizeof(saved));
        for (int i = 0; i < 5; i++) ow_tick(&ctx, &tap3, &cfg, false);
        checks++;
        if (ctx.state != OW_ST_IDLE || memcmp(saved, ctx.baseline_mv, sizeof(saved)) != 0) {
            failures++; printf("  FAIL  stale samples ignored, baseline held\n");
        } else printf("  ok    stale samples ignored, baseline held\n");
    }

    printf("\n%d checks, %d failures\n", checks, failures);
    return failures ? 1 : 0;
}
