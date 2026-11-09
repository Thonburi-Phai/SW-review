/* cell_open_wire.c -- cell tap open-wire classifier; see cell_open_wire.h. */

#include "cell_open_wire.h"

/* Confirm on three consecutive ticks, clear on ten. At the 600 ms tick three is
 * ~1.8 s, fast enough to be useful while still rejecting a single bad read; the
 * slow clear stops a flapping wire toggling the fault state. */
#define OW_CONFIRM_TICKS 3u
#define OW_CLEAR_TICKS   10u

/* Baseline EMA shift: 1/8 per tick at 600 ms, a ~5 s time constant. */
#define OW_BASELINE_SHIFT 3

void ow_config_defaults(ow_config_t *cfg)
{
    /* Plausibility window, deliberately well outside the ~2.0..3.65 V a 4S LFP
     * cell lives in: its job is to catch the physically impossible, not to judge
     * cell health. */
    cfg->cell_min_plausible_mv = 0;      /* a real cell is never negative      */
    cfg->cell_max_plausible_mv = 4000;   /* 3.65 V is already abusive for LFP  */

    cfg->drift_eps_mv    = 50;
    /* Loose on purpose. Specificity comes from the paired test -- one cell down
     * AND its neighbour up, each by more than drift_eps -- since IR drop moves
     * every cell the same direction; the sum tolerance only has to accept a
     * genuine floating node and reject thousands of mV. */
    cfg->pair_sum_tol_mv = 150;  /* drift path, and the lone-impossible rule   */

    /* Sum-vs-stack. The healthy residual is a few mV, but it is a sum of four
     * cell measurements plus a separate stack channel, so allow real margin; a
     * top-tap open puts this in the thousands of mV. */
    cfg->stack_tol_mv = 300;

    /* Above this, IR drop can move all cells together and a load step could be
     * mistaken for drift. */
    cfg->current_quiet_ma = 500;

    /* Below 1.0 V a 4S LFP channel is not reading a cell at all. Separate from
     * cell_min_plausible_mv, which is 0 and so does not catch a channel sitting
     * at exactly 0 mV -- what a factory-default 7-cell map produces on this
     * interleaved wiring. */
    cfg->cell_absent_mv = 1000;

    /* Sum-vs-stack divergence that means the measurement chain is inconsistent
     * even when every individual cell reading is plausible. */
    cfg->stack_fault_mv = 500;
}

static bool implausible(int16_t mv, const ow_config_t *cfg)
{
    return (mv < cfg->cell_min_plausible_mv) || (mv > cfg->cell_max_plausible_mv);
}

/* ------------------------------------------------------------------------- */
/* the settled path: saturated readings, which are the unambiguous ones       */
/* ------------------------------------------------------------------------- */
static ow_verdict_t classify_settled(const ow_sample_t *s, const int16_t *baseline,
                                     const ow_config_t *cfg, int16_t sum_minus_stack)
{
    ow_verdict_t v = { OW_NONE, 0, 0, true, sum_minus_stack };
    int8_t low = -1, high = -1;
    uint8_t n_low = 0, n_high = 0;

    for (uint8_t i = 0; i < OW_CELLS; i++) {
        if (s->cell_mv[i] < cfg->cell_min_plausible_mv) { n_low++;  low  = (int8_t)i; }
        if (s->cell_mv[i] > cfg->cell_max_plausible_mv) { n_high++; high = (int8_t)i; }
    }

    if ((n_low + n_high) == 0u) {
        return v;                   /* nothing impossible; settled path says nothing */
    }

    if ((n_low > 1u) || (n_high > 1u)) {
        /* Multiple simultaneous opens make the readings appear random: report
         * presence, refuse to localise. */
        v.kind = OW_MULTI;
        return v;
    }

    /* One impossibly low AND one impossibly high, adjacent, low below high: a
     * mid-tap, and nothing else produces it. The pair-sum is NOT checked here --
     * it is exactly the quantity the clamp has destroyed. */
    if ((n_low == 1u) && (n_high == 1u) && (high == low + 1)) {
        v.kind = OW_MID_TAP;
        v.tap  = (uint8_t)(low + 1);        /* tap between cell low+1 and low+2 */
        v.cell = (uint8_t)(low + 1);
        return v;
    }

    /* One impossible reading whose neighbour merely *moved* rather than going
     * impossible too: the un-clamped half of a mid-tap collapse. The pair either
     * side of a floating node spans two real cells, so with neither reading
     * clamped their SUM is preserved and identifies the tap. Checked before the
     * lone-low rules below, so a mid-tap is never mistaken for a top or bottom
     * tap. */
    if (baseline != 0) {
        for (uint8_t i = 0; i < OW_CELLS; i++) {
            if (!implausible(s->cell_mv[i], cfg)) {
                continue;
            }
            for (int8_t d = -1; d <= 1; d += 2) {
                int8_t pj = (int8_t)i + d;
                if (pj < 0 || pj >= (int8_t)OW_CELLS) {
                    continue;
                }
                if (implausible(s->cell_mv[pj], cfg)) {
                    continue;       /* both impossible: handled above */
                }
                int32_t now  = (int32_t)s->cell_mv[i] + s->cell_mv[pj];
                int32_t base = (int32_t)baseline[i]   + baseline[pj];
                int32_t err  = now - base;
                if (err < 0) { err = -err; }
                if (err <= cfg->pair_sum_tol_mv) {
                    uint8_t lo_idx = (uint8_t)((pj < (int8_t)i) ? (uint8_t)pj : i);
                    v.kind = OW_MID_TAP;
                    v.tap  = (uint8_t)(lo_idx + 1u);
                    v.cell = (uint8_t)(lo_idx + 1u);
                    return v;
                }
            }
        }
    }

    /* One impossible reading and no partner. Which tap depends on position, and
     * the stack is what makes it decidable. */
    if ((n_low == 1u) && (n_high == 0u)) {
        v.cell = (uint8_t)(low + 1);

        bool stack_disagrees = (sum_minus_stack < -cfg->stack_tol_mv) ||
                               (sum_minus_stack >  cfg->stack_tol_mv);

        if (!stack_disagrees) {
            /* The stack fell with the cell, so the charge really is missing from
             * the pack. A negative reading is still impossible for a real cell,
             * so this is not OW_REAL_CELL either: deny the grace rather than
             * guess. */
            v.kind = OW_AMBIGUOUS;
            return v;
        }

        if (low == OW_CELLS - 1) {
            v.kind = OW_TOP_TAP;        /* top cell collapsed, stack unaffected */
        } else if (low == 0) {
            v.kind = OW_BOTTOM_TAP;     /* cell 1 collapsed, no mirror          */
        } else {
            /* A middle cell low with no mirror and a stack mismatch matches no
             * characterised signature. */
            v.kind = OW_AMBIGUOUS;
        }
        return v;
    }

    /* An impossibly HIGH cell with no low partner: present but unlocalised,
     * rather than inventing a tap. */
    v.kind = OW_AMBIGUOUS;
    v.cell = (uint8_t)(high + 1);
    return v;
}

/* ------------------------------------------------------------------------- */
/* the drift path: both readings still in range, so the pair-sum is usable    */
/* ------------------------------------------------------------------------- */
static ow_verdict_t classify_drift(const ow_sample_t *s, const int16_t *baseline,
                                   const ow_config_t *cfg, int16_t sum_minus_stack)
{
    ow_verdict_t v = { OW_NONE, 0, 0, false, sum_minus_stack };

    for (uint8_t i = 0; i + 1u < OW_CELLS; i++) {
        int32_t d_lo = (int32_t)s->cell_mv[i]     - baseline[i];
        int32_t d_hi = (int32_t)s->cell_mv[i + 1] - baseline[i + 1];

        /* One down, its neighbour up, by more than noise. */
        if ((d_lo > -cfg->drift_eps_mv) || (d_hi < cfg->drift_eps_mv)) {
            continue;
        }

        /* And the pair-sum held: what a load step cannot fake, because IR drop
         * moves every cell the same way. Only meaningful while both readings are
         * in range, which is why this path is skipped once anything saturates. */
        int32_t pair_now  = (int32_t)s->cell_mv[i] + s->cell_mv[i + 1];
        int32_t pair_base = (int32_t)baseline[i]   + baseline[i + 1];
        int32_t pair_err  = pair_now - pair_base;
        if (pair_err < 0) { pair_err = -pair_err; }

        if (pair_err <= cfg->pair_sum_tol_mv) {
            v.kind = OW_MID_TAP;
            v.tap  = (uint8_t)(i + 1);
            v.cell = (uint8_t)(i + 1);
            return v;
        }
    }

    /* Top tap during drift: the top cell falls while the stack does not follow,
     * because only the cell reading is corrupted when that tap floats. */
    {
        uint8_t top = OW_CELLS - 1u;
        int32_t d_top = (int32_t)s->cell_mv[top] - baseline[top];
        bool stack_disagrees = (sum_minus_stack < -cfg->stack_tol_mv) ||
                               (sum_minus_stack >  cfg->stack_tol_mv);
        if ((d_top < -cfg->drift_eps_mv) && stack_disagrees) {
            v.kind = OW_TOP_TAP;
            v.cell = (uint8_t)(top + 1u);
        }
    }
    return v;
}

ow_verdict_t ow_classify(const ow_sample_t *s, const int16_t *baseline,
                         const ow_config_t *cfg)
{
    int32_t sum = 0;
    bool any_saturated = false;
    uint8_t absent = 0;

    for (uint8_t i = 0; i < OW_CELLS; i++) {
        sum += s->cell_mv[i];
        if (implausible(s->cell_mv[i], cfg)) {
            any_saturated = true;
        }
        if (s->cell_mv[i] < cfg->cell_absent_mv) {
            absent++;
        }
    }

    int32_t diff = sum - (int32_t)s->stack_mv;
    if (diff >  32767) { diff =  32767; }
    if (diff < -32768) { diff = -32768; }
    int16_t sum_minus_stack = (int16_t)diff;

    /* More than one channel reading no cell at all is not a single open tap, and
     * no single-tap verdict may be built on it: a chip left on its factory 7-cell
     * map reads two cells as 0 mV with the stack still correct. The caller also
     * refuses to run while BatteryStatus[POR] is set; both are kept. */
    if (absent >= 2u) {
        ow_verdict_t v = { OW_MULTI, 0, 0, true, sum_minus_stack };
        return v;
    }

    /* Settled path first: an impossible reading is stronger evidence than any
     * amount of drift, and it does not need a baseline. */
    if (any_saturated) {
        return classify_settled(s, baseline, cfg, sum_minus_stack);
    }

    /* Nothing impossible. A real low cell with a matching stack is a cell problem,
     * not a wire problem -- reported explicitly so the caller can tell "healthy"
     * from "a cell is going". */
    if (baseline == 0) {
        ow_verdict_t v = { OW_NONE, 0, 0, false, sum_minus_stack };
        if ((sum_minus_stack > cfg->stack_fault_mv) ||
            (sum_minus_stack < -cfg->stack_fault_mv)) {
            v.kind = OW_AMBIGUOUS;   /* no baseline, but the chain still disagrees */
        }
        return v;
    }

    /* Load steps move all cells together; refuse to read drift out of them. */
    if ((s->current_ma > cfg->current_quiet_ma) ||
        (s->current_ma < -cfg->current_quiet_ma)) {
        ow_verdict_t v = { OW_NONE, 0, 0, false, sum_minus_stack };
        return v;
    }

    {
        ow_verdict_t v = classify_drift(s, baseline, cfg, sum_minus_stack);
        if (v.kind != OW_NONE) {
            return v;
        }

        /* Catch-all, deliberately last so it can never pre-empt a localised
         * verdict. If the cells and the stack disagree by this much, something is
         * wrong with the measurement chain whatever the individual readings look
         * like, and OW_AMBIGUOUS denies the grace window and reports. */
        if ((sum_minus_stack > cfg->stack_fault_mv) ||
            (sum_minus_stack < -cfg->stack_fault_mv)) {
            v.kind = OW_AMBIGUOUS;
        }
        return v;
    }
}

/* ------------------------------------------------------------------------- */
/* stateful layer                                                            */
/* ------------------------------------------------------------------------- */

void ow_init(ow_ctx_t *ctx)
{
    for (uint8_t i = 0; i < OW_CELLS; i++) {
        ctx->baseline_mv[i] = 0;
    }
    ctx->baseline_valid = false;
    ctx->state          = OW_ST_IDLE;
    ctx->confirm_count  = 0;
    ctx->clear_count    = 0;
    ctx->events         = 0;
    ctx->verdict.kind   = OW_NONE;
    ctx->verdict.tap    = 0;
    ctx->verdict.cell   = 0;
    ctx->verdict.saturated = false;
    ctx->verdict.sum_minus_stack_mv = 0;
}

static void baseline_update(ow_ctx_t *ctx, const ow_sample_t *s)
{
    if (!ctx->baseline_valid) {
        for (uint8_t i = 0; i < OW_CELLS; i++) {
            ctx->baseline_mv[i] = s->cell_mv[i];
        }
        ctx->baseline_valid = true;
        return;
    }
    for (uint8_t i = 0; i < OW_CELLS; i++) {
        int32_t b = ctx->baseline_mv[i];
        ctx->baseline_mv[i] = (int16_t)(b + ((s->cell_mv[i] - b) >> OW_BASELINE_SHIFT));
    }
}

bool ow_tick(ow_ctx_t *ctx, const ow_sample_t *s, const ow_config_t *cfg, bool fresh)
{
    if (!fresh) {
        /* Balancing active or a current step: hold everything. Updating the
         * baseline from a balance-distorted reading would poison the drift
         * detector. */
        return false;
    }

    ow_verdict_t v = ow_classify(s, ctx->baseline_valid ? ctx->baseline_mv : 0, cfg);
    bool bad = (v.kind != OW_NONE);
    bool changed = false;

    switch (ctx->state) {
    case OW_ST_IDLE:
        if (bad) {
            /* Freeze the baseline at the first suspicion, so the EMA cannot
             * follow the collapse and erase the evidence. */
            ctx->confirm_count = 1;
            ctx->verdict = v;
            ctx->state = OW_ST_SUSPECT;
        } else {
            baseline_update(ctx, s);
        }
        break;

    case OW_ST_SUSPECT:
        if (!bad) {
            ctx->state = OW_ST_IDLE;
            ctx->confirm_count = 0;
            ctx->verdict.kind = OW_NONE;
            break;
        }
        /* Keep the strongest verdict seen: a settled reading beats a drift guess,
         * because the collapse deepens from one into the other and the final
         * localisation is the one wanted. */
        if (v.saturated || (ctx->verdict.kind == OW_NONE)) {
            ctx->verdict = v;
        }
        if (++ctx->confirm_count >= OW_CONFIRM_TICKS) {
            ctx->state = OW_ST_CONFIRMED;
            ctx->events++;
            changed = true;
        }
        break;

    case OW_ST_CONFIRMED:
        /* Latched. Refine the verdict if the evidence got better, but never
         * downgrade to OW_NONE here -- that is the recovery path's job. */
        if (bad && v.saturated) {
            if (v.kind != ctx->verdict.kind || v.tap != ctx->verdict.tap) {
                ctx->verdict = v;
                changed = true;
            }
        }
        if (!bad) {
            ctx->clear_count = 1;
            ctx->state = OW_ST_RECOVERY;
        }
        break;

    case OW_ST_RECOVERY:
        if (bad) {
            ctx->clear_count = 0;
            ctx->state = OW_ST_CONFIRMED;
            break;
        }
        if (++ctx->clear_count >= OW_CLEAR_TICKS) {
            /* Slow clear: an intermittent wire must not toggle the fault state. */
            ctx->state = OW_ST_IDLE;
            /* Clear the whole verdict, not just its kind: stale tap/cell/
             * sum_minus_stack fields also reach packet 131. */
            ctx->verdict = (ow_verdict_t){ 0 };
            ctx->verdict.kind = OW_NONE;
            ctx->confirm_count = 0;
            ctx->baseline_valid = false;   /* reseed from the restored readings */
            changed = true;
        }
        break;
    }

    return changed;
}

int32_t ow_pair_sum_mv(uint8_t tap, const int16_t *cell_mv, int16_t stack_mv)
{
    int32_t others = 0;
    uint8_t lo, hi;

    if ((tap == 0u) || (tap >= OW_CELLS)) {
        return 0;
    }
    lo = (uint8_t)(tap - 1u);
    hi = tap;

    for (uint8_t i = 0; i < OW_CELLS; i++) {
        if ((i != lo) && (i != hi)) {
            others += cell_mv[i];
        }
    }
    return (int32_t)stack_mv - others;
}

const char *ow_kind_name(ow_kind_t k)
{
    switch (k) {
    case OW_NONE:       return "none";
    case OW_MID_TAP:    return "mid-tap";
    case OW_TOP_TAP:    return "top-tap";
    case OW_BOTTOM_TAP: return "bottom-tap";
    case OW_MULTI:      return "multi";
    case OW_REAL_CELL:  return "real-cell";
    case OW_AMBIGUOUS:  return "ambiguous";
    default:            return "?";
    }
}
