/*
 * cell_open_wire.h -- cell tap open-wire classifier.
 *
 * Localises a broken cell sense/balance wire from the cell-voltage signature, or
 * reports that it cannot tell; it takes no action, wire_fault.c is the response.
 * On the BQ76907 a VC pin is both sense input and balance-current path, so an
 * open balance wire and an open cell tap are the same fault: the pin floats, the
 * cell below reads low, the cell above reads the span of two cells.
 *
 * Two paths. DRIFT, while both readings are in range: cell N down, cell N+1 up,
 * pair-sum constant. SETTLED, once a reading saturates and the pair-sum invariant
 * breaks: a reading below 0 mV or above ~4.0 V is impossible for a real cell.
 *
 * Fail closed: a top-tap open and a dead cell 4 both read "cell low, no mirror",
 * and only the stack separates them -- a top-tap open leaves the stack unchanged
 * so sum(cells) diverges from it, a dead cell drops the stack with it. Where the
 * evidence does not separate them the verdict is OW_AMBIGUOUS, never OW_TOP_TAP,
 * which denies the grace window.
 */

#ifndef CELL_OPEN_WIRE_H
#define CELL_OPEN_WIRE_H

#include <stdint.h>
#include <stdbool.h>

#ifndef OW_CELLS
#define OW_CELLS 4
#endif

/* What the classifier concluded about one sample. */
typedef enum {
    OW_NONE = 0,        /* nothing wrong with the cell readings              */
    OW_MID_TAP,         /* tap between cell N and N+1 open; `tap` = N (1..3) */
    OW_TOP_TAP,         /* topmost tap (VC7) open                           */
    OW_BOTTOM_TAP,      /* bottom tap (VC0) open                            */
    OW_MULTI,           /* more than one impossible reading; cannot localise */
    OW_REAL_CELL,       /* a cell really is low, and the stack agrees        */
    OW_AMBIGUOUS        /* impossible-looking but not separable -- deny grace */
} ow_kind_t;

typedef struct {
    ow_kind_t kind;
    uint8_t   tap;          /* 1..OW_CELLS-1 for OW_MID_TAP, else 0          */
    uint8_t   cell;         /* the collapsed cell, 1-based, 0 if n/a         */
    bool      saturated;    /* verdict came from the settled path            */
    int16_t   sum_minus_stack_mv;   /* the evidence, for logging             */
} ow_verdict_t;

/* One measurement set. Signed throughout: a collapsed node reads negative. */
typedef struct {
    int16_t cell_mv[OW_CELLS];
    int16_t stack_mv;
    int32_t current_ma;
} ow_sample_t;

/* Tunables, all exposed as runtime parameters. */
typedef struct {
    int16_t  cell_min_plausible_mv;  /* below this a reading is impossible    */
    int16_t  cell_max_plausible_mv;  /* above this a reading is impossible    */
    int16_t  drift_eps_mv;           /* movement that counts as drift         */
    int16_t  pair_sum_tol_mv;        /* pair-sum match, drift path only       */
    int16_t  stack_tol_mv;           /* sum-vs-stack match                    */
    int32_t  current_quiet_ma;       /* above this, load explains movement    */
    /* A reading below this is not a depressed cell but an absent channel. Counted
     * only: two or more absent channels mean something other than a single open
     * tap, which no single-tap verdict may claim. */
    int16_t  cell_absent_mv;
    /* A sum-vs-stack divergence this large means the measurement chain is
     * inconsistent even if every individual cell looks plausible. Applied as a
     * catch-all AFTER the specific rules, so it never pre-empts a localised
     * verdict. */
    int16_t  stack_fault_mv;
} ow_config_t;

void ow_config_defaults(ow_config_t *cfg);

/* The whole decision as a pure function: no I2C, no timers, no globals, so it can
 * be replayed on the host. `baseline` is the slow per-cell average from before
 * the event, or NULL when none is available yet -- the settled path alone is then
 * used. */
ow_verdict_t ow_classify(const ow_sample_t *s,
                         const int16_t *baseline,
                         const ow_config_t *cfg);

/* ---- stateful layer: debounce and latch over successive ticks ------------- */

typedef enum { OW_ST_IDLE = 0, OW_ST_SUSPECT, OW_ST_CONFIRMED, OW_ST_RECOVERY } ow_state_t;

typedef struct {
    ow_state_t   state;
    ow_verdict_t verdict;       /* latched verdict once CONFIRMED             */
    uint8_t      confirm_count;
    uint8_t      clear_count;
    uint16_t     events;        /* confirmations since boot, for telemetry    */
    int16_t      baseline_mv[OW_CELLS];
    bool         baseline_valid;
} ow_ctx_t;

void ow_init(ow_ctx_t *ctx);

/*
 * Feed one tick. `fresh` must be false when the sample cannot be trusted --
 * balancing active, or a large current step -- in which case baselines are held
 * and no verdict is formed. Returns true when the latched verdict changed, which
 * is the caller's cue to report or act.
 */
bool ow_tick(ow_ctx_t *ctx, const ow_sample_t *s, const ow_config_t *cfg, bool fresh);

/*
 * Voltage across the pair of cells spanning the floating node at `tap`. The two
 * readings inside the pair are untrustworthy individually, but the stack
 * measurement is independent of the broken tap, so the stack less every cell
 * OUTSIDE the pair is the pair's real voltage -- the only quantity an
 * undervoltage substitute can use. `tap` is 1-based as the classifier reports it.
 * Returns 0 for an out-of-range tap, which callers must treat as "no opinion",
 * never as a pair at 0 mV.
 */
int32_t ow_pair_sum_mv(uint8_t tap, const int16_t *cell_mv, int16_t stack_mv);

const char *ow_kind_name(ow_kind_t k);

#endif /* CELL_OPEN_WIRE_H */
