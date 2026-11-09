/*
 * wire_fault.h -- what the BMS does about a confirmed open cell tap.
 *
 * cell_open_wire.c decides what the fault is; this decides what to do. The
 * response is an asymmetric limp profile, the only BQ76907 mechanism that
 * suspends ONE protection and leaves the rest autonomous:
 *
 *      DSG FET Protections A  0xF5 -> 0x75     (CUV bit 7 cleared)
 *      CHG FET Protections A  0xA7 unchanged   (COV still opens CHG)
 *
 * Discharge continues; charging stops, because the mirrored cell's phantom COV
 * still opens the charge FET. Current, temperature and UTC protections stay
 * autonomous on both FETs. A host force-on is not used: it would suspend all six
 * protections mapped to the FET it forces, not the phantom one.
 *
 * Applying the profile costs a pack-open, since entering CONFIG_UPDATE drops both
 * FET drivers. Leaving it needs no timer: a condition already true when
 * CONFIG_UPDATE exits faults immediately, bypassing the configured delay, so
 * restoring the production profile with the wire still open opens the FETs at
 * once.
 *
 * Fail closed: only a localised mid-tap or top-tap earns the grace window;
 * OW_AMBIGUOUS, OW_MULTI and OW_REAL_CELL go straight to service-required with
 * the production profile intact. A bottom-tap open is not gracable either -- it
 * disturbs the VSS reference and can fire the VSS diagnostic, which sits in
 * `Both FET Protections B`, above the per-FET maps and out of reach.
 */

#ifndef WIRE_FAULT_H
#define WIRE_FAULT_H

#include <stdint.h>
#include <stdbool.h>
#include "cell_open_wire.h"

typedef enum {
    WF_INACTIVE = 0,    /* no wire fault                                      */
    WF_LIMP,            /* limp profile applied, grace window running         */
    WF_SERVICE          /* grace spent or fault not gracable: pack out of use */
} wf_state_t;

typedef struct {
    wf_state_t   state;
    ow_kind_t    kind;          /* what the classifier concluded              */
    uint8_t      tap;           /* which tap, for a mid-tap                   */
    uint32_t     entered_ms;    /* when the current state began               */
    uint32_t     grace_left_ms; /* 0 once spent                               */
    uint16_t     reapply_count; /* profile re-applied after an AFE reset       */
    uint16_t     uv_substitute_trips;
    bool         balancing_locked;
    bool         soc_anchors_suppressed;
} wf_status_t;

void wire_fault_init(void);

/*
 * Drive the response from a classifier verdict. Call once per controller tick,
 * after measurements and before balancing. `fault_stands` must be true for as
 * long as the classifier still believes in the fault -- CONFIRMED *or* RECOVERY.
 * The classifier leaves CONFIRMED on the FIRST clean sample and then spends
 * OW_CLEAR_TICKS proving the wire really is back, so passing `state == CONFIRMED`
 * would drop the limp profile on that first clean sample and throw the clear
 * debounce away.
 */
void wire_fault_tick(const ow_verdict_t *v, bool fault_stands,
                     const int16_t *cell_mv, int16_t stack_mv);

const wf_status_t *wire_fault_status(void);

/* Queried by run_balancing_task(): a phantom COV would otherwise keep the
 * balancer pumping a floating channel. The chip also kills balancing on a
 * CUV/COV alert by itself; this additionally stops the firmware's own stale
 * balance mask from freezing cell reads. */
bool wire_fault_balancing_blocked(void);

/* Queried by battery_soc_soh: the phantom CUV/COV anchors must not fire. */
bool wire_fault_soc_anchors_suppressed(void);

#endif /* WIRE_FAULT_H */
