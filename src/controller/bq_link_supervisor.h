/*
 * bq_link_supervisor.h -- notice when the AFE has lost its configuration, and
 * put it back.
 *
 * Every autonomous SHUTDOWN path is disabled by project rule, but two resets
 * cannot be turned off: VBAT brownout below 2.5 V (UVLO), and the factory trim
 * diagnostic failing at power-up or RESET. After either, the chip runs its
 * factory profile -- a 7-cell map that reads cells 2 and 4 as 0 mV on this
 * interleaved wiring, default protections, FET_EN clear, PASSQ zeroed -- and on
 * V2, which has no console RX, that state is terminal without SWD.
 *
 * Recovery restores the *configuration* only, by way of
 * `bq_controller_reconfigure()`. Whether the power path may close is decided by
 * whoever owns the reason it was open: a latched wire fault, a software
 * overcurrent trip, or a manual override. Closing it blindly turns a standing
 * fault into FET chatter -- reconfigure, close, re-trip in ~3 s, repeat on every
 * retry interval.
 */

#ifndef BQ_LINK_SUPERVISOR_H
#define BQ_LINK_SUPERVISOR_H

#include <stdint.h>
#include <stdbool.h>

typedef enum {
    LINK_OK = 0,     /* chip answering and still configured as written         */
    LINK_SILENT,     /* no answer: SHUTDOWN, brownout in progress, or bus fault */
    LINK_RECONFIG,   /* alive but its settings are gone; putting them back      */
    LINK_DEGRADED    /* recovery keeps failing; retry slowly and keep reporting */
} link_state_t;

/* Why recovery was entered, kept for the field log: a brownout and a bus fault
 * look the same once the chip is answering again. */
typedef enum {
    LINK_REASON_NONE = 0,
    LINK_REASON_POR,          /* BatteryStatus[POR] -- the authoritative signal */
    LINK_REASON_SILENT,       /* I2C not answering                              */
    LINK_REASON_DRIFT,        /* config verify mismatched with no POR           */
    LINK_REASON_EXPECTED      /* MCU-initiated reset; suppressed                */
} link_reason_t;

typedef struct {
    link_state_t  state;
    link_reason_t last_reason;
    uint16_t      por_count;          /* unexpected chip resets since boot   */
    uint16_t      silent_count;
    uint16_t      drift_count;
    uint16_t      reconfig_ok_count;
    uint16_t      reconfig_fail_count;
    uint16_t      fet_en_withheld;    /* recoveries that deliberately left the
                                       * power path open because a fault stood */
    uint32_t      last_recovery_ms;
    uint32_t      next_attempt_ms;
    uint8_t        attempt;           /* position in the backoff table        */
} link_sup_status_t;

void bq_link_supervisor_init(void);

/*
 * Call at the top of the controller tick, before measurements are trusted.
 * Returns true while the link is not OK, which the caller should read as "this
 * tick's readings are not from a configured chip".
 */
bool bq_link_supervisor_tick(void);

/*
 * Tell the supervisor that the MCU itself is about to reset the chip, so the POR
 * it is about to see is expected and must not trigger a recovery on top of the
 * one already in progress. Cleared automatically once consumed.
 */
void bq_link_supervisor_expect_por(void);

const link_sup_status_t *bq_link_supervisor_status(void);
const char *bq_link_reason_name(link_reason_t r);

#endif /* BQ_LINK_SUPERVISOR_H */
