/* bq_link_supervisor.c -- AFE configuration-loss recovery; see the header. */

#include "bq_link_supervisor.h"
#include "bq_controller.h"
#include "bq7690x_status.h"
#include "wire_fault.h"
#include "sw_oc_protection.h"
#include "hal_systime.h"
#include "hal_uart.h"

/*
 * Backoff: fast enough that a brownout clearing in a second costs almost nothing,
 * slow enough that a chip which is genuinely gone does not burn the I2C bus and
 * the log forever.
 *
 *   0.6 s x5   one per controller tick -- the common transient
 *   5 s   x12  about a minute of retries
 *   60 s  ...  forever; no give-up state, DEGRADED keeps retrying and reporting
 */
#define BACKOFF_FAST_COUNT   5u
#define BACKOFF_FAST_MS      600u
#define BACKOFF_MID_COUNT    12u
#define BACKOFF_MID_MS       5000u
#define BACKOFF_SLOW_MS      60000u

static link_sup_status_t s_st;
static bool s_expect_por;

/*
 * Consecutive silent ticks before believing it. One tick of
 * `is_connected == false` is not evidence: bq_controller.c's stale-reading
 * detector sets that flag when five successive reads come back identical, which a
 * resting pack does. Three at the 600 ms I2C cycle is about 1.8 s -- a genuinely
 * quiet chip stays quiet for far longer, and a stale-detector false positive
 * clears on the very next read.
 */
#define SILENT_CONFIRM_TICKS 3u
static uint8_t s_silent_run;

static uint32_t backoff_ms(uint8_t attempt)
{
    if (attempt < BACKOFF_FAST_COUNT) {
        return BACKOFF_FAST_MS;
    }
    if (attempt < (BACKOFF_FAST_COUNT + BACKOFF_MID_COUNT)) {
        return BACKOFF_MID_MS;
    }
    return BACKOFF_SLOW_MS;
}

void bq_link_supervisor_init(void)
{
    s_st.state               = LINK_OK;
    s_st.last_reason         = LINK_REASON_NONE;
    s_st.por_count           = 0;
    s_st.silent_count        = 0;
    s_st.drift_count         = 0;
    s_st.reconfig_ok_count   = 0;
    s_st.reconfig_fail_count = 0;
    s_st.fet_en_withheld     = 0;
    s_st.last_recovery_ms    = 0;
    s_st.next_attempt_ms     = 0;
    s_st.attempt             = 0;
    s_expect_por             = false;
    s_silent_run             = 0;
}

void bq_link_supervisor_expect_por(void) { s_expect_por = true; }

/*
 * Whether the power path may close again after a successful reconfigure. Recovery
 * restores the configuration, nothing more; three standing conditions each mean
 * the FETs were open for a reason that survives the chip reset:
 *
 *   - a latched wire fault -- the FET chatter case: close, the still-open tap
 *     re-trips CUV in about three seconds, repeat on every retry interval
 *   - a software overcurrent trip, which the MCU owns and a reset leaves set
 *   - a manual override
 */
static bool fets_may_close(void)
{
    const wf_status_t *wf = wire_fault_status();
    const sw_oc_state_t *oc = sw_oc_protection_get_state();

    if (wf->state != WF_INACTIVE) {
        hal_uart_printf("[LINK] FET_EN withheld: wire fault latched (%s)\r\n",
                        (wf->state == WF_LIMP) ? "limp" : "service");
        return false;
    }
    if (oc->dsg_tripped || oc->chg_tripped) {
        hal_uart_printf("[LINK] FET_EN withheld: software overcurrent trip standing\r\n");
        return false;
    }
    if (bq_controller_is_manual_override()) {
        hal_uart_printf("[LINK] FET_EN withheld: manual override active\r\n");
        return false;
    }
    return true;
}

static void enter(link_state_t st, link_reason_t why)
{
    if (s_st.state == st) {
        return;
    }
    s_st.state = st;
    if (why != LINK_REASON_NONE) {
        s_st.last_reason = why;
    }
    s_st.attempt = 0;
    s_st.next_attempt_ms = systime_milli();

    hal_uart_printf("[LINK] state -> %s (%s)\r\n",
                    (st == LINK_OK)       ? "OK" :
                    (st == LINK_SILENT)   ? "SILENT" :
                    (st == LINK_RECONFIG) ? "RECONFIG" : "DEGRADED",
                    bq_link_reason_name(s_st.last_reason));
}

static void do_recover(void)
{
    if (bq_controller_reconfigure() != 0) {
        s_st.reconfig_fail_count++;
        s_st.attempt++;
        s_st.next_attempt_ms = systime_milli() + backoff_ms(s_st.attempt);
        hal_uart_printf("[LINK] reconfigure failed (%u), retry in %lu ms\r\n",
                        s_st.reconfig_fail_count,
                        (unsigned long)backoff_ms(s_st.attempt));
        enter(LINK_DEGRADED, LINK_REASON_NONE);
        return;
    }

    s_st.reconfig_ok_count++;
    s_st.last_recovery_ms = systime_milli();

    /* bq_controller_reconfigure() writes the parameter cache to the chip and the
     * limp profile lives in that cache, so an active wire fault's profile has just
     * been re-applied with everything else -- which is why recovery needs no
     * separate re-apply step, and why it must not then close the FETs. */
    if (fets_may_close()) {
        /* restore_autonomous() writes FETControl = 0x00 and then re-enables
         * FET_EN. Both are correct here -- a reset leaves FET_EN clear and
         * FETControl already 0x00 -- and this line is only reached when no
         * override is standing. */
        (void)bq_controller_restore_autonomous();
    } else {
        s_st.fet_en_withheld++;
    }

    enter(LINK_OK, LINK_REASON_NONE);
    hal_uart_printf("[LINK] recovered: por=%u silent=%u drift=%u ok=%u fail=%u withheld=%u\r\n",
                    s_st.por_count, s_st.silent_count, s_st.drift_count,
                    s_st.reconfig_ok_count, s_st.reconfig_fail_count,
                    s_st.fet_en_withheld);
}

bool bq_link_supervisor_tick(void)
{
    const bq_battery_status_t *bs = bq_controller_get_status();
    uint32_t now = systime_milli();
    bool por    = (bs->battery_status & BQ_BATT_STAT_POR) != 0u;
    bool silent = !bs->is_connected;
    bool drift  = (bs->config_verify_errors != 0u);

    /* An MCU-initiated POR is not a fault. Consume the flag on sight so a later,
     * genuine reset is not swallowed by a stale expectation. */
    if (por && s_expect_por) {
        s_expect_por = false;
        s_st.last_reason = LINK_REASON_EXPECTED;
        return s_st.state != LINK_OK;
    }

    /* Debounce the silence, not the POR: POR is a latched register bit and is
     * trustworthy on sight, whereas is_connected is inferred. */
    if (silent) {
        if (s_silent_run < 0xFFu) {
            s_silent_run++;
        }
    } else {
        s_silent_run = 0;
    }
    silent = (s_silent_run >= SILENT_CONFIRM_TICKS);

    switch (s_st.state) {
    case LINK_OK:
        if (silent) {
            s_st.silent_count++;
            enter(LINK_SILENT, LINK_REASON_SILENT);
        } else if (por) {
            /* The authoritative signal: set on any full reset, cleared only on
             * CONFIG_UPDATE exit -- which is what recovery does, so the flag
             * clears itself as a side effect of fixing the problem. */
            s_st.por_count++;
            enter(LINK_RECONFIG, LINK_REASON_POR);
        } else if (drift) {
            s_st.drift_count++;
            enter(LINK_RECONFIG, LINK_REASON_DRIFT);
        }
        break;

    case LINK_SILENT:
        if (!silent) {
            /* Answering again. A chip that was silent has almost certainly lost
             * its settings -- SHUTDOWN and brownout both clear data memory -- so
             * check POR rather than assuming it is fine. */
            enter(por ? LINK_RECONFIG : LINK_OK,
                  por ? LINK_REASON_POR : LINK_REASON_NONE);
            break;
        }
        if ((int32_t)(now - s_st.next_attempt_ms) < 0) {
            break;      /* waiting out the backoff */
        }
        /* One wake pulse and one probe per attempt, never a blocking retry loop:
         * this runs inside the controller tick, so no single pass may take long. */
        bq_controller_wake_pulse(30);
        s_st.attempt++;
        s_st.next_attempt_ms = now + backoff_ms(s_st.attempt);
        break;

    case LINK_RECONFIG:
        if (silent) {
            enter(LINK_SILENT, LINK_REASON_SILENT);
            break;
        }
        if ((int32_t)(now - s_st.next_attempt_ms) < 0) {
            break;
        }
        do_recover();
        break;

    case LINK_DEGRADED:
        if (silent) {
            enter(LINK_SILENT, LINK_REASON_SILENT);
            break;
        }
        if ((int32_t)(now - s_st.next_attempt_ms) < 0) {
            break;
        }
        do_recover();       /* keeps trying forever at the slow rate */
        break;
    }

    return s_st.state != LINK_OK;
}

const link_sup_status_t *bq_link_supervisor_status(void) { return &s_st; }

const char *bq_link_reason_name(link_reason_t r)
{
    switch (r) {
    case LINK_REASON_NONE:     return "none";
    case LINK_REASON_POR:      return "POR";
    case LINK_REASON_SILENT:   return "silent";
    case LINK_REASON_DRIFT:    return "config drift";
    case LINK_REASON_EXPECTED: return "expected (we reset it)";
    default:                   return "?";
    }
}
