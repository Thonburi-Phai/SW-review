#pragma once

#include <stdint.h>
#include <stdbool.h>

/* Tick interval in ms — must match the BQ measurement cycle that calls
 * sw_oc_protection_update().  All trip_count values are derived from
 * this constant so changing it automatically rescales every channel. */
#define SW_OC_TICK_MS  600

typedef struct {
    /* Slow channels (DSG + CHG, long-duration sustained overcurrent) */
    int32_t  dsg_threshold_ma;
    int32_t  chg_threshold_ma;
    uint16_t dsg_trip_count;
    uint16_t chg_trip_count;
    uint8_t  decay_rate;
    /* Fast channel 1 (DSG only — e.g. 600A / 10s) */
    int32_t  fast1_threshold_ma;
    uint16_t fast1_trip_count;
    uint8_t  fast1_decay_rate;
    /* Fast channel 2 (DSG only — e.g. 800A / 5s) */
    int32_t  fast2_threshold_ma;
    uint16_t fast2_trip_count;
    uint8_t  fast2_decay_rate;
} sw_oc_config_t;

typedef struct {
    uint16_t dsg_counter;
    uint16_t chg_counter;
    uint8_t  dsg_tripped;
    uint8_t  chg_tripped;
    uint16_t fast1_counter;
    uint8_t  fast1_tripped;
    uint16_t fast2_counter;
    uint8_t  fast2_tripped;
} sw_oc_state_t;

void                    sw_oc_protection_init(void);
void                    sw_oc_protection_update(int32_t raw_current_ma);
const sw_oc_state_t*    sw_oc_protection_get_state(void);
const sw_oc_config_t*   sw_oc_protection_get_config(void);
void                    sw_oc_protection_clear(void);
bool                    sw_oc_protection_is_tripped(void);
