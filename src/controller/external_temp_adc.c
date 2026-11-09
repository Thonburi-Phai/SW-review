/**
 * @file external_temp_adc.c
 * @brief External NTC sensors: ADC read, NTC conversion, open-wire check.
 */

#include "external_temp_adc.h"
#include "board_pins.h"
#include "bq_config_4s_lfp.h"
#include "hal_adc.h"
#include "hal_uart.h"
#include "hal_systime.h"
#include <stddef.h>

/* ========== Private Variables ========== */

static ext_temp_module_status_t g_ext_temp_status = {0};
/* NTC_CH1..4 -> ADC0 channel numbers, from board_pins.h. V2 replaces SE9 (PTC1)
 * with SE12 (PTC14); the other three keep their channel numbers. */
static uint8_t g_adc_channels[EXT_TEMP_SENSOR_COUNT] = BRD_ADC_NTC_CHANNELS;
static bool g_module_initialized = false;

/* Open wire detection state */
static int16_t g_prev_temp_c[EXT_TEMP_SENSOR_COUNT] = {0};
static bool    g_prev_temp_valid[EXT_TEMP_SENSOR_COUNT] = {false};
static uint8_t g_ow_debounce[EXT_TEMP_SENSOR_COUNT] = {0};
static uint8_t g_ow_recovery[EXT_TEMP_SENSOR_COUNT] = {0};

/* ========== Private Function Prototypes ========== */

/**
 * @brief Convert a raw ADC value to temperature.
 * @param adc_raw Raw ADC value (0-4095)
 * @param temp_c Output: temperature in °C, EXT_TEMP_INVALID_C if disconnected
 * @return 0 on success, -1 if the reading is outside the convertible band
 */
static int adc_to_temperature(uint16_t adc_raw, int16_t *temp_c);

/** @brief Open-wire check for one channel: range plus rate-of-change. */
static void check_open_wire(int sensor_idx);

/* ========== Public Function Implementations ========== */

int ext_temp_adc_init(void)
{
    hal_uart_printf("\r\n=== External Temp ADC Init ===\r\n");

    for (int i = 0; i < EXT_TEMP_SENSOR_COUNT; i++) {
        g_ext_temp_status.sensors[i].temp_c = EXT_TEMP_INVALID_C;
        g_ext_temp_status.sensors[i].adc_raw = 0;
        g_ext_temp_status.sensors[i].is_valid = false;
        g_ext_temp_status.sensors[i].open_wire = false;
        g_ext_temp_status.sensors[i].last_update_ms = 0;

        g_prev_temp_c[i] = 0;
        g_prev_temp_valid[i] = false;
        g_ow_debounce[i] = 0;
        g_ow_recovery[i] = 0;
    }

    g_ext_temp_status.mosfet_disabled = false;
    g_ext_temp_status.fault_mask = 0;
    g_ext_temp_status.open_wire_mask = 0;
    g_ext_temp_status.valid_mask = 0;
    g_ext_temp_status.usable_mask = 0;

    g_module_initialized = true;

    hal_uart_printf("[OK] External Temp ADC initialized (%d sensors)\r\n",
                      EXT_TEMP_SENSOR_COUNT);
    for (int i = 0; i < EXT_TEMP_SENSOR_COUNT; i++) {
        hal_uart_printf("  NTC_CH%d: ADC Ch%d (%s)\r\n", i + 1, g_adc_channels[i],
                          ((1u << i) & EXT_TEMP_BATTERY_MASK) ? "battery" : "MOSFET");
    }
    hal_uart_printf("=========================\r\n\r\n");

    return 0;
}

int ext_temp_adc_update(void)
{
    if (!g_module_initialized) {
        return -1;
    }

    uint32_t now = systime_milli();

    for (int i = 0; i < EXT_TEMP_SENSOR_COUNT; i++) {
        uint16_t adc_raw = hal_adc_read_channel(g_adc_channels[i]);
        int16_t temp_c = 0;

        g_ext_temp_status.sensors[i].adc_raw = adc_raw;

        if (adc_to_temperature(adc_raw, &temp_c) != 0) {
            g_ext_temp_status.sensors[i].is_valid = false;
            g_ext_temp_status.sensors[i].temp_c = EXT_TEMP_INVALID_C;
        } else {
            g_ext_temp_status.sensors[i].temp_c = temp_c;
            g_ext_temp_status.sensors[i].is_valid = true;
            g_ext_temp_status.sensors[i].last_update_ms = now;
        }

        check_open_wire(i);
    }

    /* Recompute the masks the decision layer reads. */
    g_ext_temp_status.valid_mask = 0;
    for (int i = 0; i < EXT_TEMP_SENSOR_COUNT; i++) {
        if (g_ext_temp_status.sensors[i].is_valid) {
            g_ext_temp_status.valid_mask |= (uint8_t)(1u << i);
        }
    }
    g_ext_temp_status.usable_mask =
        (uint8_t)(g_ext_temp_status.valid_mask & (uint8_t)~g_ext_temp_status.open_wire_mask);

    return 0;
}

const ext_temp_module_status_t* ext_temp_adc_get_status(void)
{
    return &g_ext_temp_status;
}

const ext_temp_sensor_status_t* ext_temp_adc_get_sensor_status(ext_temp_sensor_id_t sensor_id)
{
    if (sensor_id >= EXT_TEMP_SENSOR_COUNT) {
        return NULL;
    }
    return &g_ext_temp_status.sensors[sensor_id];
}

bool ext_temp_adc_has_fault(void)
{
    return (g_ext_temp_status.fault_mask != 0);
}

void ext_temp_adc_set_fault_state(uint8_t fault_mask, bool mosfet_disabled)
{
    g_ext_temp_status.fault_mask = fault_mask;
    g_ext_temp_status.mosfet_disabled = mosfet_disabled;
}

int ext_temp_adc_clear_faults(void)
{
    hal_uart_printf("[INFO] Clearing external NTC open-wire latches...\r\n");

    for (int i = 0; i < EXT_TEMP_SENSOR_COUNT; i++) {
        g_ext_temp_status.sensors[i].open_wire = false;
        g_ow_debounce[i] = 0;
        g_ow_recovery[i] = 0;
    }
    g_ext_temp_status.open_wire_mask = 0;
    g_ext_temp_status.usable_mask = g_ext_temp_status.valid_mask;

    /* The latches are gone; whether that is enough depends on the readings,
     * which the next update refreshes. Report on what is in range now. */
    if ((g_ext_temp_status.valid_mask & EXT_TEMP_BATTERY_MASK) == 0u) {
        hal_uart_printf("[WARN] No battery NTC channel reads in range\r\n");
        return -1;
    }

    hal_uart_printf("[OK] External NTC latches cleared\r\n");
    return 0;
}

void ext_temp_adc_print_status(void)
{
    if (!g_module_initialized) {
        hal_uart_printf("[ERROR] External Temp ADC not initialized\r\n");
        return;
    }

    hal_uart_printf("\r\n=== External Temperature Status ===\r\n");

    for (int i = 0; i < EXT_TEMP_SENSOR_COUNT; i++) {
        const ext_temp_sensor_status_t *s = &g_ext_temp_status.sensors[i];

        hal_uart_printf("NTC_CH%d (ADC Ch%d, %s): Temp %dC, ADC %u, %s%s\r\n",
                          i + 1, g_adc_channels[i],
                          ((1u << i) & EXT_TEMP_BATTERY_MASK) ? "battery" : "MOSFET",
                          s->temp_c, s->adc_raw,
                          s->is_valid ? "valid" : "INVALID",
                          s->open_wire ? ", OPEN WIRE" : "");
    }

    hal_uart_printf("Valid 0x%02X, usable 0x%02X, open 0x%02X, band fault 0x%02X\r\n",
                      g_ext_temp_status.valid_mask,
                      g_ext_temp_status.usable_mask,
                      g_ext_temp_status.open_wire_mask,
                      g_ext_temp_status.fault_mask);
    hal_uart_printf("FET held open by NTC layer: %s\r\n",
                      g_ext_temp_status.mosfet_disabled ? "YES" : "NO");
    hal_uart_printf("===============================\r\n\r\n");
}

void ext_temp_adc_print_fault_details(void)
{
    bool any = false;

    for (int i = 0; i < EXT_TEMP_SENSOR_COUNT; i++) {
        const ext_temp_sensor_status_t *s = &g_ext_temp_status.sensors[i];
        uint8_t bit = (uint8_t)(1u << i);

        if (s->is_valid && !s->open_wire &&
            ((g_ext_temp_status.fault_mask & bit) == 0u)) {
            continue;
        }
        any = true;

        hal_uart_printf("[NTC_CH%d] ADC Ch%d raw %u, temp %dC%s%s%s\r\n",
                          i + 1, g_adc_channels[i], s->adc_raw, s->temp_c,
                          s->is_valid ? "" : ", reading outside the convertible band",
                          s->open_wire ? ", OPEN WIRE confirmed" : "",
                          (g_ext_temp_status.fault_mask & bit) ? ", band blocks a FET" : "");
    }

    if (!any) {
        hal_uart_printf("[INFO] No external temperature faults detected\r\n");
    }
}

int ext_temp_adc_set_channel(ext_temp_sensor_id_t sensor_id, uint8_t adc_channel)
{
    if (sensor_id >= EXT_TEMP_SENSOR_COUNT) {
        return -1;
    }

    g_adc_channels[sensor_id] = adc_channel;
    return 0;
}

/* ========== Private Function Implementations ========== */

static void check_open_wire(int i)
{
    ext_temp_sensor_status_t *s = &g_ext_temp_status.sensors[i];
    bool suspect = false;

    /* Range: reading already outside the convertible band (ADC at a rail) */
    if (!s->is_valid) {
        suspect = true;
    }

    /* Rate-of-change: delta above EXT_TEMP_OW_RATE_MAX_C per cycle */
    if (!suspect && g_prev_temp_valid[i]) {
        int16_t delta = (int16_t)(s->temp_c - g_prev_temp_c[i]);
        if (delta < 0) delta = (int16_t)(-delta);
        if (delta > EXT_TEMP_OW_RATE_MAX_C) {
            suspect = true;
        }
    }

    if (suspect) {
        if (g_ow_debounce[i] < EXT_TEMP_OW_DEBOUNCE_COUNT) {
            g_ow_debounce[i]++;
        }
        if (g_ow_debounce[i] >= EXT_TEMP_OW_DEBOUNCE_COUNT && !s->open_wire) {
            s->open_wire = true;
            g_ext_temp_status.open_wire_mask |= (uint8_t)(1u << i);
            hal_uart_printf("[OPEN WIRE] NTC_CH%d confirmed (ADC=%u)\r\n",
                              i + 1, s->adc_raw);
        }
        g_ow_recovery[i] = 0;
    } else {
        if (s->open_wire) {
            if (g_ow_recovery[i] < EXT_TEMP_OW_RECOVERY_COUNT) {
                g_ow_recovery[i]++;
            }
            if (g_ow_recovery[i] >= EXT_TEMP_OW_RECOVERY_COUNT) {
                s->open_wire = false;
                g_ext_temp_status.open_wire_mask &= (uint8_t)~(1u << i);
                g_ow_debounce[i] = 0;
                g_ow_recovery[i] = 0;
                hal_uart_printf("[OPEN WIRE] NTC_CH%d recovered\r\n", i + 1);
            }
        } else {
            g_ow_debounce[i] = 0;
        }
    }

    /* Store for next cycle */
    g_prev_temp_c[i] = s->temp_c;
    g_prev_temp_valid[i] = s->is_valid;
}

static int adc_to_temperature(uint16_t adc_raw, int16_t *temp_c)
{
    if (!temp_c) {
        return -1;
    }

    if (adc_raw < EXT_TEMP_ADC_DISCONNECT_LOW || adc_raw > EXT_TEMP_ADC_DISCONNECT_HIGH) {
        *temp_c = EXT_TEMP_INVALID_C;
        return -1;
    }

    // V_adc(mV) = adc_raw * V_ref(mV) / ADC_resolution
    uint32_t v_adc_mv = ((uint32_t)adc_raw * EXT_TEMP_ADC_VREF_MV) / EXT_TEMP_ADC_RESOLUTION;

    // Voltage divider: R_NTC = R_pullup × V_adc / (V_ref - V_adc)
    if (v_adc_mv >= EXT_TEMP_ADC_VREF_MV) {
        *temp_c = EXT_TEMP_INVALID_C;
        return -1;
    }

    uint32_t r_ntc_ohm = (EXT_TEMP_PULLUP_R_OHM * v_adc_mv) / (EXT_TEMP_ADC_VREF_MV - v_adc_mv);

    // Resistance to °C via LUT with linear interpolation
    *temp_c = ntc_resistance_to_temp((int32_t)r_ntc_ohm);
    return 0;
}
