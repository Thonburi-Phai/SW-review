/**
 * @file external_temp_adc.h
 * @brief External NTC sensors on the MCU ADC: read, convert, and classify
 *        open wires.
 *
 * Measurement only. What a temperature means -- the band ladders, the channel
 * roles, the TS1 fallback and the FET requests -- is ntc_fault.c.
 */

#pragma once
#include <stdint.h>
#include <stdbool.h>

/* ========== Configuration ========== */

/** @brief External temperature sensor indices */
typedef enum {
    EXT_TEMP_SENSOR_1 = 0,  // NTC_CH1: battery temperature
    EXT_TEMP_SENSOR_2 = 1,  // NTC_CH2: battery temperature
    EXT_TEMP_SENSOR_3 = 2,  // NTC_CH3: battery temperature
    EXT_TEMP_SENSOR_4 = 3,  // NTC_CH4: MOSFET temperature
    EXT_TEMP_SENSOR_COUNT = 4
} ext_temp_sensor_id_t;

/** Channel roles: CH1-3 watch the cells, CH4 watches the MOSFETs. */
#define EXT_TEMP_BATTERY_MASK   0x07u
#define EXT_TEMP_MOSFET_MASK    0x08u

/** Temperature reported for a channel that cannot be converted. */
#define EXT_TEMP_INVALID_C      (-999)

/** @brief Per-sensor status */
typedef struct {
    int16_t  temp_c;            // Temperature (°C), EXT_TEMP_INVALID_C if invalid
    uint16_t adc_raw;           // Raw ADC value (0-4095 for 12-bit ADC)
    bool     is_valid;          // Reading inside the convertible band
    bool     open_wire;         // Confirmed open wire (debounced)
    uint32_t last_update_ms;    // Last update timestamp
} ext_temp_sensor_status_t;

/** @brief Module status */
typedef struct {
    ext_temp_sensor_status_t sensors[EXT_TEMP_SENSOR_COUNT];
    bool    mosfet_disabled;    // ntc_fault.c is holding a FET open
    uint8_t fault_mask;         // Channels whose band currently blocks a FET
    uint8_t open_wire_mask;     // Confirmed open wire (bit 0=CH1 .. bit 3=CH4)
    uint8_t valid_mask;         // Channels with a convertible reading
    uint8_t usable_mask;        // valid and not open-wire
} ext_temp_module_status_t;

/* ========== Conversion constants ========== */

#define EXT_TEMP_ADC_RESOLUTION        (4096)   // 12-bit ADC: 0-4095
#define EXT_TEMP_ADC_VREF_MV           (3300)   // ADC reference voltage (mV)

// NTC thermistor: 10kΩ @ 25°C, B=3950K, 10kΩ pull-up divider
#define EXT_TEMP_NTC_R0_OHM            (10000)  // Resistance at 25°C (ohm)
#define EXT_TEMP_NTC_T0_K              (298.15) // 25°C in Kelvin
#define EXT_TEMP_NTC_B_VALUE           (3950)   // Beta value (K)
#define EXT_TEMP_PULLUP_R_OHM          (10000)  // Pull-up resistor (ohm)

/* Disconnect detection: raw ADC counts outside [LOW, HIGH] are invalid.
 *
 * The NTC sits low-side under a pull-up, so an open wire rails the channel high
 * and a short rails it low. Converted through the divider and NTC_LUT these two
 * limits put the convertible band at roughly -40..+125 °C, which is what makes
 * "ignore the open channel and judge on the rest" safe: an open channel cannot
 * present itself as a valid cold reading. The same limit means a sensor colder
 * than about -40 °C is indistinguishable from an open one. */
#define EXT_TEMP_ADC_DISCONNECT_LOW    (100)    // ADC counts
#define EXT_TEMP_ADC_DISCONNECT_HIGH   (3900)   // ADC counts

// Open wire detection
#define EXT_TEMP_OW_DEBOUNCE_COUNT     (5)      // Consecutive detections to confirm
#define EXT_TEMP_OW_RECOVERY_COUNT     (10)     // Consecutive clean readings to auto-clear
#define EXT_TEMP_OW_RATE_MAX_C         (5)      // Max plausible delta per 600ms cycle (°C)

/* ========== Public Functions ========== */

/** @brief Clear status and report the channel map. @return 0 on success */
int ext_temp_adc_init(void);

/**
 * @brief Periodic update: read each ADC channel, convert to °C, and run the
 *        open-wire check.
 * @return 0 on success, -1 if the module is not initialised
 */
int ext_temp_adc_update(void);

/** @brief Module status for all sensors. */
const ext_temp_module_status_t* ext_temp_adc_get_status(void);

/**
 * @brief One sensor's status.
 * @return Pointer to sensor status, or NULL if sensor_id is invalid
 */
const ext_temp_sensor_status_t* ext_temp_adc_get_sensor_status(ext_temp_sensor_id_t sensor_id);

/** @brief true if any channel's band currently blocks a FET. */
bool ext_temp_adc_has_fault(void);

/**
 * @brief Publish ntc_fault.c's verdict for telemetry and the health report.
 * @param fault_mask Channels whose band blocks a FET
 * @param mosfet_disabled true while this layer holds a FET open
 */
void ext_temp_adc_set_fault_state(uint8_t fault_mask, bool mosfet_disabled);

/**
 * @brief Drop the open-wire latches so a repaired channel is re-admitted.
 *
 * Band faults are not cleared here: a band follows the temperature, so there is
 * nothing latched to clear. Only the channels that are not currently open-wire
 * have to read in range; requiring all four would let a single unplugged NTC
 * block recovery for the rest of the power cycle.
 *
 * @return 0 if at least one battery channel is usable afterwards, else -1
 */
int ext_temp_adc_clear_faults(void);

/** @brief Print per-channel readings and open-wire status via UART. */
void ext_temp_adc_print_status(void);

/** @brief Print detail for every channel that is open or out of range. */
void ext_temp_adc_print_fault_details(void);

/**
 * @brief Map a sensor to an ADC channel. Call before ext_temp_adc_init() to
 *        override the defaults.
 * @return 0 on success, -1 on error
 */
int ext_temp_adc_set_channel(ext_temp_sensor_id_t sensor_id, uint8_t adc_channel);
