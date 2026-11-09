/**
 * @file debug_report.h
 * @brief Periodic reporting of BMS sensor data over UART, as CSV text or
 *        binary protocol binary packets.
 *
 * @author Ming
 */

#ifndef DEBUG_REPORT_H
#define DEBUG_REPORT_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>
#include <stdbool.h>

/*---------------------------------------------------------------------------*/
/* Debug Output Types                                                        */
/*---------------------------------------------------------------------------*/

/** @brief CSV text format for serial plotter/terminal */
#define DEBUG_TYPE_CSV 0

/** @brief binary protocol binary format for Flutter app */
#define DEBUG_TYPE_BP 1

/*---------------------------------------------------------------------------*/
/* Configuration                                                             */
/*---------------------------------------------------------------------------*/

/** @brief Default update interval in milliseconds */
#define DEBUG_REPORT_DEFAULT_INTERVAL_MS 100

/** @brief Minimum update interval (prevent flooding UART) */
#define DEBUG_REPORT_MIN_INTERVAL_MS 50

/** @brief Maximum update interval */
#define DEBUG_REPORT_MAX_INTERVAL_MS 10000

    /*---------------------------------------------------------------------------*/
    /* Public Functions                                                          */
    /*---------------------------------------------------------------------------*/

    /**
     * @brief Initialize debug report module: default interval, reporting off.
     */
    void debug_report_init(void);

    /**
     * @brief Enable periodic debug output at the configured interval.
     */
    void debug_report_enable(void);

    /**
     * @brief Disable periodic debug output.
     */
    void debug_report_disable(void);

    /**
     * @brief Toggle debug reporting on/off.
     * @return true if reporting is now enabled, false if disabled.
     */
    bool debug_report_toggle(void);

    /**
     * @brief Check if debug reporting is enabled.
     * @return true if enabled, false otherwise.
     */
    bool debug_report_is_enabled(void);

    /**
     * @brief Set debug output type.
     * @param type DEBUG_TYPE_CSV or DEBUG_TYPE_BP.
     */
    void debug_report_set_type(uint8_t type);

    /**
     * @brief Get current debug output type.
     * @return DEBUG_TYPE_CSV or DEBUG_TYPE_BP.
     */
    uint8_t debug_report_get_type(void);

    /**
     * @brief Set update interval.
     * @param interval_ms Update interval in milliseconds (50-10000).
     * @return 0 on success, -1 if interval out of range.
     */
    int debug_report_set_interval(uint32_t interval_ms);

    /**
     * @brief Get current update interval.
     * @return Current interval in milliseconds.
     */
    uint32_t debug_report_get_interval(void);

    /**
     * @brief Run debug report task. Call periodically from the main loop; emits
     *        a report when enabled and the interval has elapsed.
     */
    void debug_report_run(void);

    /**
     * @brief Print one full debug report immediately, whatever the enable state.
     */
    void debug_report_print_once(void);

#ifdef __cplusplus
}
#endif

#endif /* DEBUG_REPORT_H */
