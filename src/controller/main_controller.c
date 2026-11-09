/**
 * @file main_controller.c
 * @brief Main application controller implementation.
 *
 * Top-level controller coordinating battery monitoring, temperature sensing,
 * SOC/SOH estimation, fault handling and UART command processing.
 */

#include "device_registers.h"   /* SCB AIRCR, for the software reset into the bootloader */
#include "bl_compat.h"
#include "main_controller.h"
#include "bq_link_supervisor.h"
#include "hal_uart.h"
#include "hal_uart1.h"
#include "hal_gpio.h"
#include "hal_systime.h"
#include "hal_i2c.h"
#include "hal_crc.h"
#include "hal_adc.h"

#include "tripple_uart_cmd.h"

#include "bq_controller.h"
#include "bq7690x.h"
#include "bq7690x_status.h"
#include "external_temp_adc.h"
#include "ntc_fault.h"
#include "battery_soc_soh.h"
#include "system_health.h"
#include "debug_report.h"
#include "link_protocol.h"
#include "parameter.h"
#include "param_cmd_handler.h"
#include "sw_oc_protection.h"

/*---------------------------------------------------------------------------*/
/* binary protocol Command IDs (for BMS_COMMAND packet)                              */
/*---------------------------------------------------------------------------*/

#define BPCMD_SET_CHG_FET          0x01
#define BPCMD_SET_DSG_FET          0x02
#define BPCMD_SET_BOTH_FETS        0x03
#define BPCMD_SET_BALANCING        0x04
#define BPCMD_CLEAR_FAULTS         0x05
#define BPCMD_ENTER_SLEEP          0x06
#define BPCMD_ENTER_SHUTDOWN       0x07
#define BPCMD_WAKE_PULSE           0x08
#define BPCMD_RESET_PASSQ          0x09
#define BPCMD_SET_DEBUG_INTERVAL   0x0A
#define BPCMD_SET_DEBUG_TYPE       0x0B
#define BPCMD_TOGGLE_DEBUG         0x0C
#define BPCMD_REQUEST_STATUS       0x0D
#define BPCMD_REQUEST_FAULTS       0x0E
#define BPCMD_REQUEST_CALIBRATION  0x0F
#define BPCMD_REQUEST_FET_DIAG     0x10
#define BPCMD_REQUEST_HEALTH       0x11
#define BPCMD_SET_SOC              0x12
#define BPCMD_SINGLE_REPORT        0x13
#define BPCMD_FET_SHUTDOWN         0x14
#define BPCMD_FET_RESTORE          0x15
#define BPCMD_FET_RESET            0x16
#define BPCMD_ENTER_BOOTLOADER     0x17
#define BPCMD_SET_UART_MODE        0x18


/* Handing the board to the bootloader stops every firmware-level function --
 * balancing, SOC, the software overcurrent limits, the FET policy -- until an
 * image is programmed and it jumps back; the pack is unmanaged meanwhile. The
 * chip-side protections stay live throughout (OV, UV, OCD, SCD are autonomous,
 * and the host watchdog is disabled, so the FETs are left as they are).
 *
 * The command is guarded twice: the parameter must carry a confirmation value,
 * and the command is refused while current is flowing. */
#define BPCMD_BOOTLOADER_CONFIRM   0xB1
#define BPCMD_BOOTLOADER_MAX_MA    1000

/* BL_ENTRY_MAGIC_ADDR and BL_ENTRY_MAGIC come from bl_compat.h, which also checks
 * them against what the bootloader publishes at run time. */

/*---------------------------------------------------------------------------*/
/* Private Variables                                                         */
/*---------------------------------------------------------------------------*/

/** @brief Last received command from UART interface */
CommandType command_received = CMD_NONE;

/** @brief Heartbeat LED toggle timestamp */
static uint32_t heartbeat_tick __attribute__((unused)) = 0;

/** @brief binary protocol RX parser state (accessed from USART1 IRQ) */
link_packet_t g_bp_rx_pkt;
link_monitor_t g_bp_rx_mon;
volatile uint8_t g_bp_rx_ready = 0;

/** @brief TX buffer for binary protocol command responses */
static uint8_t cmd_resp_tx_buf[64];

/*---------------------------------------------------------------------------*/
/* Forward Declarations                                                      */
/*---------------------------------------------------------------------------*/

static void controller_handle_command(CommandType cmd);
static void handle_bp_command(link_packet_t *pkt);
static void send_cmd_response(uint8_t cmd_id, uint8_t result, int32_t value);

/*---------------------------------------------------------------------------*/
/* Public Functions                                                          */
/*---------------------------------------------------------------------------*/

void controller_init(void)
{
    /* NOTE: hal_systime_init(), hal_uart_init(), hal_gpio_init(), hal_i2c_init(),
     * hal_adc_init(), hal_crc_init() are already called from main.c */

    /* Initialize parameter cache with defaults */
    param_init();

    /* Initialize command parser */
    CMD_Init();

    /* Initialize BQ7690x battery management controller */
    /* Report which bootloader started this image, or that none did */
    bl_compat_print();

    bq_controller_init();

    /* Initialize external temperature ADC sensors */
    ext_temp_adc_init();
    ntc_fault_init();

    /* Initialize SOC/SOH module (auto-detect initial SOC from voltage) */
    battery_soc_soh_init(NULL, -1);

    /* Initialize software overcurrent protection */
    sw_oc_protection_init();

    /* Initialize debug report module -- defaults to binary protocol framing, not CSV */
    debug_report_init();

    /*
     * Hand the single UART to the binary protocol: from here on the wire carries
     * frames and console text would be garbage to the app's parser.
     * BPCMD_SET_UART_MODE borrows it back on request, with a deadline.
     */
    hal_uart_puts("[UART] handing the line to the binary protocol; console off\r\n");
    hal_uart_console_disable();
}

/* Debug: track LPUART1 RX bytes per wake cycle */
static uint32_t g_uart1_rx_total = 0;
static uint32_t g_uart1_rx_cmds = 0;
static uint32_t g_uart1_crc_err = 0;

/* ISR counters from hal_uart1.c */
extern volatile uint32_t g_uart1_isr_count;
extern volatile uint32_t g_uart1_isr_rx_count;
extern volatile uint32_t g_uart1_isr_err_count;

void controller_poll_uart(void)
{
    /* Feed debug command parser from LPUART0 RX buffer */
    while (hal_uart_rx_available()) {
        CMD_ProcessChar((char)hal_uart_rx_getc());
    }

    /* Feed binary protocol parser from LPUART1 RX buffer */
    uint16_t rx_count = 0;
    while (hal_uart1_rx_available()) {
        uint8_t byte = hal_uart1_rx_getc();
        rx_count++;
        g_uart1_rx_total++;

        uint16_t prev_err = g_bp_rx_mon.received_error_packes;
        if (link_uint8_parser(byte, &g_bp_rx_pkt, &g_bp_rx_mon)) {
            g_bp_rx_ready = 1;
            g_uart1_rx_cmds++;
            hal_uart_printf("[RX1] PKT op=0x%02X id=%d len=%d\r\n",
                              g_bp_rx_pkt.op, g_bp_rx_pkt.packet_id,
                              g_bp_rx_pkt.len);
        }
        if (g_bp_rx_mon.received_error_packes != prev_err) {
            g_uart1_crc_err++;
            hal_uart_printf("[RX1] CRC_ERR #%lu\r\n", (unsigned long)g_uart1_crc_err);
        }
    }
    if (rx_count > 0) {
        hal_uart_printf("[RX1] %d bytes total=%lu cmds=%lu err=%lu ISR=%lu/%lu/%lu\r\n",
                          rx_count,
                          (unsigned long)g_uart1_rx_total,
                          (unsigned long)g_uart1_rx_cmds,
                          (unsigned long)g_uart1_crc_err,
                          (unsigned long)g_uart1_isr_count,
                          (unsigned long)g_uart1_isr_rx_count,
                          (unsigned long)g_uart1_isr_err_count);
    }

    /* Process binary protocol commands */
    if (g_bp_rx_ready) {
        handle_bp_command(&g_bp_rx_pkt);
        g_bp_rx_ready = 0;
    }

    /* Process debug UART commands */
    if (CMD_IsAvailable() != 0) {
        command_received = CMD_Execute();
        if (command_received != CMD_NONE) {
            controller_handle_command(command_received);
            command_received = CMD_NONE;
        }
    }
}

static void controller_handle_command(CommandType cmd)
{
    switch (cmd) {
                case CMD_SSS:
                    /* Print all status information */
                    bq_controller_print_status();
                    ext_temp_adc_print_status();
                    ntc_fault_print_status();
                    battery_soc_soh_print_status();
                    break;

                case CMD_FFF:
                    /* Print fault details */
                    if (bq_controller_has_fault()) {
                        bq_controller_print_fault_details();
                    } else {
                        hal_uart_printf("[OK] No BQ faults active\r\n");
                    }
                    if (ext_temp_adc_has_fault()) {
                        ext_temp_adc_print_fault_details();
                    } else {
                        hal_uart_printf("[OK] No external temp faults active\r\n");
                    }
                    break;

                case CMD_DDD:
                    /* Clear all faults */
                    hal_uart_printf("Clearing faults...\r\n");
                    if (bq_controller_clear_faults() == 0) {
                        hal_uart_printf("[OK] BQ faults cleared\r\n");
                    } else {
                        hal_uart_printf("[ERROR] Failed to clear BQ faults\r\n");
                    }
                    if (ext_temp_adc_clear_faults() == 0) {
                        hal_uart_printf("[OK] External temp faults cleared\r\n");
                    } else {
                        hal_uart_printf("[ERROR] Failed to clear external temp faults\r\n");
                    }
                    break;

                case CMD_AAA:
                    /* Enter sleep mode */
                    hal_uart_printf("Entering sleep mode...\r\n");
                    bq_controller_enter_sleep();
                    break;

                case CMD_EEE:
                    /* Enter shutdown mode */
                    hal_uart_printf("Entering shutdown mode...\r\n");
                    bq_controller_enter_shutdown();
                    break;

                case CMD_WWW:
                    /* Send wake pulse via TS pin (push-pull HIGH through diode) */
                    hal_uart_printf("Sending wake pulse...\r\n");
                    bq_controller_wake_pulse(30);
                    hal_uart_printf("[OK] Wake pulse sent\r\n");
                    break;

                case CMD_OOO:
                    /* Restore autonomous FET control (FET_EN toggle ON) */
                    hal_uart_printf("[CMD] Restoring autonomous FET control...\r\n");
                    bq_controller_print_fet_diag();
                    if (bq_controller_restore_autonomous() == 0) {
                        hal_uart_printf("[OK] Autonomous FET control restored\r\n");
                    } else {
                        hal_uart_printf("[ERROR] Failed to restore autonomous control\r\n");
                    }
                    bq_controller_print_fet_diag();
                    break;

                case CMD_XXX:
                    /* Safe FET shutdown (FET_EN toggle OFF) */
                    hal_uart_printf("[CMD] Manual FET shutdown...\r\n");
                    bq_controller_print_fet_diag();
                    if (bq_controller_manual_shutdown() == 0) {
                        hal_uart_printf("[OK] FETs disabled (FET_EN OFF)\r\n");
                    } else {
                        hal_uart_printf("[ERROR] Failed to disable FETs\r\n");
                    }
                    bq_controller_print_fet_diag();
                    break;

                case CMD_CCC:
                    /* Turn ON CHG FET only */
                    hal_uart_printf("[CMD] Turning ON CHG FET...\r\n");
                    hal_uart_printf("[WARN] Host FET override - autonomous OCD/OCC protection disabled until reboot\r\n");
                    if (bq_controller_set_fets(true, false) == 0) {
                        hal_uart_printf("[OK] CHG FET: ON, DSG FET: OFF\r\n");
                    } else {
                        hal_uart_printf("[ERROR] Failed to control FETs\r\n");
                    }
                    break;

                case CMD_III:
                    /* Turn OFF CHG FET only (DSG stays ON = host FET write) */
                    hal_uart_printf("[CMD] Turning OFF CHG FET...\r\n");
                    hal_uart_printf("[WARN] Host FET override - autonomous OCD/OCC protection disabled until reboot\r\n");
                    if (bq_controller_set_fets(false, true) == 0) {
                        hal_uart_printf("[OK] CHG FET: OFF, DSG FET: ON\r\n");
                    } else {
                        hal_uart_printf("[ERROR] Failed to control FETs\r\n");
                    }
                    break;

                case CMD_GGG:
                    /* Turn ON DSG FET only */
                    hal_uart_printf("[CMD] Turning ON DSG FET...\r\n");
                    hal_uart_printf("[WARN] Host FET override - autonomous OCD/OCC protection disabled until reboot\r\n");
                    if (bq_controller_set_fets(false, true) == 0) {
                        hal_uart_printf("[OK] CHG FET: OFF, DSG FET: ON\r\n");
                    } else {
                        hal_uart_printf("[ERROR] Failed to control FETs\r\n");
                    }
                    break;

                case CMD_JJJ:
                    /* Turn OFF DSG FET only (CHG stays ON = host FET write) */
                    hal_uart_printf("[CMD] Turning OFF DSG FET...\r\n");
                    hal_uart_printf("[WARN] Host FET override - autonomous OCD/OCC protection disabled until reboot\r\n");
                    if (bq_controller_set_fets(true, false) == 0) {
                        hal_uart_printf("[OK] CHG FET: ON, DSG FET: OFF\r\n");
                    } else {
                        hal_uart_printf("[ERROR] Failed to control FETs\r\n");
                    }
                    break;

                case CMD_BBB:
                    /* Enable cell balancing */
                    hal_uart_printf("[CMD] Enabling cell balancing...\r\n");
                    if (bq_controller_set_balancing_enable(true) == 0) {
                        hal_uart_printf("[OK] Cell balancing ENABLED\r\n");
                    } else {
                        hal_uart_printf("[ERROR] Failed to enable balancing\r\n");
                    }
                    break;

                case CMD_NNN:
                    /* Disable cell balancing */
                    hal_uart_printf("[CMD] Disabling cell balancing...\r\n");
                    if (bq_controller_set_balancing_enable(false) == 0) {
                        hal_uart_printf("[OK] Cell balancing DISABLED\r\n");
                    } else {
                        hal_uart_printf("[ERROR] Failed to disable balancing\r\n");
                    }
                    break;

                case CMD_MMM:
                    /* Reset autonomous FET control via CONFIG_UPDATE cycle */
                    hal_uart_printf("[CMD] Full autonomous FET reset...\r\n");
                    bq_controller_print_fet_diag();
                    if (bq_controller_reset_autonomous() == 0) {
                        hal_uart_printf("[OK] Autonomous FET control fully reset\r\n");
                    } else {
                        hal_uart_printf("[ERROR] Failed to reset autonomous control\r\n");
                    }
                    bq_controller_print_fet_diag();
                    break;

                case CMD_KKK:
                    /* Bare CMD_RESET — reset BQ IC and reconfigure */
                    hal_uart_printf("[CMD] Bare CMD_RESET...\r\n");
                    bq_controller_print_fet_diag();
                    if (bq_controller_bare_reset() == 0) {
                        hal_uart_printf("[OK] CMD_RESET done\r\n");
                    } else {
                        hal_uart_printf("[ERROR] CMD_RESET failed\r\n");
                    }
                    bq_controller_print_fet_diag();
                    break;

                case CMD_HHH:
                    /* Show system health status */
                    system_health_print_status();
                    break;

                case CMD_LLL:
                    /* Show calibration values */
                    hal_uart_printf("[CMD] Reading calibration values...\r\n");
                    bq_controller_print_calibration();
                    break;

                case CMD_TTT:
                    /* Toggle debug report output */
                    hal_uart_printf("[CMD] Toggle debug report...\r\n");
                    debug_report_toggle();
                    break;

                case CMD_RRR:
                    /* Print single debug report */
                    hal_uart_printf("[CMD] Single debug report...\r\n");
                    debug_report_print_once();
                    break;

                case CMD_QQQ:
                    /* Toggle balance override cycle test (open-wire bench) */
                    hal_uart_printf("[CMD] Toggle balance override cycle...\r\n");
                    bq_controller_balance_override_toggle();
                    break;

                case CMD_VVV:
                    /* binary protocol protocol self-test */
                    {
                        hal_uart_printf("[CMD] Binary Protocol Self-Test\r\n");

                        /* Create test data with known values */
                        _link_bms_status_t test_data = {
                            .timestamp_ms = 0x12345678,
                            .cell1_mv = 3300,
                            .cell2_mv = 3310,
                            .cell3_mv = 3320,
                            .cell4_mv = 3330,
                            .stack_mv = 13260,
                            .current_ma = -100,
                            .power_dw = -13,
                            .temp_internal = 30,
                            .temp_ts1 = 31,
                            .temp_ext1 = 25,
                            .temp_ext2 = 26,
                            .soc_x100 = 7500,
                            .soh_x100 = 9800,
                            .chg_fet_on = 1,
                            .dsg_fet_on = 1,
                            .balance_active = 0x05,
                            .status_flags = 0x03,
                            .safety_status_a = 0x00,
                            .safety_status_b = 0x00
                        };

                        /* Encode the packet */
                        static uint8_t tx_buffer[64];
                        uint16_t pkt_len = link_encode_bms_status_pkt(
                            tx_buffer, LINK_ID_BMS, LINK_ID_PC, &test_data);

                        /* Print packet info */
                        hal_uart_printf("Packet size: %d bytes (expected 44)\r\n", pkt_len);
                        hal_uart_printf("Payload size: %d bytes\r\n", LINK_BMS_STATUS_PKT_LEN);

                        /* Print hex dump */
                        hal_uart_printf("HEX: ");
                        for (uint16_t i = 0; i < pkt_len; i++) {
                            hal_uart_printf("%02X ", tx_buffer[i]);
                        }
                        hal_uart_printf("\r\n");

                        /* Print packet breakdown */
                        hal_uart_printf("--- Packet Breakdown ---\r\n");
                        hal_uart_printf("Start: 0x%02X (expect 0x07)\r\n", tx_buffer[0]);
                        hal_uart_printf("Op:    0x%02X (expect 0xAC=BROADCAST)\r\n", tx_buffer[1]);
                        hal_uart_printf("Len:   %d (0x%02X%02X)\r\n",
                            tx_buffer[2] | (tx_buffer[3] << 8), tx_buffer[2], tx_buffer[3]);
                        hal_uart_printf("Sender:   0x%02X (expect 0x10=BMS)\r\n", tx_buffer[4]);
                        hal_uart_printf("Receiver: 0x%02X (expect 0xFE=PC)\r\n", tx_buffer[5]);
                        hal_uart_printf("PktID: 0x%02X (expect 0x01=BMS_STATUS)\r\n", tx_buffer[6]);
                        hal_uart_printf("Count: %d\r\n", tx_buffer[7]);

                        /* Print CRC32 (last 4 bytes) */
                        uint32_t crc = tx_buffer[pkt_len-4] |
                                      (tx_buffer[pkt_len-3] << 8) |
                                      (tx_buffer[pkt_len-2] << 16) |
                                      (tx_buffer[pkt_len-1] << 24);
                        hal_uart_printf("CRC32: 0x%08lX\r\n", crc);

                        /* Send raw binary packet */
                        hal_uart_printf("--- Sending Binary Packet ---\r\n");
                        delay_ms(50);

                        hal_uart1_send(tx_buffer, pkt_len);

                        delay_ms(50);
                        hal_uart_printf("\r\n--- End of Test ---\r\n");
                    }
                    break;

                case CMD_PPP:
                    /* Reset PASSQ coulomb counter */
                    hal_uart_printf("[CMD] Resetting PASSQ counter...\r\n");
                    bq_controller_reset_passq();
                    battery_soc_soh_post_config_update();
                    break;

                case CMD_UUU:
                    /* Toggle BQ sleep/wake test */
                    if (bq_controller_is_sleep_test_active()) {
                        bq_controller_sleep_test_stop();
                    } else {
                        bq_controller_sleep_test_start();
                    }
                    break;

                case CMD_YYY: {
                    hal_uart_printf("[UART1_DBG] ISR: total=%lu rx=%lu err=%lu\r\n",
                                     (unsigned long)g_uart1_isr_count,
                                     (unsigned long)g_uart1_isr_rx_count,
                                     (unsigned long)g_uart1_isr_err_count);
                    hal_uart_printf("[UART1_DBG] Parser: total=%lu cmds=%lu crc_err=%lu\r\n",
                                     (unsigned long)g_uart1_rx_total,
                                     (unsigned long)g_uart1_rx_cmds,
                                     (unsigned long)g_uart1_crc_err);
                    hal_uart_printf("[UART1_DBG] BP: rx_pkts=%d rx_err=%d\r\n",
                                     g_bp_rx_mon.received_packets,
                                     g_bp_rx_mon.received_error_packes);
                    /* Loopback test: enable LOOPS, send 3 bytes, check if ISR fires */
                    hal_uart_printf("[UART1_DBG] Loopback test...\r\n");
                    uint32_t isr_before = g_uart1_isr_rx_count;
                    hal_uart1_loopback_enable();
                    uint8_t test_data[3] = {0xAA, 0xBB, 0xCC};
                    hal_uart1_send(test_data, 3);
                    delay_ms(10);
                    hal_uart_printf("[UART1_DBG] Loopback: isr_rx %lu -> %lu (delta=%lu)\r\n",
                                     (unsigned long)isr_before,
                                     (unsigned long)g_uart1_isr_rx_count,
                                     (unsigned long)(g_uart1_isr_rx_count - isr_before));
                    hal_uart1_loopback_disable();
                    break;
                }

                default:
                    break;
            }
}

void controller_run(void)
{
    uint32_t now = systime_milli();
    static uint32_t last_alert_check = 0;
    static uint32_t last_auto_recovery = 0;

    /* Process UART commands first — runs every wake cycle for fast response */
    controller_poll_uart();

    /* Update battery status (I2C reads every 600ms internally) */
    int bq_updated = bq_controller_run();

    /* After the reads, before anything trusts them: notice if the AFE has lost
     * its configuration and put it back. Placed after bq_controller_run() because
     * the signals it watches -- POR, is_connected, config_verify_errors -- are what
     * that call refreshes.
     *
     * Gated on bq_updated so one supervisor tick means one I2C cycle: this function
     * is reached on every LPTMR wake, roughly every 100 ms, while the I2C reads
     * happen every 600 ms, so ungated the three-tick debounce would count wakes
     * rather than measurements. */
    if (bq_updated) {
        (void)bq_link_supervisor_tick();
    }

    /* Get current battery status pointer */
    const bq_battery_status_t *status = bq_controller_get_status();

    /* Synchronize ADC + broadcast with I2C cycle to keep lightweight
     * wakes (~100ms LPTMR) at minimal current. */
    if (bq_updated) {
        sw_oc_protection_update(status->raw_current_ma);
        ext_temp_adc_update();
        /* After the sweep, so the bands see this tick's readings and the
         * open-wire masks they produced. */
        ntc_fault_tick();
    }

    /* Update SOC/SOH estimation every 1 second */
    static uint32_t last_soc_update = 0;
    if ((now - last_soc_update) >= 1000) {
        last_soc_update = now;
        battery_soc_soh_update(status->cell_min_mv, status->current_ma,
                               status->ts1_temp_c, 1000);
    }

    /* Check ALERT pin — during sleep test, check every wake cycle */
    if (bq_controller_is_sleep_test_active()) {
        if (bq_controller_is_alert_active()) {
            bq_controller_sleep_test_handle_alert();
        }
    } else if ((now - last_alert_check) >= 500) {
        if (bq_controller_is_alert_active()) {
            bq_controller_handle_alert();
            last_alert_check = now;
        }
    }

    /* Auto recovery disabled: all protection recovery (COV, CUV, OCD, OCC, SCD,
     * temperature) is handled by the BQ IC's autonomous mechanism using its
     * configured thresholds, hysteresis and RecoveryTime. An MCU PROT_RECOVERY
     * conflicts with the IC's delay counters and recovers prematurely. Manual
     * recovery remains available via UART commands (DDD/CCC). */

    /* Broadcast fresh data only — synchronized with I2C read cycle */
    if (bq_updated) {
        debug_report_run();
    }

    /* Process UART again after heavy work for faster turnaround */
    controller_poll_uart();
}

/*---------------------------------------------------------------------------*/
/* binary protocol Command Handler                                                   */
/*---------------------------------------------------------------------------*/

/**
 * @brief Send BMS_CMD_RESPONSE packet via USART1.
 */
static void send_cmd_response(uint8_t cmd_id, uint8_t result, int32_t value)
{
    _link_bms_cmd_response_t resp = {
        .cmd_id = cmd_id,
        .result = result,
        .value = value};
    uint16_t len = link_encode_bms_cmd_response_pkt(
        cmd_resp_tx_buf, LINK_ID_BMS, LINK_ID_APP, &resp);
    hal_uart1_send(cmd_resp_tx_buf, len);
}

/**
 * @brief Handle incoming binary protocol command packet from USART1.
 */
static void handle_bp_command(link_packet_t *pkt)
{
    /* Parameter commands (REQ/SET/SAVE/RESET) handled first */
    if (param_cmd_handle(pkt)) {
        return;
    }

    if (pkt->packet_id == LINK_BMS_COMMAND_PKT_ID) {
        _link_bms_command_t cmd;
        link_decode_bms_command_pkt(pkt, &cmd);

        int ret = 0;
        int32_t resp_value = 0;

        switch (cmd.cmd_id) {
            case BPCMD_SET_CHG_FET: {
                bool chg = (cmd.param != 0);
                const bq_battery_status_t *st = bq_controller_get_status();
                bool dsg = (st->battery_status & BQ_BATT_STAT_DSG_FET) ? true : false;
                if (chg || dsg)
                    hal_uart_printf("[WARN] Binary-protocol host FET override - autonomous OCD/OCC protection disabled until reboot\r\n");
                ret = bq_controller_set_fets(chg, dsg);
                break;
            }
            case BPCMD_SET_DSG_FET: {
                bool dsg = (cmd.param != 0);
                const bq_battery_status_t *st = bq_controller_get_status();
                bool chg = (st->battery_status & BQ_BATT_STAT_CHG_FET) ? true : false;
                if (chg || dsg)
                    hal_uart_printf("[WARN] Binary-protocol host FET override - autonomous OCD/OCC protection disabled until reboot\r\n");
                ret = bq_controller_set_fets(chg, dsg);
                break;
            }
            case BPCMD_SET_BOTH_FETS: {
                bool chg = (cmd.param & 0x01) ? true : false;
                bool dsg = (cmd.param & 0x02) ? true : false;
                if (chg || dsg)
                    hal_uart_printf("[WARN] Binary-protocol host FET override - autonomous OCD/OCC protection disabled until reboot\r\n");
                ret = bq_controller_set_fets(chg, dsg);
                break;
            }
            case BPCMD_SET_BALANCING:
                ret = bq_controller_set_balancing_enable(cmd.param != 0);
                break;

            case BPCMD_CLEAR_FAULTS:
                ret = bq_controller_clear_faults();
                if (ret == 0) ext_temp_adc_clear_faults();
                sw_oc_protection_clear();
                break;

            case BPCMD_ENTER_SLEEP:
                ret = bq_controller_enter_sleep();
                break;

            /*
             * BPCMD_ENTER_SHUTDOWN (0x07) has no case. SHUTDOWN turns off the
             * internal 1.8 V and REGOUT regulators: all comms die and the whole
             * RAM configuration is lost, recoverable only by a TS-pin wake pulse
             * plus a full reconfigure. Autonomous paths into that state stay
             * disabled by project rule. The switch reaches `default: ret = -1`,
             * so an app that sends 0x07 gets a clean rejection; the opcode
             * define stays so the number is not silently reused.
             */

            case BPCMD_WAKE_PULSE:
                bq_controller_wake_pulse(30);
                break;

            case BPCMD_RESET_PASSQ:
                ret = bq_controller_reset_passq();
                battery_soc_soh_post_config_update();
                break;

            case BPCMD_SET_DEBUG_INTERVAL:
                ret = debug_report_set_interval((uint32_t)cmd.param);
                resp_value = (int32_t)debug_report_get_interval();
                break;

            case BPCMD_SET_DEBUG_TYPE:
                debug_report_set_type((uint8_t)cmd.param);
                resp_value = (int32_t)debug_report_get_type();
                break;

            case BPCMD_TOGGLE_DEBUG:
                resp_value = debug_report_toggle() ? 1 : 0;
                break;

            /*
             * Who owns the single V2 UART.  param 0 = give it to binary protocol, param 1..N =
             * lend it to the console for N seconds, 0xFF = query without changing.
             *
             * The loan is bounded, so a board whose console text is corrupting the
             * binary protocol line recovers without needing that line to carry the
             * instruction.  The reply goes out before the mode changes, so the host
             * always gets it.
             */
            case BPCMD_SET_UART_MODE:
                if (cmd.param == 0xFFu) {
                    resp_value = hal_uart_console_is_enabled() ? 1 : 0;
                } else if (cmd.param == 0u) {
                    hal_uart_console_disable();
                    resp_value = 0;
                } else {
                    hal_uart_console_enable((uint32_t)cmd.param * 1000u);
                    resp_value = 1;
                }
                break;

            case BPCMD_REQUEST_STATUS:
                bq_controller_print_status();
                ext_temp_adc_print_status();
                ntc_fault_print_status();
                battery_soc_soh_print_status();
                break;

            case BPCMD_REQUEST_FAULTS:
                if (bq_controller_has_fault()) {
                    bq_controller_print_fault_details();
                }
                if (ext_temp_adc_has_fault()) {
                    ext_temp_adc_print_fault_details();
                }
                break;

            case BPCMD_REQUEST_CALIBRATION:
                bq_controller_print_calibration();
                break;

            case BPCMD_REQUEST_FET_DIAG:
                bq_controller_print_fet_diag();
                break;

            case BPCMD_REQUEST_HEALTH:
                system_health_print_status();
                break;

            case BPCMD_SET_SOC:
                if (cmd.param >= 0 && cmd.param <= 10000) {
                    float soc = (float)cmd.param / 100.0f;
                    ret = battery_soc_calibrate(soc);
                    resp_value = cmd.param;
                } else {
                    ret = -1;
                }
                break;

            case BPCMD_SINGLE_REPORT:
                debug_report_print_once();
                break;

            case BPCMD_FET_SHUTDOWN:
                ret = bq_controller_manual_shutdown();
                break;

            case BPCMD_FET_RESTORE:
                ret = bq_controller_restore_autonomous();
                break;

            case BPCMD_FET_RESET:
                ret = bq_controller_reset_autonomous();
                break;

            case BPCMD_ENTER_BOOTLOADER: {
                const bq_battery_status_t *st = bq_controller_get_status();
                int32_t ma = st->current_ma;
                bl_compat_status_t compat = bl_compat_check();
                if (ma < 0) ma = -ma;

                /* param 0 is a query, not a request: it reports whether a
                 * bootloader is present and compatible in the response's value
                 * field, without handing anything over. */
                if (cmd.param == 0) {
                    resp_value = (int32_t)bl_compat_report_word();
                    bl_compat_print();
                    ret = 0;
                } else if (cmd.param != BPCMD_BOOTLOADER_CONFIRM) {
                    hal_uart_printf("[BL] refused: confirmation value missing\r\n");
                    ret = -1;
                } else if (compat == BL_COMPAT_NONE) {
                    /* Nothing to hand over to: without this the magic is written,
                     * the board resets and the flat image simply starts again. */
                    hal_uart_printf("[BL] refused: no bootloader present\r\n");
                    ret = -1;
                } else if (compat != BL_COMPAT_OK) {
                    /* A disagreement about the handover window means the magic would
                     * land where the bootloader does not look. */
                    bl_compat_print();
                    hal_uart_printf("[BL] refused: bootloader contract mismatch\r\n");
                    ret = -1;
                } else if (ma > BPCMD_BOOTLOADER_MAX_MA) {
                    hal_uart_printf("[BL] refused: %ld mA flowing\r\n", (long)ma);
                    ret = -1;
                } else {
                    hal_uart_printf("[BL] entering bootloader\r\n");
                    send_cmd_response(cmd.cmd_id, 0, 0);

                    /* Leave the word where the bootloader looks, then reset: a
                     * software reset preserves SRAM. */
                    *(volatile uint32_t *)BL_ENTRY_MAGIC_ADDR = BL_ENTRY_MAGIC;

                    /* Let the response reach the wire before the reset. */
                    for (volatile uint32_t d = 0; d < 400000u; d++) { }

                    S32_SCB->AIRCR = S32_SCB_AIRCR_VECTKEY(FEATURE_SCB_VECTKEY)
                                   | S32_SCB_AIRCR_SYSRESETREQ_MASK;
                    for (;;) { }
                }
                break;
            }


            default:
                ret = -1;  /* Unknown command */
                break;
        }

        send_cmd_response(cmd.cmd_id, (ret == 0) ? 0 : 1, resp_value);
    }
    /* Legacy individual command packets (ID 16-19) */
    else if (pkt->packet_id == LINK_BMS_FET_CONTROL_PKT_ID) {
        _link_bms_fet_control_t fet;
        link_decode_bms_fet_control_pkt(pkt, &fet);
        if (fet.chg_enable || fet.dsg_enable)
            hal_uart_printf("[WARN] Binary-protocol host FET override - autonomous OCD/OCC protection disabled until reboot\r\n");
        int ret = bq_controller_set_fets(fet.chg_enable != 0, fet.dsg_enable != 0);
        _link_bms_ack_t ack = {.cmd_id = LINK_BMS_FET_CONTROL_PKT_ID, .result = (ret == 0) ? 0 : 1};
        uint16_t len = link_encode_bms_ack_pkt(cmd_resp_tx_buf, LINK_ID_BMS, LINK_ID_APP, &ack);
        hal_uart1_send(cmd_resp_tx_buf, len);
    }
    else if (pkt->packet_id == LINK_BMS_BALANCE_PKT_ID) {
        _link_bms_balance_t bal;
        link_decode_bms_balance_pkt(pkt, &bal);
        int ret = bq_controller_set_balancing_enable(bal.enable != 0);
        _link_bms_ack_t ack = {.cmd_id = LINK_BMS_BALANCE_PKT_ID, .result = (ret == 0) ? 0 : 1};
        uint16_t len = link_encode_bms_ack_pkt(cmd_resp_tx_buf, LINK_ID_BMS, LINK_ID_APP, &ack);
        hal_uart1_send(cmd_resp_tx_buf, len);
    }
    else if (pkt->packet_id == LINK_BMS_SLEEP_PKT_ID) {
        int ret = bq_controller_enter_sleep();
        _link_bms_ack_t ack = {.cmd_id = LINK_BMS_SLEEP_PKT_ID, .result = (ret == 0) ? 0 : 1};
        uint16_t len = link_encode_bms_ack_pkt(cmd_resp_tx_buf, LINK_ID_BMS, LINK_ID_APP, &ack);
        hal_uart1_send(cmd_resp_tx_buf, len);
    }
    else if (pkt->packet_id == LINK_BMS_CLEAR_FAULT_PKT_ID) {
        int ret = bq_controller_clear_faults();
        if (ret == 0) ext_temp_adc_clear_faults();
        _link_bms_ack_t ack = {.cmd_id = LINK_BMS_CLEAR_FAULT_PKT_ID, .result = (ret == 0) ? 0 : 1};
        uint16_t len = link_encode_bms_ack_pkt(cmd_resp_tx_buf, LINK_ID_BMS, LINK_ID_APP, &ack);
        hal_uart1_send(cmd_resp_tx_buf, len);
    }
}
