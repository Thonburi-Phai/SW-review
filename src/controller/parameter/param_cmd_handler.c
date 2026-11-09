/**
 * @file    param_cmd_handler.c
 * @brief   binary protocol parameter command handler for BMS
 *
 * Handles REQ_PARAMETER, SET_PARAMETER, SAVE_TO_ROM, RESET_ROM
 * commands received over binary protocol and sends responses.
 *
 * - SAVE_TO_ROM applies cached params to BQ IC (no flash)
 * - RESET_ROM resets cache to defaults and applies to BQ IC
 */

#include "param_cmd_handler.h"
#include "link_req_param.h"
#include "link_set_param.h"
#include "link_save_to_rom.h"
#include "link_reset_rom.h"
#include "parameter.h"
#include "battery_soc_soh.h"
#include "hal_uart.h"
#include "hal_uart1.h"
#include "hal_systime.h"
#include <stdio.h>
#include <string.h>

/* TX buffer for response packets */
static uint8_t param_tx_buf[64];

/* --------------------------------------------------------------------
 * REQ_PARAMETER (ID 1): Read a parameter and send response
 * ------------------------------------------------------------------ */
static void handle_req_parameter(const link_packet_t *pkt)
{
    link_cmd_req_param_t cmd;
    link_decode_req_param(pkt->payload, &cmd);

    param_value_t value;
    uint8_t ack;
    param_type_t type;

    if (cmd.param_id < PARAM_COUNT && param_get((param_id_t)cmd.param_id, &value)) {
        type = param_get_type((param_id_t)cmd.param_id);
        ack = 0x00; /* success */
    } else {
        value.u32 = 0;
        type = PARAM_TYPE_UINT32;
        ack = 0x01; /* fail */
    }

    uint16_t len = link_encode_resp_req_param(param_tx_buf, &value,
                                               cmd.param_id, (uint8_t)type, ack);
    hal_uart1_send(param_tx_buf, len);

    hal_uart_printf("[PARAM] GET id=%d val=%ld ack=%d txLen=%d\r\n",
                      cmd.param_id, (long)value.i32, ack, len);
}

/* --------------------------------------------------------------------
 * SET_PARAMETER (ID 2): Write a parameter value (no response)
 * ------------------------------------------------------------------ */
static void handle_set_parameter(const link_packet_t *pkt)
{
    link_cmd_set_param_t cmd;
    link_decode_set_param(pkt->payload, &cmd);

    param_value_t value;
    value.u32 = cmd.param_value;

    bool ok = false;
    if (cmd.param_id < PARAM_COUNT) {
        ok = param_set((param_id_t)cmd.param_id, &value);
    }

    if (ok && cmd.param_id == PARAM_DESIGN_CAPACITY_MAH) {
        battery_soc_soh_update_capacity();
    }

    hal_uart_printf("[PARAM] SET id=%d ok=%d\r\n", cmd.param_id, ok);
}

/* --------------------------------------------------------------------
 * SAVE_TO_ROM (ID 3): Apply cached params to BQ IC
 * ------------------------------------------------------------------ */
static void handle_save_to_rom(const link_packet_t *pkt)
{
    (void)pkt;

    hal_uart_printf("[PARAM] APPLY start...\r\n");

    uint32_t t0 = systime_milli();
    int ret = param_apply_to_bq();
    uint32_t elapsed = systime_milli() - t0;

    uint8_t ack = (ret == 0) ? 0x00 : 0x01;

    uint16_t len = link_encode_resp_save_to_rom(param_tx_buf, ack);
    hal_uart1_send(param_tx_buf, len);

    hal_uart_printf("[PARAM] APPLY ack=%d %lums\r\n", ack, (unsigned long)elapsed);
}

/* --------------------------------------------------------------------
 * RESET_ROM (ID 25): Reset parameters to defaults and apply
 * ------------------------------------------------------------------ */
static void handle_reset_rom(const link_packet_t *pkt)
{
    (void)pkt;

    hal_uart_printf("[PARAM] RESET start...\r\n");

    param_apply_defaults_to_cache();

    uint32_t t0 = systime_milli();
    int ret = param_apply_to_bq();
    uint32_t elapsed = systime_milli() - t0;

    uint8_t ack = (ret == 0) ? 0x00 : 0x01;

    uint16_t len = link_encode_resp_reset_rom(param_tx_buf, ack);
    hal_uart1_send(param_tx_buf, len);

    hal_uart_printf("[PARAM] RESET ack=%d %lums\r\n", ack, (unsigned long)elapsed);
}

/* ====================================================================
 * Public API
 * ==================================================================== */

bool param_cmd_handle(const link_packet_t *pkt)
{
    /* Only handle COMMAND packets */
    if (pkt->op != (uint8_t)LINK_OPERATION_COMMAND) {
        return false;
    }

    switch (pkt->packet_id) {
        case LINK_REQ_PARAMETER_PKT_ID:
            if (pkt->len >= LINK_CMD_REQ_PARAM_LEN) {
                handle_req_parameter(pkt);
                return true;
            }
            break;

        case LINK_SET_PARAMETER_PKT_ID:
            if (pkt->len >= LINK_CMD_SET_PARAM_LEN) {
                handle_set_parameter(pkt);
                return true;
            }
            break;

        case LINK_SAVE_TO_ROM_PKT_ID:
            handle_save_to_rom(pkt);
            return true;

        case LINK_RESET_ROM_PKT_ID:
            if (pkt->len >= LINK_CMD_RESET_ROM_LEN) {
                handle_reset_rom(pkt);
                return true;
            }
            break;

        default:
            break;
    }

    return false;
}
