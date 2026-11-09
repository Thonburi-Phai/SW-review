/*
 * link_bms_wire_fault.h -- open-wire classifier verdict and wire-fault response.
 */

#pragma once

#include <stdint.h>
#include "link_protocol.h"

#define LINK_BMS_WIRE_FAULT_PKT_ID 131

typedef struct
{
    uint32_t timestamp_ms;

    /* Classifier (see cell_open_wire.h for the ow_kind_t values). */
    uint8_t  ow_kind;           /* none/mid-tap/top-tap/bottom-tap/multi/... */
    uint8_t  ow_tap;            /* 1..3 for a mid-tap, else 0                */
    uint8_t  ow_cell;           /* collapsed cell, 1-based, 0 if n/a         */
    uint8_t  ow_saturated;      /* verdict came from settled readings        */
    uint8_t  ow_state;          /* idle/suspect/confirmed/recovery           */
    uint16_t ow_events;         /* confirmations since boot                  */

    /* Sum-of-cells minus stack: the discriminator that separates a top-tap open
     * from a dead cell. */
    int16_t  sum_minus_stack_mv;

    /* Response (see wire_fault.h). */
    uint8_t  wf_state;          /* inactive/limp/service                     */
    uint8_t  wf_balancing_locked;
    uint8_t  wf_soc_suppressed;
    uint8_t  dsg_fet_map;       /* live value: 0xF5 production, 0x75 limp     */
    uint32_t wf_grace_left_ms;
    uint16_t wf_reapply_count;  /* profile re-applied after an AFE reset      */
    uint16_t wf_uv_sub_trips;   /* software undervoltage substitute fired     */
} __attribute__((packed)) _link_bms_wire_fault_t;

#define LINK_BMS_WIRE_FAULT_PKT_LEN sizeof(_link_bms_wire_fault_t)

static inline uint16_t link_encode_bms_wire_fault_pkt(uint8_t *buff,
                                                      uint8_t sender_id,
                                                      uint8_t receiver_id,
                                                      _link_bms_wire_fault_t *data)
{
    static uint8_t _count = 0;
    return link_finalize_packet(buff,
                                LINK_OPERATION_BROADCAST,
                                LINK_BMS_WIRE_FAULT_PKT_LEN,
                                sender_id,
                                receiver_id,
                                LINK_BMS_WIRE_FAULT_PKT_ID,
                                _count++,
                                (const uint8_t *)data);
}

static inline void link_decode_bms_wire_fault_pkt(link_packet_t *link_pkt,
                                                  _link_bms_wire_fault_t *data_pkt)
{
    memcpy(data_pkt, link_pkt->payload, LINK_BMS_WIRE_FAULT_PKT_LEN);
}
