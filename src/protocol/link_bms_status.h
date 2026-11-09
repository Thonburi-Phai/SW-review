#ifndef LINK_BMS_STATUS_PKT_H
#define LINK_BMS_STATUS_PKT_H

#include "link_protocol.h"

// Define the payload structure for BMS_STATUS
typedef struct
{
    uint32_t timestamp_ms;
    /* Cell/stack voltages are SIGNED two's complement: a collapsed open-wire
     * node reads -0.2..-0.6 V. Decoders must read int16, not uint16. */
    int16_t cell1_mv;
    int16_t cell2_mv;
    int16_t cell3_mv;
    int16_t cell4_mv;
    int16_t stack_mv;
    int32_t current_ma;
    int32_t power_dw;
    int8_t temp_internal;
    int8_t temp_ts1;
    int8_t temp_ext1;
    int8_t temp_ext2;
    uint16_t soc_x100;
    uint16_t soh_x100;
    uint8_t chg_fet_on;
    uint8_t dsg_fet_on;
    uint8_t balance_active;
    uint8_t status_flags;
    uint8_t safety_status_a;
    uint8_t safety_status_b;
    int8_t temp_ext3;
    int8_t temp_ext4;
    int32_t raw_current_ma;
} __attribute__((packed)) _link_bms_status_t;

// Define packet ID and payload length
#define LINK_BMS_STATUS_PKT_ID 1
#define LINK_BMS_STATUS_PKT_LEN sizeof(_link_bms_status_t)

// Encode function: serializes the BMS_STATUS packet into bytes.
static inline uint16_t link_encode_bms_status_pkt(uint8_t *buff,
                                                    uint8_t sender_id,
                                                    uint8_t receiver_id,
                                                    _link_bms_status_t *data)
{
    static uint8_t _count = 0;
    return link_finalize_packet(buff,
                                LINK_OPERATION_BROADCAST,
                                LINK_BMS_STATUS_PKT_LEN,
                                sender_id,
                                receiver_id,
                                LINK_BMS_STATUS_PKT_ID,
                                _count++,
                                (const uint8_t *)data);
}

// Decode function: extracts the payload from a received BMS_STATUS packet.
static inline void link_decode_bms_status_pkt(link_packet_t *link_pkt, _link_bms_status_t *data_pkt)
{
    memcpy(data_pkt, &(link_pkt->payload[0]), LINK_BMS_STATUS_PKT_LEN);
}

#endif // LINK_BMS_STATUS_PKT_H
