#ifndef LINK_BMS_COMMAND_PKT_H
#define LINK_BMS_COMMAND_PKT_H

#include "link_protocol.h"

// Define the payload structure for BMS_COMMAND
typedef struct
{
    uint8_t cmd_id;
    uint8_t reserved;
    int32_t param;
} __attribute__((packed)) _link_bms_command_t;

// Define packet ID and payload length
#define LINK_BMS_COMMAND_PKT_ID 20
#define LINK_BMS_COMMAND_PKT_LEN sizeof(_link_bms_command_t)

// Encode function: serializes the BMS_COMMAND packet into bytes.
static inline uint16_t link_encode_bms_command_pkt(uint8_t *buff,
                                                    uint8_t sender_id,
                                                    uint8_t receiver_id,
                                                    _link_bms_command_t *data)
{
    static uint8_t _count = 0;
    return link_finalize_packet(buff,
                                LINK_OPERATION_COMMAND,
                                LINK_BMS_COMMAND_PKT_LEN,
                                sender_id,
                                receiver_id,
                                LINK_BMS_COMMAND_PKT_ID,
                                _count++,
                                (const uint8_t *)data);
}

// Decode function: extracts the payload from a received BMS_COMMAND packet.
static inline void link_decode_bms_command_pkt(link_packet_t *link_pkt, _link_bms_command_t *data_pkt)
{
    memcpy(data_pkt, &(link_pkt->payload[0]), LINK_BMS_COMMAND_PKT_LEN);
}

#endif // LINK_BMS_COMMAND_PKT_H
