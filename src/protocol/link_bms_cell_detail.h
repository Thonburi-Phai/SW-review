#ifndef LINK_BMS_CELL_DETAIL_PKT_H
#define LINK_BMS_CELL_DETAIL_PKT_H

#include "link_protocol.h"

// Define the payload structure for BMS_CELL_DETAIL
typedef struct
{
    /* SIGNED: open-wire nodes read negative (see link_bms_status.h note) */
    int16_t cell1_mv;
    int16_t cell2_mv;
    int16_t cell3_mv;
    int16_t cell4_mv;
    int16_t cell_max_mv;
    int16_t cell_min_mv;
    int16_t cell_delta_mv;
    uint8_t cell_max_idx;
    uint8_t cell_min_idx;
} __attribute__((packed)) _link_bms_cell_detail_t;

// Define packet ID and payload length
#define LINK_BMS_CELL_DETAIL_PKT_ID 2
#define LINK_BMS_CELL_DETAIL_PKT_LEN sizeof(_link_bms_cell_detail_t)

// Encode function: serializes the BMS_CELL_DETAIL packet into bytes.
static inline uint16_t link_encode_bms_cell_detail_pkt(uint8_t *buff,
                                                    uint8_t sender_id,
                                                    uint8_t receiver_id,
                                                    _link_bms_cell_detail_t *data)
{
    static uint8_t _count = 0;
    return link_finalize_packet(buff,
                                LINK_OPERATION_BROADCAST,
                                LINK_BMS_CELL_DETAIL_PKT_LEN,
                                sender_id,
                                receiver_id,
                                LINK_BMS_CELL_DETAIL_PKT_ID,
                                _count++,
                                (const uint8_t *)data);
}

// Decode function: extracts the payload from a received BMS_CELL_DETAIL packet.
static inline void link_decode_bms_cell_detail_pkt(link_packet_t *link_pkt, _link_bms_cell_detail_t *data_pkt)
{
    memcpy(data_pkt, &(link_pkt->payload[0]), LINK_BMS_CELL_DETAIL_PKT_LEN);
}

#endif // LINK_BMS_CELL_DETAIL_PKT_H
