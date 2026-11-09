#ifndef LINK_BMS_FAULT_DETAIL_PKT_H
#define LINK_BMS_FAULT_DETAIL_PKT_H

#include "link_protocol.h"

// Define the payload structure for BMS_FAULT_DETAIL
typedef struct
{
    uint32_t timestamp_ms;
    uint8_t safety_alert_a;
    uint8_t safety_status_a;
    uint8_t safety_alert_b;
    uint8_t safety_status_b;
    uint16_t battery_status;
    uint16_t alarm_status;
    uint8_t ext_temp_fault_mask;
    uint8_t system_health_flags;
    uint8_t is_connected;
    uint8_t cuv_tripped;
} __attribute__((packed)) _link_bms_fault_detail_t;

// Define packet ID and payload length
#define LINK_BMS_FAULT_DETAIL_PKT_ID 6
#define LINK_BMS_FAULT_DETAIL_PKT_LEN sizeof(_link_bms_fault_detail_t)

// Encode function: serializes the BMS_FAULT_DETAIL packet into bytes.
static inline uint16_t link_encode_bms_fault_detail_pkt(uint8_t *buff,
                                                    uint8_t sender_id,
                                                    uint8_t receiver_id,
                                                    _link_bms_fault_detail_t *data)
{
    static uint8_t _count = 0;
    return link_finalize_packet(buff,
                                LINK_OPERATION_BROADCAST,
                                LINK_BMS_FAULT_DETAIL_PKT_LEN,
                                sender_id,
                                receiver_id,
                                LINK_BMS_FAULT_DETAIL_PKT_ID,
                                _count++,
                                (const uint8_t *)data);
}

// Decode function: extracts the payload from a received BMS_FAULT_DETAIL packet.
static inline void link_decode_bms_fault_detail_pkt(link_packet_t *link_pkt, _link_bms_fault_detail_t *data_pkt)
{
    memcpy(data_pkt, &(link_pkt->payload[0]), LINK_BMS_FAULT_DETAIL_PKT_LEN);
}

#endif // LINK_BMS_FAULT_DETAIL_PKT_H
