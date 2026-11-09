#ifndef LINK_BMS_ENERGY_PKT_H
#define LINK_BMS_ENERGY_PKT_H

#include "link_protocol.h"

// Define the payload structure for BMS_ENERGY
typedef struct
{
    uint32_t timestamp_ms;
    int32_t current_ma;
    int32_t power_mw;
    int32_t passed_charge_uah;
    int32_t dod0_uah;
    int32_t remaining_capacity_mah;
    int32_t used_capacity_mah;
    uint32_t passq_elapsed_time_sec;
} __attribute__((packed)) _link_bms_energy_t;

// Define packet ID and payload length
#define LINK_BMS_ENERGY_PKT_ID 4
#define LINK_BMS_ENERGY_PKT_LEN sizeof(_link_bms_energy_t)

// Encode function: serializes the BMS_ENERGY packet into bytes.
static inline uint16_t link_encode_bms_energy_pkt(uint8_t *buff,
                                                    uint8_t sender_id,
                                                    uint8_t receiver_id,
                                                    _link_bms_energy_t *data)
{
    static uint8_t _count = 0;
    return link_finalize_packet(buff,
                                LINK_OPERATION_BROADCAST,
                                LINK_BMS_ENERGY_PKT_LEN,
                                sender_id,
                                receiver_id,
                                LINK_BMS_ENERGY_PKT_ID,
                                _count++,
                                (const uint8_t *)data);
}

// Decode function: extracts the payload from a received BMS_ENERGY packet.
static inline void link_decode_bms_energy_pkt(link_packet_t *link_pkt, _link_bms_energy_t *data_pkt)
{
    memcpy(data_pkt, &(link_pkt->payload[0]), LINK_BMS_ENERGY_PKT_LEN);
}

#endif // LINK_BMS_ENERGY_PKT_H
