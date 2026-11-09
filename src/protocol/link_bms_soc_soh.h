#ifndef LINK_BMS_SOC_SOH_PKT_H
#define LINK_BMS_SOC_SOH_PKT_H

#include "link_protocol.h"

// Define the payload structure for BMS_SOC_SOH
typedef struct
{
    uint16_t soc_x100;
    uint16_t soc_coulomb_x100;
    uint16_t soc_voltage_x100;
    uint16_t soh_x100;
    uint32_t actual_capacity_mah;
    uint32_t nominal_capacity_mah;
    uint16_t capacity_fade_x100;
    uint16_t coulombic_eff_x100;
    uint32_t cycle_count;
    uint32_t time_to_empty_min;
    uint32_t time_to_full_min;
    uint8_t is_calibrated;
    uint8_t is_fully_charged;
    uint8_t is_empty;
    uint8_t _pad0;
    /* --- v2 fields (offset 36) --- */
    int32_t remaining_capacity_mah;
    uint8_t last_anchor_event;
    uint8_t anchor_flags;
    uint16_t rest_duration_s;
    uint8_t passq_jump_count;
    uint8_t raw_coulomb_active;
    uint8_t _pad1[2];
    /* --- v3 fields (offset 48) --- */
    int32_t  raw_coulomb_mah;
    uint32_t last_learned_capacity_mah;
} __attribute__((packed)) _link_bms_soc_soh_t;

// Define packet ID and payload length
#define LINK_BMS_SOC_SOH_PKT_ID 5
#define LINK_BMS_SOC_SOH_PKT_LEN sizeof(_link_bms_soc_soh_t)

// Encode function: serializes the BMS_SOC_SOH packet into bytes.
static inline uint16_t link_encode_bms_soc_soh_pkt(uint8_t *buff,
                                                    uint8_t sender_id,
                                                    uint8_t receiver_id,
                                                    _link_bms_soc_soh_t *data)
{
    static uint8_t _count = 0;
    return link_finalize_packet(buff,
                                LINK_OPERATION_BROADCAST,
                                LINK_BMS_SOC_SOH_PKT_LEN,
                                sender_id,
                                receiver_id,
                                LINK_BMS_SOC_SOH_PKT_ID,
                                _count++,
                                (const uint8_t *)data);
}

// Decode function: extracts the payload from a received BMS_SOC_SOH packet.
static inline void link_decode_bms_soc_soh_pkt(link_packet_t *link_pkt, _link_bms_soc_soh_t *data_pkt)
{
    memcpy(data_pkt, &(link_pkt->payload[0]), LINK_BMS_SOC_SOH_PKT_LEN);
}

#endif // LINK_BMS_SOC_SOH_PKT_H
