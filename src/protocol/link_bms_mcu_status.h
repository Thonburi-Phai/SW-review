#ifndef LINK_BMS_MCU_STATUS_PKT_H
#define LINK_BMS_MCU_STATUS_PKT_H

#include "link_protocol.h"

// MCU Status flags bitmask definitions
#define MCU_FLAG_ALERT_ACTIVE   (1 << 0)   // BQ76907 ALERT pin is LOW (active)
// Bits 1-31 reserved for future use

// Define the payload structure for BMS_MCU_STATUS
typedef struct
{
    uint32_t timestamp_ms;
    uint32_t mcu_flags;              // 32-bit bitmask
    uint32_t config_verify_errors;   // BQ_CFG_ERR_* bitfield — nonzero = readback mismatch
} __attribute__((packed)) _link_bms_mcu_status_t;

// Define packet ID and payload length
#define LINK_BMS_MCU_STATUS_PKT_ID 7
#define LINK_BMS_MCU_STATUS_PKT_LEN sizeof(_link_bms_mcu_status_t)

// Encode function: serializes the BMS_MCU_STATUS packet into bytes.
static inline uint16_t link_encode_bms_mcu_status_pkt(uint8_t *buff,
                                                    uint8_t sender_id,
                                                    uint8_t receiver_id,
                                                    _link_bms_mcu_status_t *data)
{
    static uint8_t _count = 0;
    return link_finalize_packet(buff,
                                LINK_OPERATION_BROADCAST,
                                LINK_BMS_MCU_STATUS_PKT_LEN,
                                sender_id,
                                receiver_id,
                                LINK_BMS_MCU_STATUS_PKT_ID,
                                _count++,
                                (const uint8_t *)data);
}

// Decode function: extracts the payload from a received BMS_MCU_STATUS packet.
static inline void link_decode_bms_mcu_status_pkt(link_packet_t *link_pkt, _link_bms_mcu_status_t *data_pkt)
{
    memcpy(data_pkt, &(link_pkt->payload[0]), LINK_BMS_MCU_STATUS_PKT_LEN);
}

#endif // LINK_BMS_MCU_STATUS_PKT_H
