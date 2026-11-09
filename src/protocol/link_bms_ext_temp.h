#ifndef LINK_BMS_EXT_TEMP_PKT_H
#define LINK_BMS_EXT_TEMP_PKT_H

#include "link_protocol.h"

typedef struct
{
    uint32_t timestamp_ms;
    int8_t   temp_c[4];         // NTC_CH1..4 temperature (°C)
    uint16_t adc_raw[4];        // raw 12-bit ADC values
    uint8_t  valid_mask;        // bit 0=CH1 .. bit 3=CH4
    uint8_t  fault_mask;        // bit 0=CH1 .. bit 3=CH4
    uint8_t  mosfet_disabled;
    uint8_t  open_wire_mask;        // bits 0-3 = ext NTC CH1-4, bit 4 = BQ TS1
} __attribute__((packed)) _link_bms_ext_temp_t;

#define LINK_BMS_EXT_TEMP_PKT_ID  10
#define LINK_BMS_EXT_TEMP_PKT_LEN sizeof(_link_bms_ext_temp_t)

static inline uint16_t link_encode_bms_ext_temp_pkt(uint8_t *buff,
                                                    uint8_t sender_id,
                                                    uint8_t receiver_id,
                                                    _link_bms_ext_temp_t *data)
{
    static uint8_t _count = 0;
    return link_finalize_packet(buff,
                                LINK_OPERATION_BROADCAST,
                                LINK_BMS_EXT_TEMP_PKT_LEN,
                                sender_id,
                                receiver_id,
                                LINK_BMS_EXT_TEMP_PKT_ID,
                                _count++,
                                (const uint8_t *)data);
}

static inline void link_decode_bms_ext_temp_pkt(link_packet_t *link_pkt,
                                                _link_bms_ext_temp_t *data_pkt)
{
    memcpy(data_pkt, &(link_pkt->payload[0]), LINK_BMS_EXT_TEMP_PKT_LEN);
}

#endif
