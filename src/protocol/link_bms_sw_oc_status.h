#ifndef LINK_BMS_SW_OC_STATUS_PKT_H
#define LINK_BMS_SW_OC_STATUS_PKT_H

#include "link_protocol.h"

typedef struct
{
    uint32_t timestamp_ms;
    /* Slow DSG/CHG */
    uint16_t dsg_oc_counter;
    uint16_t chg_oc_counter;
    uint16_t dsg_oc_trip_count;
    uint16_t chg_oc_trip_count;
    uint8_t  dsg_oc_tripped;
    uint8_t  chg_oc_tripped;
    int32_t  dsg_threshold_ma;
    int32_t  chg_threshold_ma;
    uint8_t  decay_rate;
    uint8_t  _pad0;
    /* Fast1 DSG */
    uint16_t fast1_counter;
    uint16_t fast1_trip_count;
    uint8_t  fast1_tripped;
    uint8_t  fast1_decay_rate;
    int32_t  fast1_threshold_ma;
    /* Fast2 DSG */
    uint16_t fast2_counter;
    uint16_t fast2_trip_count;
    uint8_t  fast2_tripped;
    uint8_t  fast2_decay_rate;
    int32_t  fast2_threshold_ma;
} __attribute__((packed)) _link_bms_sw_oc_status_t;

#define LINK_BMS_SW_OC_STATUS_PKT_ID  8
#define LINK_BMS_SW_OC_STATUS_PKT_LEN sizeof(_link_bms_sw_oc_status_t)

static inline uint16_t link_encode_bms_sw_oc_status_pkt(uint8_t *buff,
                                                        uint8_t sender_id,
                                                        uint8_t receiver_id,
                                                        _link_bms_sw_oc_status_t *data)
{
    static uint8_t _count = 0;
    return link_finalize_packet(buff,
                                LINK_OPERATION_BROADCAST,
                                LINK_BMS_SW_OC_STATUS_PKT_LEN,
                                sender_id,
                                receiver_id,
                                LINK_BMS_SW_OC_STATUS_PKT_ID,
                                _count++,
                                (const uint8_t *)data);
}

static inline void link_decode_bms_sw_oc_status_pkt(link_packet_t *link_pkt,
                                                    _link_bms_sw_oc_status_t *data_pkt)
{
    memcpy(data_pkt, &(link_pkt->payload[0]), LINK_BMS_SW_OC_STATUS_PKT_LEN);
}

#endif
