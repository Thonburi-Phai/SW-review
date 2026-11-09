/*
 * link_bms_ntc_fault.h -- MCU NTC band verdict and the FET request it raises.
 *
 * Packet 10 carries the raw per-channel readings; this carries what the
 * decision layer made of them. Separate packet so packet 10's length, which the
 * app's parser fixes, does not move.
 */

#pragma once

#include <stdint.h>
#include "link_protocol.h"

#define LINK_BMS_NTC_FAULT_PKT_ID 132

typedef struct
{
    uint32_t timestamp_ms;

    uint8_t  enabled;            /* PARAM_NTC_SAFETY_EN                       */
    uint8_t  state;              /* ok/degraded/tripped/service               */
    uint8_t  usable_battery_mask;/* CH1-3 present and not open-wire           */
    uint8_t  open_wire_mask;     /* bits 0-3 = NTC_CH1-4, bit 4 = BQ TS1      */

    /* Most restrictive band over the usable battery channels, and the MOSFET
     * channel's own band (see ntc_bands.h for the values). */
    uint8_t  chg_band;
    uint8_t  dsg_band;
    uint8_t  fet_band;
    uint8_t  fet_temp_source;    /* 0 = none, 1 = NTC_CH4, 2 = BQ TS1         */

    int16_t  battery_min_c;
    int16_t  battery_max_c;
    int16_t  fet_temp_c;

    uint8_t  want_fets_off;      /* what the bands ask for                    */
    uint8_t  applied_fets_off;   /* what is requested, after the trip delay   */
    uint8_t  fets_off_union;     /* every owner's bits, as written to FETControl */
} __attribute__((packed)) _link_bms_ntc_fault_t;

#define LINK_BMS_NTC_FAULT_PKT_LEN sizeof(_link_bms_ntc_fault_t)

static inline uint16_t link_encode_bms_ntc_fault_pkt(uint8_t *buff,
                                                     uint8_t sender_id,
                                                     uint8_t receiver_id,
                                                     _link_bms_ntc_fault_t *data)
{
    static uint8_t _count = 0;
    return link_finalize_packet(buff,
                                LINK_OPERATION_BROADCAST,
                                LINK_BMS_NTC_FAULT_PKT_LEN,
                                sender_id,
                                receiver_id,
                                LINK_BMS_NTC_FAULT_PKT_ID,
                                _count++,
                                (const uint8_t *)data);
}

static inline void link_decode_bms_ntc_fault_pkt(link_packet_t *link_pkt,
                                                 _link_bms_ntc_fault_t *data_pkt)
{
    memcpy(data_pkt, link_pkt->payload, LINK_BMS_NTC_FAULT_PKT_LEN);
}
