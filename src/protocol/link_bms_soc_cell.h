#ifndef LINK_BMS_SOC_CELL_PKT_H
#define LINK_BMS_SOC_CELL_PKT_H

#include "link_protocol.h"

/* BMS_SOC_CELL (ID 22) -- per-cell SOC offsets, anchor grade, capacity learning
 * state and the current-sign guard. The pack SOC in BMS_STATUS / BMS_SOC_SOH is
 * the LOWEST cell's SOC. */
typedef struct
{
    uint32_t timestamp_ms;
    uint16_t soc_cell_x100[4];      /* per-cell SOC (% x100) */
    uint16_t soc_pack_x100;         /* lowest cell = displayed pack SOC */
    uint16_t soc_max_x100;          /* highest cell */
    uint16_t offset_max_x100;       /* highest cell offset above pack (% x100) */
    uint32_t usable_capacity_mah;   /* actual_capacity - imbalance_loss */
    uint32_t imbalance_loss_mah;    /* actual_capacity * offset_max / 100 */
    uint32_t cell_capacity_mah[4];  /* per-cell learned capacity */
    int16_t  drift_at_anchor_x10;   /* target - soc when the last anchor fired (% x10) */
    int16_t  reconcile_delta_x10;   /* same, for the last rest reconciliation (% x10) */
    int16_t  v_ocv_mv[4];           /* IR-corrected cell voltages used for anchoring */
    uint8_t  anchor_grade;          /* 0 none, 1 low, 2 medium, 3 high */
    uint8_t  last_anchor_event;     /* SOC_ANCHOR_* */
    uint8_t  learn_count;
    uint8_t  learn_refused_reason;  /* SOC_LEARN_* */
    uint8_t  flags2;                /* SOC_FLAG2_* */
    uint8_t  rest_minutes;          /* current rest duration, saturating */
    uint8_t  _pad[2];
} __attribute__((packed)) _link_bms_soc_cell_t;

#define LINK_BMS_SOC_CELL_PKT_ID 22
#define LINK_BMS_SOC_CELL_PKT_LEN sizeof(_link_bms_soc_cell_t)

static inline uint16_t link_encode_bms_soc_cell_pkt(uint8_t *buff,
                                                    uint8_t sender_id,
                                                    uint8_t receiver_id,
                                                    _link_bms_soc_cell_t *data)
{
    static uint8_t _count = 0;
    return link_finalize_packet(buff,
                                LINK_OPERATION_BROADCAST,
                                LINK_BMS_SOC_CELL_PKT_LEN,
                                sender_id,
                                receiver_id,
                                LINK_BMS_SOC_CELL_PKT_ID,
                                _count++,
                                (const uint8_t *)data);
}

static inline void link_decode_bms_soc_cell_pkt(link_packet_t *link_pkt, _link_bms_soc_cell_t *data_pkt)
{
    memcpy(data_pkt, &(link_pkt->payload[0]), LINK_BMS_SOC_CELL_PKT_LEN);
}

#endif /* LINK_BMS_SOC_CELL_PKT_H */
