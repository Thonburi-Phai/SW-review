#ifndef LINK_BMS_BALANCE_DEBUG_PKT_H
#define LINK_BMS_BALANCE_DEBUG_PKT_H

#include "link_protocol.h"

/* Block-reason bitmask for _link_bms_balance_debug_t.block_flags */
#define BAL_BLOCK_FAULT        (1u << 0)  /* SS_A (excl COV/CUV) or SS_B set */
#define BAL_BLOCK_HEAVY_CURR   (1u << 1)  /* |I| > cfg_max_curr_ma */
#define BAL_BLOCK_HOT          (1u << 2)  /* internal_temp > cfg_max_temp_c */
#define BAL_BLOCK_LOW_CELL     (1u << 3)  /* vmin < cfg_min_cell_mv */
#define BAL_BLOCK_BELOW_START  (1u << 4)  /* delta_mv < cfg_start_mv (info, not block) */
#define BAL_BLOCK_DISABLED     (1u << 5)  /* g_balancing_enabled == false */
#define BAL_BLOCK_LOW_VMAX     (1u << 6)  /* vmax < cfg_min_start_cell_mv */

typedef struct
{
    uint32_t timestamp_ms;

    /* Host control state */
    uint8_t  enabled;              /* g_balancing_enabled */
    uint8_t  block_flags;          /* see BAL_BLOCK_* bits */
    uint8_t  cmd_mask;             /* g_balance_prev_mask (host bit 0 = cell 1) */
    uint8_t  hw_active_mask;       /* CB_ACTIVE_CELLS readback (logical, bit 0 = cell 1) */
    uint8_t  alarm_cb;             /* 1 if AlarmStatus[CB] (chip actually balancing) */
    uint8_t  cfg_bal_config_reg;   /* BalancingConfiguration (0x9020) raw byte */
    uint16_t tick_count;           /* number of balance ticks since boot */
    uint16_t write_err_count;      /* balance_apply failures */
    int8_t   last_write_status;    /* last bq_status_t */
    uint8_t  cfg_allow_adjacent;

    /* Algorithm inputs (snapshot at last tick) */
    int16_t  vmin_mv;
    int16_t  vmax_mv;
    int16_t  delta_mv;
    int32_t  current_ma;           /* signed current, mA */
    int8_t   internal_temp_c;
    uint8_t  vmin_idx;             /* 0-based cell index */
    uint8_t  vmax_idx;             /* 0-based cell index */
    uint8_t  ncell;

    /* Active configuration (live param values used at last tick) */
    uint16_t cfg_start_mv;
    uint16_t cfg_stop_mv;
    uint16_t cfg_min_cell_mv;
    uint32_t cfg_max_curr_ma;      /* mA */
    uint8_t  cfg_max_temp_c;

    /* Balance voltage gate */
    uint16_t cfg_min_start_cell_mv;

    /* Reserved tail */
    uint8_t  reserved[1];
} __attribute__((packed)) _link_bms_balance_debug_t;

#define LINK_BMS_BALANCE_DEBUG_PKT_ID 21
#define LINK_BMS_BALANCE_DEBUG_PKT_LEN sizeof(_link_bms_balance_debug_t)

static inline uint16_t link_encode_bms_balance_debug_pkt(uint8_t *buff,
                                                          uint8_t sender_id,
                                                          uint8_t receiver_id,
                                                          _link_bms_balance_debug_t *data)
{
    static uint8_t _count = 0;
    return link_finalize_packet(buff,
                                LINK_OPERATION_BROADCAST,
                                LINK_BMS_BALANCE_DEBUG_PKT_LEN,
                                sender_id,
                                receiver_id,
                                LINK_BMS_BALANCE_DEBUG_PKT_ID,
                                _count++,
                                (const uint8_t *)data);
}

static inline void link_decode_bms_balance_debug_pkt(link_packet_t *link_pkt, _link_bms_balance_debug_t *data_pkt)
{
    memcpy(data_pkt, &(link_pkt->payload[0]), LINK_BMS_BALANCE_DEBUG_PKT_LEN);
}

#endif /* LINK_BMS_BALANCE_DEBUG_PKT_H */
