#ifndef LINK_RESET_ROM_H
#define LINK_RESET_ROM_H

#include "link_protocol.h"

/* binary protocol packet ID for reset ROM / reset to defaults */
#define LINK_RESET_ROM_PKT_ID   25

/* === Command: App -> BMS === */

typedef struct {
    uint16_t reset_id;      /* Parameter ID to reset (0xFFFF = all) */
    uint8_t  reset_type;    /* 0 = reset to defaults */
} __attribute__((packed)) link_cmd_reset_rom_t;

#define LINK_CMD_RESET_ROM_LEN  sizeof(link_cmd_reset_rom_t)

static inline void link_decode_reset_rom(const uint8_t *payload,
                                         link_cmd_reset_rom_t *out)
{
    memcpy(out, payload, LINK_CMD_RESET_ROM_LEN);
}

/* === Response: BMS -> App === */

typedef struct {
    uint8_t ack;    /* 0x00 = success */
} __attribute__((packed)) link_resp_reset_rom_t;

#define LINK_RESP_RESET_ROM_LEN sizeof(link_resp_reset_rom_t)

static inline uint16_t link_encode_resp_reset_rom(uint8_t *buff, uint8_t ack)
{
    static uint8_t _count = 0;
    link_resp_reset_rom_t payload;
    payload.ack = ack;

    return link_finalize_packet(buff,
                                LINK_OPERATION_RESPONSE,
                                LINK_RESP_RESET_ROM_LEN,
                                LINK_ID_BMS,
                                LINK_ID_APP,
                                LINK_RESET_ROM_PKT_ID,
                                _count++,
                                (const uint8_t *)&payload);
}

#endif /* LINK_RESET_ROM_H */
