#ifndef LINK_SAVE_TO_ROM_H
#define LINK_SAVE_TO_ROM_H

#include "link_protocol.h"

/* binary protocol packet ID for save to ROM / apply to IC */
#define LINK_SAVE_TO_ROM_PKT_ID    3

/* === Command: App -> BMS === */

typedef struct {
    uint8_t device_type;
} __attribute__((packed)) link_cmd_save_to_rom_t;

#define LINK_CMD_SAVE_TO_ROM_LEN  sizeof(link_cmd_save_to_rom_t)

static inline void link_decode_save_to_rom(const uint8_t *payload,
                                           link_cmd_save_to_rom_t *out)
{
    memcpy(out, payload, LINK_CMD_SAVE_TO_ROM_LEN);
}

/* === Response: BMS -> App === */

typedef struct {
    uint8_t ack;    /* 0x00 = success */
} __attribute__((packed)) link_resp_save_to_rom_t;

#define LINK_RESP_SAVE_TO_ROM_LEN  sizeof(link_resp_save_to_rom_t)

static inline uint16_t link_encode_resp_save_to_rom(uint8_t *buff, uint8_t ack)
{
    static uint8_t _count = 0;
    link_resp_save_to_rom_t payload;
    payload.ack = ack;

    return link_finalize_packet(buff,
                                LINK_OPERATION_RESPONSE,
                                LINK_RESP_SAVE_TO_ROM_LEN,
                                LINK_ID_BMS,
                                LINK_ID_APP,
                                LINK_SAVE_TO_ROM_PKT_ID,
                                _count++,
                                (const uint8_t *)&payload);
}

#endif /* LINK_SAVE_TO_ROM_H */
