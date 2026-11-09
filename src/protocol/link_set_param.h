#ifndef LINK_SET_PARAM_H
#define LINK_SET_PARAM_H

#include "link_protocol.h"

/* binary protocol packet ID for set parameter */
#define LINK_SET_PARAMETER_PKT_ID   2

/* === Command: App -> BMS (COMMAND, no response) === */

typedef struct {
    uint32_t param_value;   /* Raw 4-byte value */
    uint16_t param_id;
    uint8_t  param_type;
} __attribute__((packed)) link_cmd_set_param_t;

#define LINK_CMD_SET_PARAM_LEN  sizeof(link_cmd_set_param_t)

static inline void link_decode_set_param(const uint8_t *payload,
                                         link_cmd_set_param_t *out)
{
    memcpy(out, payload, LINK_CMD_SET_PARAM_LEN);
}

#endif /* LINK_SET_PARAM_H */
