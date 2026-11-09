#ifndef LINK_REQ_PARAM_H
#define LINK_REQ_PARAM_H

#include "link_protocol.h"
#include "parameter.h"

/* binary protocol packet ID for request parameter */
#define LINK_REQ_PARAMETER_PKT_ID   1

/* === Request: App -> BMS (COMMAND) === */

typedef struct {
    uint16_t param_id;
} __attribute__((packed)) link_cmd_req_param_t;

#define LINK_CMD_REQ_PARAM_LEN  sizeof(link_cmd_req_param_t)

static inline void link_decode_req_param(const uint8_t *payload,
                                         link_cmd_req_param_t *out)
{
    memcpy(out, payload, LINK_CMD_REQ_PARAM_LEN);
}

/* === Response: BMS -> App (RESPONSE) === */

typedef struct {
    param_value_t param_value;  /* 4 bytes (union) */
    uint16_t      param_id;
    uint8_t       param_type;
    uint8_t       ack;          /* 0x00 = success, 0x01 = fail */
} __attribute__((packed)) link_resp_req_param_t;

#define LINK_RESP_REQ_PARAM_LEN sizeof(link_resp_req_param_t)

static inline uint16_t link_encode_resp_req_param(uint8_t *buff,
                                                   const param_value_t *value,
                                                   uint16_t param_id,
                                                   uint8_t param_type,
                                                   uint8_t ack)
{
    static uint8_t _count = 0;
    link_resp_req_param_t payload;
    payload.param_value = *value;
    payload.param_id    = param_id;
    payload.param_type  = param_type;
    payload.ack         = ack;

    return link_finalize_packet(buff,
                                LINK_OPERATION_RESPONSE,
                                LINK_RESP_REQ_PARAM_LEN,
                                LINK_ID_BMS,
                                LINK_ID_APP,
                                LINK_REQ_PARAMETER_PKT_ID,
                                _count++,
                                (const uint8_t *)&payload);
}

#endif /* LINK_REQ_PARAM_H */
