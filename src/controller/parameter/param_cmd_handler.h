#ifndef PARAM_CMD_HANDLER_H
#define PARAM_CMD_HANDLER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include "link_protocol.h"

/**
 * @brief Handle an incoming binary protocol command packet for parameter operations
 *
 * Dispatches REQ_PARAMETER (ID 1), SET_PARAMETER (ID 2),
 * SAVE_TO_ROM (ID 3), and RESET_ROM (ID 25).
 *
 * @param pkt Pointer to received packet
 * @return true if the packet was handled (matched a param command ID)
 */
bool param_cmd_handle(const link_packet_t *pkt);

#ifdef __cplusplus
}
#endif

#endif /* PARAM_CMD_HANDLER_H */
