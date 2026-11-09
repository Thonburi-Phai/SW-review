#include "link_protocol.h"
#include "crc_calc.h"

uint16_t link_uint8_parser(uint8_t b, link_packet_t *packet, link_monitor_t *monitor)
{

    uint16_t ret = 0;

    switch (monitor->parse_state)
    {

    case LINK_PARSE_STATE_IDLE:
        if (b == LINK_START_BYTE)
        {
            monitor->parse_state = LINK_PARSE_STATE_GOT_START;
            monitor->idx = 0;
            monitor->payload_idx = 0;
            monitor->crc_idx = 0;
            packet->start = LINK_START_BYTE;
            packet->crc32 = 0;

            monitor->idx++;
        }

        break;

    case LINK_PARSE_STATE_GOT_START:

        if ((b == LINK_OPERATION_COMMAND) ||
            (b == LINK_OPERATION_RESPONSE) ||
            (b == LINK_OPERATION_BROADCAST))
        {

            monitor->parse_state = LINK_PARSE_STATE_GOT_OP;
            packet->op = b;
            monitor->idx++;
        }
        else
        {
            monitor->parse_state = LINK_PARSE_STATE_IDLE;
        }

        break;

    case LINK_PARSE_STATE_GOT_OP:
        /* length low byte */
        monitor->parse_state = LINK_PARSE_STATE_GOT_LENGTH_LOW;
        packet->len = b;
        monitor->idx++;

        break;

    case LINK_PARSE_STATE_GOT_LENGTH_LOW:
        /* length high byte; len is little-endian on the wire */
        monitor->parse_state = LINK_PARSE_STATE_GOT_LENGTH_HIGH;
        packet->len = b << 8 | packet->len;
        monitor->idx++;
        break;

    case LINK_PARSE_STATE_GOT_LENGTH_HIGH:
        /* sender id, not checked */
        monitor->parse_state = LINK_PARSE_STATE_GOT_SENDER_ID;
        packet->sender_id = b;
        monitor->idx++;
        break;

    case LINK_PARSE_STATE_GOT_SENDER_ID:
        /* receiver id: BMS, PC, APP and NONE are accepted */
        if ((b == LINK_ID_BMS) || (b == LINK_ID_PC) || (b == LINK_ID_APP) || (b == LINK_ID_NONE))
        {
            packet->receiver_id = b;
            monitor->idx++;
            monitor->parse_state = LINK_PARSE_STATE_GOT_RECIEVER_ID;
        }
        else
        {
            monitor->parse_state = LINK_PARSE_STATE_IDLE;
        }
        break;

    case LINK_PARSE_STATE_GOT_RECIEVER_ID:
        /* packet id, not checked */
        monitor->parse_state = LINK_PARSE_STATE_GOT_PACKET_ID;
        packet->packet_id = b;
        monitor->idx++;

        break;

    case LINK_PARSE_STATE_GOT_PACKET_ID:
        /* rolling frame counter */
        packet->count = b;
        monitor->parse_state = LINK_PARSE_STATE_GOT_COUNT;
        monitor->idx++;
        break;

    case LINK_PARSE_STATE_GOT_COUNT:
        /* payload bytes */
        if (packet->len != 0)
        {
            packet->payload[monitor->payload_idx++] = b;
            monitor->idx++;
        }

        /* over LINK_MAX_PAYLOAD_LEN: discard the frame */
        if ((monitor->payload_idx > LINK_MAX_PAYLOAD_LEN))
        {

            monitor->parse_state = LINK_PARSE_STATE_IDLE;
        }
        else if ((monitor->payload_idx >= packet->len))
        {
            monitor->parse_state = LINK_PARSE_STATE_GOT_PAYLOAD;
        }

        break;
    case LINK_PARSE_STATE_GOT_PAYLOAD:
        /* CRC-32, little-endian, over op..end of payload */
        packet->crc32 |= (uint32_t)b << (monitor->crc_idx * 8);
        monitor->crc_idx++;
        monitor->idx++;
        if (monitor->crc_idx == 4)
        {
            monitor->parse_state = LINK_PARSE_STATE_GOT_CRC32;
            uint32_t temp_crc = crc_periph_32bit_compute(
                (const uint8_t *)&(packet->op),
                LINK_HEADER_LEN + packet->len - 1);
            if (packet->crc32 != temp_crc)
            {
                monitor->received_error_packes++;
                ret = 0;
            }
            else
            {

                ret = 1;
            }
            monitor->received_packets++;
            monitor->parse_state = LINK_PARSE_STATE_IDLE;
        }

        break;
    case LINK_PARSE_STATE_GOT_CRC32:

        break;
    default:

        break;
    }

    return ret;
}

uint16_t link_finalize_packet(uint8_t *buff,
                              uint8_t op,
                              uint16_t len,
                              uint8_t sender_id,
                              uint8_t receiver_id,
                              uint8_t packet_id,
                              uint8_t count,
                              const uint8_t *payload)
{
    link_packet_t fp;
    fp.start = LINK_START_BYTE;
    fp.op = op;
    fp.len = len;
    fp.sender_id = sender_id;
    fp.receiver_id = receiver_id;
    fp.packet_id = packet_id;
    fp.count = count;
    memcpy((uint8_t *)&(fp.payload[0]), (const uint8_t *)payload, len);

    memcpy(buff, (const uint8_t *)&(fp.start), len + LINK_HEADER_LEN);

    fp.crc32 =
        crc_periph_32bit_compute((const uint8_t *)&(fp.op),
                                 LINK_HEADER_LEN + len - 1);

    memcpy(buff + len + LINK_HEADER_LEN, &(fp.crc32), 4);

    return LINK_HEADER_LEN + LINK_CRC32_LEN + fp.len;
}

