#pragma once

#include <stdint.h>
#include <string.h>

#include "crc_calc.h"

#define LINK_START_BYTE 0x07

#define LINK_HEADER_LEN 8
#define LINK_CRC32_LEN 4
#define LINK_MAX_PAYLOAD_LEN 512 - LINK_HEADER_LEN - LINK_CRC32_LEN
#define LINK_MAX_PACKET_LEN LINK_HEADER_LEN + LINK_MAX_PAYLOAD_LEN + LINK_CRC32_LEN
#define LINK_NON_PAYLOAD_LEN LINK_HEADER_LEN + LINK_CRC32_LEN


typedef enum {
	LINK_OPERATION_COMMAND = 0xAA,
	LINK_OPERATION_RESPONSE = 0xAB,
	LINK_OPERATION_BROADCAST = 0xAC
}LINK_OPERATION;

typedef enum{
	LINK_ID_APP = 0xFF,
	LINK_ID_PC  = 0xFE,
	LINK_ID_BMS = 0x10,    // Battery Management System
	LINK_ID_HGPS = 0x01,
	LINK_ID_PCU = 0x02,
	LINK_ID_RGB = 0x03,
	LINK_ID_PANNEL = 0x04,
	LINK_ID_LANDING_PLATFORM = 0x05,
	LINK_ID_IMU_CALIBRATOR = 0x06,
	LINK_ID_FCU_COMPUTER =0x07,
	LINK_ID_AVISION = 0x08,
	LINK_ID_RTK_STATION = 0x09,
	LINK_ID_NONE = 0x00
}LINK_ID;

typedef struct __link_packet {
	uint32_t crc32;
	uint8_t start;
	uint8_t op;
	uint16_t len;
	uint8_t sender_id;
	uint8_t receiver_id;
	uint8_t packet_id;
	uint8_t count;
	uint8_t payload[LINK_MAX_PAYLOAD_LEN];
}__attribute__((packed)) link_packet_t;

typedef enum {
	LINK_PARSE_STATE_IDLE =0,
	LINK_PARSE_STATE_GOT_START,
	LINK_PARSE_STATE_GOT_OP,
	LINK_PARSE_STATE_GOT_LENGTH_LOW,
	LINK_PARSE_STATE_GOT_LENGTH_HIGH,
	LINK_PARSE_STATE_GOT_SENDER_ID,
	LINK_PARSE_STATE_GOT_RECIEVER_ID,
	LINK_PARSE_STATE_GOT_PACKET_ID,
	LINK_PARSE_STATE_GOT_COUNT,
	LINK_PARSE_STATE_GOT_PAYLOAD,
	LINK_PARSE_STATE_GOT_CRC32
} link_parse_state_t;

typedef struct __link_monitor
{
	uint16_t received_packets;
	uint16_t received_error_packes;
	uint16_t transmitted_packets;
	link_parse_state_t parse_state;
	uint16_t packet_idx;
	uint16_t idx;
	uint16_t payload_idx ;
    uint8_t crc_idx;
} link_monitor_t;

uint16_t link_uint8_parser(uint8_t b, link_packet_t *packet, link_monitor_t *monitor);
uint16_t link_finalize_packet(uint8_t *buff, 
                            uint8_t op, 
                            uint16_t len, 
                            uint8_t sender_id, 
                            uint8_t receiver_id, 
                            uint8_t packet_id, 
                            uint8_t count,
                            const uint8_t* payload);

static inline uint16_t bp_packet_serialize(uint8_t *buff, link_packet_t * pkt)
{

    memcpy( buff, (const uint8_t*)&(pkt->start), pkt->len+ LINK_HEADER_LEN);

    buff[pkt->len + LINK_HEADER_LEN   ] = (uint8_t)(pkt->crc32);
	buff[pkt->len + LINK_HEADER_LEN +1] = (uint8_t)(pkt->crc32>>8);
	buff[pkt->len + LINK_HEADER_LEN +2] = (uint8_t)(pkt->crc32>>16);
	buff[pkt->len + LINK_HEADER_LEN +3] = (uint8_t)(pkt->crc32>>24);

	return (LINK_NON_PAYLOAD_LEN + pkt->len);
}

// BMS packet definitions
#include "link_bms_status.h"
#include "link_bms_cell_detail.h"
#include "link_bms_fault.h"
#include "link_bms_energy.h"
#include "link_bms_soc_soh.h"
#include "link_bms_fault_detail.h"
#include "link_bms_fet_control.h"
#include "link_bms_balance.h"
#include "link_bms_sleep.h"
#include "link_bms_clear_fault.h"
#include "link_bms_command.h"
#include "link_bms_ack.h"
#include "link_bms_cmd_response.h"
#include "link_bms_sw_oc_status.h"
#include "link_bms_ext_temp.h"

