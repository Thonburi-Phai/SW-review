/*
 * hal_crc.c -- STM32F4-compatible hardware CRC-32 for S32K116
 *
 * - Poly 0x04C11DB7, seed 0xFFFFFFFF
 * - No bit reversal (TOT=0, TOTR=0), no final XOR (FXOR=0)
 * - 32-bit word feed, MSB-first byte packing
 * - Remainder bytes zero-padded in lower positions
 */

#include "hal_crc.h"
#include "device_registers.h"

void hal_crc_init(void)
{
    PCC->PCCn[PCC_CRC_INDEX] |= PCC_PCCn_CGC_MASK;
}

uint32_t hal_crc_stm32_compute(const uint8_t *data, uint32_t size)
{
    /* 32-bit, no transpose, no XOR */
    CRC->CTRL = CRC_CTRL_TCRC(1) | CRC_CTRL_WAS(1);
    CRC->GPOLY = 0x04C11DB7u;
    CRC->DATAu.DATA = 0xFFFFFFFFu;
    CRC->CTRL = CRC_CTRL_TCRC(1);   /* Clear WAS -> data mode */

    /* Feed full 32-bit words, MSB-first byte packing */
    while (size >= 4u) {
        CRC->DATAu.DATA = ((uint32_t)data[0] << 24)
                         | ((uint32_t)data[1] << 16)
                         | ((uint32_t)data[2] << 8)
                         |  (uint32_t)data[3];
        data += 4u;
        size -= 4u;
    }

    /* Remaining bytes zero-padded in lower positions */
    if (size > 0u) {
        uint32_t word = 0u;
        switch (size) {
        case 3u:
            word = ((uint32_t)data[0] << 24)
                 | ((uint32_t)data[1] << 16)
                 | ((uint32_t)data[2] << 8);
            break;
        case 2u:
            word = ((uint32_t)data[0] << 24)
                 | ((uint32_t)data[1] << 16);
            break;
        case 1u:
            word = ((uint32_t)data[0] << 24);
            break;
        default:
            break;
        }
        CRC->DATAu.DATA = word;
    }

    return CRC->DATAu.DATA;
}
