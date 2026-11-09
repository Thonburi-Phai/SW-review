#pragma once
#include <stdint.h>

void crc_init(void);
uint32_t crc_periph_32bit_compute(const uint8_t* data, uint32_t in_size);
