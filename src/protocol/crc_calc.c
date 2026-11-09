#include "crc_calc.h"
#include "hal_crc.h"

void crc_init(void) {
    hal_crc_init();
}

uint32_t crc_periph_32bit_compute(const uint8_t* data, uint32_t in_size) {
    return hal_crc_stm32_compute(data, in_size);
}
