#include "crc16.h"

uint16_t crc16_ccitt_false(const uint8_t *data, size_t length)
{
    uint16_t crc = 0xFFFFU;
    size_t i;
    uint8_t bit;

    for (i = 0U; i < length; ++i) {
        crc ^= (uint16_t)data[i] << 8U;
        for (bit = 0U; bit < 8U; ++bit) {
            if ((crc & 0x8000U) != 0U) {
                crc = (uint16_t)((crc << 1U) ^ 0x1021U);
            } else {
                crc <<= 1U;
            }
        }
    }
    return crc;
}

