#ifndef CRC16_H
#define CRC16_H

#include <stddef.h>
#include <stdint.h>

/* CRC16-CCITT-FALSE：poly=0x1021，init=0xFFFF，refin/refout=false，xorout=0。 */
uint16_t crc16_ccitt_false(const uint8_t *data, size_t length);

#endif /* CRC16_H */

