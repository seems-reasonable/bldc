#ifndef CRSF_CRC8_H_
#define CRSF_CRC8_H_

#include <stdint.h>

void crc8_init(uint8_t poly);
uint8_t crc8_calc(const uint8_t *data, uint8_t len);

#endif  // CRSF_CRC8_H_
