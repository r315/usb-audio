#ifndef _UTIL_H_
#define _UTIL_H_

#include <stdint.h>

void hexDump(uint8_t *buf, uint32_t count);
void triangleWave(uint16_t *dst, uint16_t count, uint8_t nch);
void memset16(uint16_t *buffer, uint32_t set, uint32_t len);
void memcpy16(uint16_t *dst, const uint16_t *src, uint32_t len);
#endif
