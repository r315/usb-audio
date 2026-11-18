#include <stdio.h>
#include "util.h"

/**
 * @brief Generates a triangle wave using all values of uin16_t
 * Note: a full period of this wave is 128k samples
 *
 * @param dst
 * @param nsamples
 * @param nch
 */
void triangleWave(uint16_t *dst, uint16_t nsamples, uint8_t nch)
{
    static uint16_t sample_value = 0;
    static uint8_t sample_count_dir = 0;

    for (int i = 0; i < nsamples; i += nch){
        if(sample_count_dir == 0 && sample_value == 0x8000){
            sample_count_dir = 1;
            sample_value = 0x7ffe;
        }else if(sample_count_dir == 1 && sample_value == 0x7fff){
            sample_count_dir = 0;
            sample_value = 0x8001;
        }

        if(nch == 1)
            *dst++ = sample_value;
        else{
            *dst++ = sample_value;
            *dst++ = sample_value;
        }
        sample_value = sample_count_dir ? sample_value - 1 : sample_value + 1;
    }
}

void memset16(uint16_t *buffer, uint32_t set, uint32_t len)
{
    while(len--){
        *buffer++ = (uint16_t)set;
    }
}

void memcpy16(uint16_t *dst, const uint16_t *src, uint32_t len)
{
    while(len--){
        *dst++ = *src++;
    }
}

void hexDump(uint8_t *buf, uint32_t count)
{
    for(unsigned int i = 0; i < count; i ++){
        if( (i & 15) == 0)
            printf("\n%02X: ", i & 0xF0);
        printf("%02X ", buf[i]);
    }
    putchar('\n');
}