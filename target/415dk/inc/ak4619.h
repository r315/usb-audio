#ifndef _ak4619_h_
#define _ak4619_h_

#include <stdint.h>
#include "audio.h"

#define AK4619_ADDR           0x20  // 8-bit
#define AK4619_ALT_ADDR       AK4619_ALT_ADDR + 2

#define AK4619_PWRMGM_REG     0x00
#define AK4619_AUDFORM1_REG   0x01
#define AK4619_AUDFORM2_REG   0x02
#define AK4619_SYSCLKSET_REG  0x03
#define AK4619_MICGAIN1_REG   0x04
#define AK4619_MICGAIN2_REG   0x05
#define AK4619_ADC1LVOL_REG   0x06
#define AK4619_ADC1RVOL_REG   0x07
#define AK4619_ADC2LVOL_REG   0x08
#define AK4619_ADC2RVOL_REG   0x09
#define AK4619_ADCFILT_REG    0x0A
#define AK4619_ADCAIN_REG     0x0B
#define AK4619_ADCMUTEHPF_REG 0x0D
#define AK4619_DAC1LVOL_REG   0x0E
#define AK4619_DAC1RVOL_REG   0x0F
#define AK4619_DAC2LVOL_REG   0x10
#define AK4619_DAC2RVOL_REG   0x11
#define AK4619_DACDIN_REG     0x12 
#define AK4619_DACDEEM_REG    0x13
#define AK4619_DACMUTFLT_REG  0x14

#define AK4619_PWRMGM_RST     1
#define AK4619_PWRMGM_PMDA1   2
#define AK4619_PWRMGM_PMDA2   4
#define AK4619_PWRMGM_PMAD1   16
#define AK4619_PWRMGM_PMAD2   32



uint8_t ak4619_Init (void);
uint8_t ak4619_Config (uint8_t DevID, uint8_t Mode);
void    ak4619_SampleRate (uint32_t Rate);
void    ak4619_Enable (void);
void    ak4619_Disable (void);
void    ak4619_Volume (uint8_t DevID, uint8_t Volume);
void    ak4619_Mute (uint8_t DevID, uint8_t Mode);

extern const audio_codec_t ak4619;
#endif