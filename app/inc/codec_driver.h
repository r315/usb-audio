#ifndef _CODEC_DRIVER_H_
#define _CODEC_DRIVER_H_

#include <stdint.h>

enum audio_proto {
    AUDIO_FORMAT_I2S,
    AUDIO_FORMAT_MSB,
    AUDIO_FORMAT_PCM,
    AUDIO_FORMAT_TDM,
};

enum audio_bitw {
    AUDIO_BITW_16 = 16,
    AUDIO_BITW_24 = 24,
    AUDIO_BITW_32 = 32,
};

enum audio_sr {
    AUDIO_SR_8K = 8000,
    AUDIO_SR_16K = 16000,
    AUDIO_SR_22K = 22000,
    AUDIO_SR_24K = 24000,
    AUDIO_SR_32K = 32000,
    AUDIO_SR_44K = 44000,
    AUDIO_SR_48K = 48000,
    AUDIO_SR_88K = 88000,
    AUDIO_SR_96K = 96000,
    AUDIO_SR_192K = 192000,
    AUDIO_MCLK_PLL,
};

enum cdc_ctrl {
    CDC_CTRL_DAI_PROTO = 0,
    CDC_CTRL_DAI_BITW,
    CDC_CTRL_DAI_SLOTW,

    CDC_CTRL_ADC1,
    CDC_CTRL_ADC2,
    CDC_CTRL_ADC3,
    CDC_CTRL_ADC4,

    CDC_CTRL_DAC1,
    CDC_CTRL_DAC2,
    CDC_CTRL_DAC3,
    CDC_CTRL_DAC4,
    CDC_CTRL_ALL,

    CDC_CTRL_ADDR_SET,
    CDC_CTRL_ADDR_GET,
    CDC_CTRL_MCLK_GET,
};

enum cdc_cfg {
    CDC_MCLK_32FS,
    CDC_MCLK_48FS,
    CDC_MCLK_64FS,
    CDC_MCLK_128FS,
    CDC_MCLK_256FS,
    CDC_MCLK_512FS,
};


typedef struct codec_drv
{
   uint8_t (*Init) (uint8_t Addr);
   uint32_t (*Config) (uint8_t Control, uint8_t Param);
   void    (*SampleRate) (uint32_t Rate);
   void    (*Enable) (void);
   void    (*Disable) (void);
   void    (*Volume) (uint8_t DevID, uint8_t Volume);
   void    (*Mute) (uint8_t DevID, uint8_t Mode);
   uint8_t (*WriteReg) (uint16_t Register, uint8_t Val);
   uint8_t (*ReadReg) (uint16_t Register, uint8_t *Dst);
}codec_drv_t;


#endif