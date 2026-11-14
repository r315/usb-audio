#ifndef _CODEC_DRIVER_H_
#define _CODEC_DRIVER_H_

#include <stdint.h>

enum codec_if {
    CDC_DEV_DAI = 0,
    CDC_DEV_ADC1,
    CDC_DEV_ADC2,
    CDC_DEV_ADC3,
    CDC_DEV_ADC4,

    CDC_DEV_DAC1,
    CDC_DEV_DAC2,
    CDC_DEV_DAC3,
    CDC_DEV_DAC4,
    CDC_DEV_ALL,

    CDC_CFG_DAI_I2S,
    CDC_CFG_DAI_TDM,
    CDC_SET_I2C_ADDR,
    CDC_GET_I2C_ADDR,
    CDC_GET_MCLK,
};

enum cdc_cfg {
    CDC_CFG_DAI_SLOT_16BIT,
    CDC_CFG_DAI_SLOT_24BIT,
    CDC_CFG_DAI_SLOT_32BIT,
    CDC_CFG_DAI_WORD_16BIT,
    //CDC_CFG_DAI_ŴORD_24BIT,

    CDC_MCLK_PLL,
    CDC_MCLK_32FS,
    CDC_MCLK_48FS,
    CDC_MCLK_64FS,
    CDC_MCLK_128FS,
    CDC_MCLK_256FS,
    CDC_MCLK_512FS,
    CDC_SR_8K = 8000,
    CDC_SR_16K = 16000,
    CDC_SR_22K = 22000,
    CDC_SR_24K = 24000,
    CDC_SR_32K = 32000,
    CDC_SR_44K = 44000,
    CDC_SR_48K = 48000,
    CDC_SR_88K = 88000,
    CDC_SR_96K = 96000,
    CDC_SR_192K = 192000
};


typedef struct codec_drv
{
   uint8_t (*Init) (uint8_t Addr);
   uint8_t (*Config) (uint8_t DevID, uint8_t Mode);
   void    (*SampleRate) (uint32_t Rate);
   void    (*Enable) (void);
   void    (*Disable) (void);
   void    (*Volume) (uint8_t DevID, uint8_t Volume);
   void    (*Mute) (uint8_t DevID, uint8_t Mode);
   uint8_t (*WriteReg) (uint16_t Register, uint8_t Val);
   uint8_t (*ReadReg) (uint16_t Register, uint8_t *Dst);
}codec_drv_t;


#endif