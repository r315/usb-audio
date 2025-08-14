/**
  **************************************************************************
  * @file     audio_codec.h
  * @brief    audio codec header file
  **************************************************************************
  *                       Copyright notice & Disclaimer
  *
  * The software Board Support Package (BSP) that is made available to
  * download from Artery official website is the copyrighted work of Artery.
  * Artery authorizes customers to use, copy, and distribute the BSP
  * software and its related documentation for the purpose of design and
  * development in conjunction with Artery microcontrollers. Use of the
  * software is governed by this copyright notice and the following disclaimer.
  *
  * THIS SOFTWARE IS PROVIDED ON "AS IS" BASIS WITHOUT WARRANTIES,
  * GUARANTEES OR REPRESENTATIONS OF ANY KIND. ARTERY EXPRESSLY DISCLAIMS,
  * TO THE FULLEST EXTENT PERMITTED BY LAW, ALL EXPRESS, IMPLIED OR
  * STATUTORY OR OTHER WARRANTIES, GUARANTEES OR REPRESENTATIONS,
  * INCLUDING BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY,
  * FITNESS FOR A PARTICULAR PURPOSE, OR NON-INFRINGEMENT.
  *
  **************************************************************************
  */

/* define to prevent recursive inclusion -------------------------------------*/
#ifndef __AUDIO_CODEC_H
#define __AUDIO_CODEC_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "audio_conf.h"
/**
  * @}
  */

/** @defgroup USB_device_audio_hid_codec_pin_definition
  * @{
  */
#define I2S1_WS_PIN                      GPIO_PINS_4
#define I2S1_CK_PIN                      GPIO_PINS_5
#define I2S1_SD_PIN                      GPIO_PINS_7
#define I2S1_MCK_PIN                     GPIO_PINS_0

#define I2S1_GPIO                        GPIOA
#define I2S1_GPIO_CRM_CLK                CRM_GPIOA_PERIPH_CLOCK
#define I2S1_DT_ADDRESS                  (&(SPI1->dt))
#define I2S1_MCK_GPIO                    GPIOB

/*I2S2 Pin*/
#define I2S2_WS_PIN                      GPIO_PINS_12
#define I2S2_CK_PIN                      GPIO_PINS_13
#define I2S2_SD_PIN                      GPIO_PINS_15

#define I2S2_GPIO                        GPIOB
#define I2S2_GPIO_CRM_CLK                CRM_GPIOB_PERIPH_CLOCK
#define I2S2_DT_ADDRESS                  (&(SPI2->dt))

#define SPK_TX_FIFO_SIZE                 (1024 * 4)

enum cdc_cfg_e {
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

enum cdc_cfg_val_e {
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

typedef struct audio_stream_s
{
    //uint32_t freq;
    uint16_t *queue_start;  // Start of queue that holds usb data
    uint16_t *queue_end;
    uint16_t queue_size;
    uint16_t *roff;         // Queue read index pointer
    uint16_t *woff;         // Queue write index pointer
    uint16_t nsamples;      // number of samples per millisecond
    uint32_t wtotal;
    uint32_t rtotal;
    uint32_t delta;
    uint16_t *dma_buffer;
    uint16_t threshold;
    uint16_t calc;
    uint16_t adj_count;
    uint8_t  nchannels;
    uint8_t  volume;
    uint8_t  stage;
    uint8_t  adj_stage;
    uint8_t  mute;
    uint8_t  enabled;
}audio_stream_t;

typedef struct audio_codec_s
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
}audio_codec_t;

typedef struct audio_driver_s
{
    uint32_t freq;
    uint32_t bitw;
    uint8_t  mode;
    //spk part
    audio_stream_t spk;
    //mic part
    audio_stream_t mic;
    // Common part
    const audio_codec_t *codec;
}audio_driver_t;

typedef enum audio_status_e{
    AUDIO_OK = 0,
    AUDIO_ERROR_FREQ,
    AUDIO_ERROR_BITW,
    AUDIO_ERROR_CODEC,
}audio_status_t;

/**
  * @brief audio codec interface
  */
audio_status_t audio_init(const audio_codec_t *codec);
audio_status_t audio_deinit(void);
audio_status_t audio_loop(void);
audio_status_t audio_change_mode(uint32_t mode);
void audio_cfg_mclk(uint32_t freq, uint32_t enable);
void audio_set_freq(uint32_t freq);
void audio_set_codec(const audio_codec_t *codec);

void audio_enqueue_data(uint8_t *data, uint32_t len);
uint32_t audio_dequeue_data(uint8_t *buffer);
uint8_t audio_spk_feedback(uint8_t *feedback);
void audio_spk_alt_setting(uint32_t alt_seting);
void audio_mic_alt_setting(uint32_t alt_seting);
void audio_set_mic_mute(uint8_t mute);
void audio_set_spk_mute(uint8_t mute);
void audio_set_mic_volume(uint16_t volume);
uint8_t audio_get_mic_volume(void);
void audio_set_spk_volume(uint16_t volume);
uint8_t audio_get_spk_volume(void);
void audio_set_mic_freq(uint32_t freq);
void audio_set_spk_freq(uint32_t freq);
void audio_suspend_event(void);
#ifdef __cplusplus
}
#endif

#endif

