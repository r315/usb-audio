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
#include "codec.h"
/**
  * @}
  */

/** @defgroup USB_device_audio_hid_codec_pin_definition
  * @{
  */
#define I2S1_WS_PIN                      GPIO_PINS_4
#define I2S1_CK_PIN                      GPIO_PINS_5
#define I2S1_SD_PIN                      GPIO_PINS_7

#define I2S1_GPIO                        GPIOA
#define I2S1_GPIO_CRM_CLK                CRM_GPIOA_PERIPH_CLOCK
#define I2S1_DT_ADDRESS                  (&(SPI1->dt))

/*I2S2 Pin*/
#define I2S2_WS_PIN                      GPIO_PINS_12
#define I2S2_CK_PIN                      GPIO_PINS_13
#define I2S2_SD_PIN                      GPIO_PINS_15


#define I2S2_GPIO                        GPIOB
#define I2S2_GPIO_CRM_CLK                CRM_GPIOB_PERIPH_CLOCK
#define I2S2_DT_ADDRESS                  (&(SPI2->dt))

#define SPK_TX_FIFO_SIZE                 (1024 * 4)

typedef struct audio_channel_s 
{
    uint32_t freq;
    uint16_t *buffer;
    uint16_t *roff;
    uint16_t *woff;
    uint16_t *end;
    uint32_t size;
    uint32_t wtotal;
    uint32_t rtotal;
    uint32_t delta;
    uint16_t *dma_buffer;
    uint16_t threshold;
    uint16_t calc;
    uint16_t volume;
    uint8_t  stage;
    uint8_t  adj_stage;
    uint16_t adj_count;
    uint8_t  mute;
    uint8_t  enabled;
}audio_channel_t;

typedef struct audio_driver_s
{
    uint32_t freq;
    uint32_t bitw;
    uint8_t  mode;
    //spk part
    audio_channel_t spk;
    //mic part
    audio_channel_t mic;
    // Common part
    const CDC_Type *codec;
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
audio_status_t audio_init(const CDC_Type *codec);
audio_status_t audio_loop(void);
audio_status_t audio_change_mode(uint32_t mode);
void audio_cfg_mclk(uint32_t freq, uint32_t enable);
void audio_set_freq(uint32_t freq);

void audio_enqueue_data(uint8_t *data, uint32_t len);
uint32_t audio_dequeue_data(uint8_t *buffer);
uint8_t audio_spk_feedback(uint8_t *feedback);
void audio_spk_alt_setting(uint32_t alt_seting);
void audio_mic_alt_setting(uint32_t alt_seting);
void audio_set_mic_mute(uint8_t mute);
void audio_set_spk_mute(uint8_t mute);
void audio_set_mic_volume(uint16_t volume);
void audio_set_spk_volume(uint16_t volume);
void audio_set_mic_freq(uint32_t freq);
void audio_set_spk_freq(uint32_t freq);

#ifdef __cplusplus
}
#endif

#endif

