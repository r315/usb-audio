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
#include "audio_queue.h"
#include "codec_driver.h"

enum stream_state{
    STREAM_INIT = 0,
    STREAM_PAUSED,
    STREAM_RESUME,
    STREAM_FILL_FIRST,
    STREAM_FILL_SECOND
};

typedef enum audio_status{
    AUDIO_OK = 0,
    AUDIO_ERROR_FREQ,
    AUDIO_ERROR_BITW,
    AUDIO_ERROR_PROTO,
    AUDIO_ERROR_CODEC,
}audio_status_t;

typedef struct audio_stream_s
{
    uint16_t nsamples;      // number of samples per millisecond times number of channels
    uint16_t *dma_buffer;
    uint16_t threshold;
    uint16_t calc;
    uint8_t  nchannels;
    uint8_t  volume;
    enum stream_state state;
}audio_stream_t;

typedef struct audio_driver_s
{
    uint32_t freq;
    uint32_t bitw;
    uint8_t  mode;
    enum audio_proto protocol;
    //spk part
    audio_stream_t spk;
    //mic part
    audio_stream_t mic;
    // Common part
    uint32_t feedback_counter;
    const codec_drv_t *codec;
}audio_driver_t;

/**
  * @brief audio codec interface
  */
audio_status_t audio_init(const codec_drv_t *codec);
audio_status_t audio_deinit(void);

audio_status_t audio_set_bitw(uint8_t bitw);
void audio_set_codec(const codec_drv_t *codec);
audio_status_t audio_set_freq(uint32_t freq);
audio_status_t audio_set_proto(enum audio_proto proto);
void audio_set_mic_state(uint8_t state);
void audio_set_mic_freq(uint32_t freq);
void audio_set_mic_mute(uint8_t mute);
void audio_set_mic_volume(uint16_t volume);
void audio_set_spk_state(uint8_t state);
void audio_set_spk_freq(uint32_t freq);
void audio_set_spk_mute(uint8_t mute);
void audio_set_spk_volume(uint16_t volume);

uint8_t audio_get_mic_state(void);
uint8_t audio_get_mic_mute(void);
uint8_t audio_get_mik_volume(void);
uint8_t audio_get_spk_state(void);
uint8_t audio_get_spk_mute(void);
uint8_t audio_get_spk_volume(void);

enum audio_bitw audio_get_bitw(void);
enum audio_proto audio_get_proto(void);
uint8_t audio_get_mic_volume(void);
uint8_t audio_get_spk_feedback(uint8_t *feedback);
uint8_t audio_get_spk_volume(void);

void audio_enqueue_data(const uint16_t *data, uint32_t len);
uint32_t audio_dequeue_data(uint16_t *buffer);
void audio_suspend_event(void);
#ifdef __cplusplus
}
#endif

#endif

