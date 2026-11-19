/**
  **************************************************************************
  * @file     audio_codec.c
  * @brief    audio codec function
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

/* includes ------------------------------------------------------------------*/
#include <string.h>
#include <stdio.h>
#include "at32f415.h"
#include "audio.h"
#include "audio_desc.h"
#include "board.h"
#include "debug.h"
#include "audio_queue.h"
#include "util.h"

#ifndef ENABLE_DBG_AUDIO
#define DBG_TAG             "AUD : "
#define DBG_AUD_PRINT       DBG_PRINT
#define DBG_AUD_INF(...)    DBG_INF(DBG_TAG __VA_ARGS__)
#define DBG_AUD_WRN(...)    DBG_WRN(DBG_TAG __VA_ARGS__)
#define DBG_AUD_ERR(...)    DBG_ERR(DBG_TAG __VA_ARGS__)
#else
#define DBG_AUD_PRINT(...)
#define DBG_AUD_INF(...)
#define DBG_AUD_WRN(...)
#define DBG_AUD_ERR(...)
#endif

#define BUFFER_MAX_SIZE     1024            // Arbitrary size, must be sufficient at least for 2ms of audio
#define DMA_BUFFER_SIZE     (48 * 2 * 2)    // Number of samples for 1ms, two channels, 48kHz@16bit audio, and double buffering

#define MIC_BUFFER_SIZE     BUFFER_MAX_SIZE
#define SPK_BUFFER_SIZE     BUFFER_MAX_SIZE

#define DEFAULT_SPK_VOL     50
#define DEFAULT_MIC_VOL     50

static audio_driver_t audio_driver;
#if AUDIO_SUPPORT_SPK
static audio_queue_t spk_queue;
static uint16_t spk_queue_buffer[SPK_BUFFER_SIZE];
static uint16_t spk_dma_buffer[DMA_BUFFER_SIZE];
#endif

#if AUDIO_SUPPORT_MIC
static audio_queue_t mic_queue;
static uint16_t mic_queue_buffer[MIC_BUFFER_SIZE];
static uint16_t mic_dma_buffer[DMA_BUFFER_SIZE];
#endif

/**
 * @brief Compute the number of required samples for one millisecond
 * of audio with n channels
 *
 * @param stream
 * @param freq
 */
static void stream_cfg_nsamples(audio_stream_t *stream, uint32_t freq)
{
    // Calculate the number of samples per millisecond unit
    stream->nsamples = freq / 1000 * stream->nchannels;
}

/**
 * @brief Initialize I2S bus with audio driver
 * parameters
 *
 */
static audio_status_t audio_init_i2s(void)
{
#if AUDIO_SUPPORT_MIC || AUDIO_SUPPORT_SPK
    i2s_config_t i2s_cfg;
    audio_driver.codec->Disable();
    bus_i2s_reset();

    i2s_cfg.freq = audio_driver.freq;
    i2s_cfg.bitw = audio_driver.bitw;
    i2s_cfg.mode = audio_driver.mode;
    i2s_cfg.protocol = audio_driver.protocol;

    // DMA buffer has to hold 2ms of audio data
    #if AUDIO_SUPPORT_SPK
    stream_cfg_nsamples(&audio_driver.spk, audio_driver.freq);
    audio_queue_init(&spk_queue, spk_queue_buffer, audio_driver.spk.nsamples, SPK_BUFFER_SIZE);
    i2s_cfg.dma_buf_tx_size = audio_driver.spk.nsamples << 1;
    i2s_cfg.dma_buf_tx = audio_driver.spk.dma_buffer;
    #endif

    #if AUDIO_SUPPORT_MIC
    stream_cfg_nsamples(&audio_driver.mic, audio_driver.freq);
    audio_queue_init(&mic_queue, mic_queue_buffer, audio_driver.mic.nsamples, MIC_BUFFER_SIZE);
    i2s_cfg.dma_buf_rx_size = audio_driver.mic.nsamples << 1;
    i2s_cfg.dma_buf_rx = audio_driver.mic.dma_buffer;
    #endif

    audio_driver.codec->SampleRate(audio_driver.freq);
    audio_driver.codec->Enable();

    if(audio_driver.codec->Config(CDC_CTRL_DAI_PROTO, audio_driver.protocol)){
        DBG_AUD_ERR("CDC Bad Proto");
    }

    if(audio_driver.codec->Config(CDC_CTRL_DAI_BITW, audio_driver.bitw)){
        DBG_AUD_ERR("CDC Bad bitw");
    }

    if(audio_driver.codec->Config(CDC_CTRL_DAI_SLOTW, audio_driver.bitw == 16 ? 16 : 32)){
        DBG_AUD_ERR("CDC Bad slotw");
    }

    return (audio_status_t)bus_i2s_init(&i2s_cfg);
#endif
}

/**
  * @brief  audio codec modify freq
  * @param  freq: freq
  * @retval none
  */
audio_status_t audio_set_freq(uint32_t freq)
{
    if(audio_driver.freq != freq)
    {
        audio_driver.freq = freq;
        audio_init_i2s();
    }
    return AUDIO_OK;
}

/**
  * @brief  audio codec set microphone freq
  * @param  freq: freq (wm8988 microphone and speaker must same freq)
  * @retval none
  */
void audio_set_mic_freq(uint32_t freq)
{
    DBG_AUD_INF("%s :%lu", __func__, freq);
    audio_set_freq(freq);
}

/**
  * @brief  audio codec set speaker freq
  * @param  freq: freq(wm8988 microphone and speaker must same freq)
  * @retval none
  */
void audio_set_spk_freq(uint32_t freq)
{
    DBG_AUD_INF("%s :%lu", __func__, freq);
    audio_set_freq(freq);
}

/**
  * @brief  audio codec set microphone mute
  * @param  mute: mute state
  * @retval none
  */
void audio_set_mic_mute(uint8_t mute)
{
    DBG_AUD_INF("%s :%d", __func__, mute);
}

/**
  * @brief  audio codec set speaker mute
  * @param  mute: mute state
  * @retval none
  */
void audio_set_spk_mute(uint8_t mute)
{
    DBG_AUD_INF("%s :%d", __func__, mute);
}

/**
 * @brief  audio codec set microphone volume
 * @param  volume: the new volume
 * @retval none
 */
void audio_set_mic_volume(uint16_t volume)
{
    DBG_AUD_INF("%s :%d", __func__, volume);
    audio_driver.mic.volume = volume;
}

/**
 * @brief  audio codec set speaker volume
 * @param  volume: the new volume [0-100]
 * @retval none
 */
void audio_set_spk_volume(uint16_t volume)
{
    DBG_AUD_INF("%s :%d", __func__, volume);
    audio_driver.spk.volume = volume;
    audio_driver.codec->Volume(CDC_CTRL_DAC1, volume);
    audio_driver.codec->Volume(CDC_CTRL_DAC2, volume);
}

/**
  * @brief  audio codec microphone alt setting config
  * @param  none
  * @retval none
  */
 void audio_set_mic_state(uint8_t state)
 {
     audio_stream_t *stream = &audio_driver.mic;

     DBG_AUD_INF("%s :%lu", __func__, state);

     stream->state = state ? STREAM_RESUME : STREAM_PAUSED;
 }

/**
 * @brief  audio codec speaker alt setting config
 * @param  none
 * @retval none
 */
void audio_set_spk_state(uint8_t state)
{
    audio_stream_t *stream = &audio_driver.spk;

    DBG_AUD_INF("%s :%lu", __func__, state);

    stream->state = state ? STREAM_RESUME : STREAM_PAUSED;
    audio_driver.feedback_counter = 0;
}

uint8_t audio_get_mic_mute(void)
{
    return 0;
}

uint8_t audio_get_spk_mute(void)
{
    return 0;
}

uint8_t audio_get_mic_volume(void)
{
    return audio_driver.mic.volume;
}

uint8_t audio_get_spk_volume(void)
{
    return audio_driver.spk.volume;
}

uint8_t audio_get_mic_state(void)
{
    audio_stream_t *stream = &audio_driver.mic;
    return !!(stream->state == STREAM_RESUME);
}

uint8_t audio_get_spk_state(void)
{
    audio_stream_t *stream = &audio_driver.spk;
    return !!(stream->state == STREAM_RESUME);
}

/**
 * @brief  Feedback to host for async mode
 *
 * @param  buf output buffer
 * @retval feedback size in bytes
 */
uint8_t audio_get_spk_feedback(uint8_t *buf)
{
    uint32_t feedback_cnt = (audio_driver.feedback_counter) << 1;
    float fd_fval = (float) feedback_cnt / (float)1024.0 * (float)16384/*2^14*/;
    uint32_t feedback_value = (uint32_t)fd_fval;
    audio_driver.feedback_counter = 0;
    buf[0] = (uint8_t)(feedback_value);
    buf[1] = (uint8_t)(feedback_value >> 8);
    buf[2] = (uint8_t)(feedback_value >> 16);
    return 3;
}

void audio_set_codec(const codec_drv_t *codec)
{
    if(!codec){
        return;
    }

    audio_driver.codec = codec;
}

audio_status_t audio_set_proto(enum audio_proto proto)
{
    audio_driver.protocol = proto;
    return AUDIO_OK;
}

audio_status_t audio_set_bitw(uint8_t bitw)
{
    audio_driver.bitw = bitw;
    return AUDIO_OK;
}

enum audio_bitw audio_get_bitw(void)
{
    return audio_driver.bitw;
}

enum audio_proto audio_get_proto(void)
{
    return audio_driver.protocol;
}

void audio_suspend_event(void)
{
    DBG_AUD_INF("%s ", __func__);
}

/**
  * @brief  Callback from audio_class with
  * data to be sent to I2S bus
  *
  * @param  data: Samples to be queued
  * @param  len: Number of samples
  * @retval none
  */
void audio_enqueue_data(const uint16_t *data, uint32_t len)
{
#if AUDIO_SUPPORT_SPK
    audio_stream_t *stream = &audio_driver.spk;
    if (stream->state != STREAM_PAUSED){
        audio_queue_enqueue(&spk_queue, data, len);
    }
#else
    (void)data;
    (void)len;
#endif
}

/**
  * @brief  callback from audio_class to get data from I2S bus
  *
  * @param  buffer: Output buffer
  * @retval Number of samples placed on buffer
  */
 uint32_t audio_dequeue_data(uint16_t *buffer)
 {
#if AUDIO_SUPPORT_MIC
    audio_stream_t *stream = &audio_driver.mic;
    uint16_t nsamples = stream->nsamples;

    if(stream->state != STREAM_PAUSED)
    {
        triangleWave(buffer, nsamples, stream->nchannels);
        //nsamples = audio_queue_dequeue(&mic_queue, buffer, nsamples);
    }
    return nsamples;
#else
    (void)buffer;
    return 0;
#endif
}

/**
  * @brief  audio driver init
  * @param  codec Audio codec driver connected to i2s interface
  * @retval error status
  */
audio_status_t audio_init(const codec_drv_t *codec)
{
    audio_status_t res;

    if(!codec){
        return AUDIO_ERROR_CODEC;
    }
    // Load default format on first call
    if(audio_driver.codec == NULL){
        audio_driver.freq = AUDIO_DEFAULT_FREQ;
        audio_driver.bitw = AUDIO_DEFAULT_BITW;
        audio_driver.mode = AUDIO_DEFAULT_MODE;
        audio_driver.protocol = AUDIO_FORMAT_MSB;
    }
    // Bind codec to driver
    audio_driver.codec = codec;

#if AUDIO_SUPPORT_SPK
    audio_driver.spk.dma_buffer = spk_dma_buffer;
    audio_driver.spk.nchannels = AUDIO_SPK_CHANEL_NUM;
    audio_driver.spk.state = STREAM_INIT;
    audio_set_spk_volume(DEFAULT_SPK_VOL);
#endif
#if AUDIO_SUPPORT_MIC
    audio_driver.mic.dma_buffer = mic_dma_buffer;
    audio_driver.mic.nchannels = AUDIO_MIC_CHR;
    audio_driver.mic.state = STREAM_INIT;
    audio_set_mic_volume(DEFAULT_MIC_VOL);
#endif

    uint8_t cdc_addr = codec->Config(CDC_CTRL_ADDR_GET, 0);

    if(!codec->Init(cdc_addr)){
        DBG_AUD_WRN("Codec init");
    }

    res = audio_init_i2s();

    if(res != AUDIO_OK){
        DBG_AUD_ERR("Fail to configure i2s bus");
        return res;
    }

    bus_i2s_mclk(AUDIO_DEFAULT_MCLK_FREQ, AUDIO_DEFAULT_MCLK_SRC, audio_driver.codec->Config(CDC_CTRL_MCLK_GET, 0));

    return AUDIO_OK;
}

/**
  * @brief  audio codec init
  * @param  none
  * @retval error status
  */
audio_status_t audio_deinit(void)
{
    gpio_init_type gpio_init_struct;
    bus_i2s_reset();
    spi_i2s_reset(SPI1);
    spi_i2s_reset(SPI2);

    gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_MODERATE;
    gpio_init_struct.gpio_mode = GPIO_MODE_INPUT;
    gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
    gpio_init_struct.gpio_pins = I2S1_WS_PIN | I2S1_SD_PIN | I2S1_CK_PIN;
    gpio_init(I2S1_GPIO, &gpio_init_struct);

    gpio_init_struct.gpio_pins = I2S2_WS_PIN | I2S2_SD_PIN | I2S2_CK_PIN;
    gpio_init(I2S2_GPIO, &gpio_init_struct);

    return AUDIO_OK;
}

/**
 * @brief  Handles audio dma half transfer and end of transfer
 * interrupts. It is invoked every one ms
 *
 * @param  none
 * @retval none
 */
void DMA1_Channel3_IRQHandler(void)
{
#if AUDIO_SUPPORT_SPK
    audio_stream_t *stream = &audio_driver.spk;

#if AUDIO_MODE == AUDIO_MODE_ASYNC
    if(stream->state == STREAM_PAUSED){
        audio_queue_flush(&spk_queue);
        dma_flag_clear(DMA1_HDT3_FLAG | DMA1_FDT3_FLAG);
    }else{
        uint16_t half = audio_driver.spk.nsamples;

        if(dma_flag_get(DMA1_HDT3_FLAG) == SET)
        {
            audio_queue_dequeue(&spk_queue, audio_driver.spk.dma_buffer, half);
            dma_flag_clear(DMA1_HDT3_FLAG);
            audio_driver.feedback_counter += (half << 1) - dma_data_number_get(DMA1_CHANNEL3);
        }
        else if(dma_flag_get(DMA1_FDT3_FLAG) == SET)
        {
            audio_queue_dequeue(&spk_queue, audio_driver.spk.dma_buffer + half, half);
            dma_flag_clear(DMA1_FDT3_FLAG);
            audio_driver.feedback_counter += half + ((half << 1) - dma_data_number_get(DMA1_CHANNEL3));
        }
    }
#else
    dma_flag_clear(DMA1_GL3_FLAG | DMA1_HDT3_FLAG | DMA1_FDT3_FLAG);
#endif

#endif /* AUDIO_SUPPORT_SPK */
}

/**
 * @brief  this function handles dma1 channel4 interrupt.
 * @param  none
 * @retval none
 */
void DMA1_Channel4_IRQHandler(void)
{
#if AUDIO_SUPPORT_MIC
    audio_stream_t *stream = &audio_driver.mic;

#if AUDIO_MODE == AUDIO_MODE_ASYNC
    uint16_t *psrc;
    uint16_t half = stream->nsamples;

    if (dma_flag_get(DMA1_HDT4_FLAG) == SET){
        psrc = stream->dma_buffer + half;
        dma_flag_clear(DMA1_HDT4_FLAG);
    }else if (dma_flag_get(DMA1_FDT4_FLAG) == SET){
        psrc = stream->dma_buffer;
        dma_flag_clear(DMA1_FDT4_FLAG);
    }

    if (audio_queue_enqueue(&mic_queue, psrc, half) != half){
        DBG_AUD_WRN("In stream buffer full");
    }
#else
    DMA1->clr = DMA1_GL4_FLAG | DMA1_FDT4_FLAG | DMA1_HDT4_FLAG;
#endif

#else
    dma_flag_clear(DMA1_GL4_FLAG | DMA1_HDT4_FLAG | DMA1_FDT4_FLAG);
#endif /* AUDIO_SUPPORT_MIC */
}
