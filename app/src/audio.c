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

typedef enum stream_stage_e{
    STREAM_INIT = 0,
    STREAM_PAUSED,
    STREAM_RESUME,
    STREAM_FILL_FIRST,
    STREAM_FILL_SECOND
}stream_stage_e;

static audio_driver_t audio_driver;
static uint16_t spk_dma_buffer[DMA_BUFFER_SIZE];
static uint16_t mic_dma_buffer[DMA_BUFFER_SIZE];
static uint16_t spk_buffer[SPK_BUFFER_SIZE];        // spk queue buffer for usb
static uint16_t mic_buffer[MIC_BUFFER_SIZE];        // mic queue buffer for usb

static void wave_triangle(uint16_t *dst, uint16_t count)
{
    static uint16_t sample_count = 0;
    static uint8_t sample_count_dir = 0;

    for (int i = 0; i < count; ++i){
        if(sample_count_dir == 0 && sample_count == 0x8000){
            sample_count_dir = 1;
            sample_count = 0x7ffe;
        }else if(sample_count_dir == 1 && sample_count == 0x7fff){
            sample_count_dir = 0;
            sample_count = 0x8001;
        }

        *dst++ = sample_count;
        sample_count = sample_count_dir ? sample_count - 1  : sample_count + 1;
    }
}

static void memset16(uint16_t *buffer, uint32_t set, uint32_t len)
{
    while(len--){
        *buffer++ = (uint16_t)set;
    }
}

static void memcpy16(uint16_t *dst, uint16_t *src, uint32_t len)
{
    while(len--){
        *dst++ = *src++;
    }
}

static void stream_cfg_nsamples(audio_stream_t *stream, uint32_t freq)
{
    // Calculate the number of samples per millisecond unit
    stream->nsamples = freq / 1000 * stream->nchannels;
}

/**
  * @brief  audio codec modify freq
  * @param  freq: freq
  * @retval none
  */
void audio_set_freq(uint32_t freq)
{
    i2s_config_t i2s_cfg;

    if(audio_driver.freq != freq)
    {
        audio_driver.freq = freq;
        audio_driver.codec->Disable();
        bus_i2s_reset();

        stream_cfg_nsamples(&audio_driver.spk, audio_driver.freq);
        stream_cfg_nsamples(&audio_driver.mic, audio_driver.freq);

        i2s_cfg.freq = freq;
        i2s_cfg.mode = audio_driver.mode;
        i2s_cfg.bitw = audio_driver.bitw;
        i2s_cfg.dma_buf_tx_size = audio_driver.spk.nsamples;
        i2s_cfg.dma_buf_rx_size = audio_driver.mic.nsamples;
        i2s_cfg.dma_buf_tx = audio_driver.spk.dma_buffer;
        i2s_cfg.dma_buf_rx = audio_driver.mic.dma_buffer;
        bus_i2s_init(&i2s_cfg);

        audio_driver.codec->SampleRate(freq);
        audio_driver.codec->Enable();
    }
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

uint8_t audio_get_mic_volume(void)
{
    return audio_driver.mic.volume;
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
    audio_driver.codec->Volume(CDC_DEV_DAC1, volume);
    audio_driver.codec->Volume(CDC_DEV_DAC2, volume);
}

uint8_t audio_get_spk_volume(void)
{
    return audio_driver.spk.volume;
}

/**
  * @brief  audio codec speaker alt setting config
  * @param  none
  * @retval none
  */
void audio_spk_alt_setting(uint32_t alt_seting)
{
    audio_stream_t *stream = &audio_driver.spk;

    DBG_AUD_INF("%s :%lu", __func__, alt_seting);

    if(!alt_seting){
        // Fill both buffers
        memset16(stream->dma_buffer, 0x0000, stream->nsamples << 1);
        stream->stage = STREAM_PAUSED;
    }else{
        stream->stage = STREAM_RESUME;
    }
}

/**
  * @brief  audio codec microphone alt setting config
  * @param  none
  * @retval none
  */
void audio_mic_alt_setting(uint32_t alt_seting)
{
    audio_stream_t *stream = &audio_driver.mic;

    DBG_AUD_INF("%s :%lu", __func__, alt_seting);

    if(!alt_seting){
        memset16(stream->dma_buffer, 0x0000, stream->nsamples << 1);
        stream->stage = STREAM_PAUSED;
    }else{
        stream->stage = STREAM_RESUME;
    }
}

/**
  * @brief  codec speaker feedback
  * @param  feedback: data buffer
  * @retval feedback len
  */
uint8_t audio_spk_feedback(uint8_t *feedback)
{
    uint32_t feedback_value = (audio_driver.freq);

    feedback_value = ((feedback_value/1000)<<14)|((feedback_value % 1000)<<4);
    feedback[0] = (uint8_t)(feedback_value);
    feedback[1] = (uint8_t)(feedback_value >> 8);
    feedback[2] = (uint8_t)(feedback_value >> 16);

    return 3;
}

void audio_suspend_event(void)
{
    DBG_AUD_INF("%s ", __func__);
}

void audio_set_codec(const audio_codec_t *codec)
{
    if(!codec){
        return;
    }

    audio_driver.codec = codec;
}

/**
  * @brief  Callback from audio_class with
  * data to be sent to I2S bus
  *
  * @param  data: Input buffer
  * @param  len: Input data length
  * @retval none
  */
void audio_enqueue_data(uint8_t *buffer, uint32_t len)
{
    audio_stream_t *stream = &audio_driver.spk;
    uint16_t nsamples = len >> 1;
#ifdef AUDIO_SYNCHRONOUS_MODE
    switch (stream->stage)
    {
        case STREAM_PAUSED:
            break;
        case STREAM_FILL_FIRST:
            memcpy16(stream->dma_buffer, (uint16_t*)buffer, nsamples);
            break;
        case STREAM_FILL_SECOND:
            memcpy16(stream->dma_buffer + nsamples, (uint16_t*)buffer, nsamples);
            break;
    }
#else
    switch (stream->stage)
    {
        case STREAM_INIT:
            queue_flush(&stream->queue);
            stream->threshold = queue_size(&stream->queue) >> 1;
            stream->stage = STREAM_FILL_FIRST;
            DBG_AUD_INF("Starting out stream");
            break;

        case STREAM_FILL_FIRST:
            if (queue_count(&stream->queue) >= stream->threshold){
                stream->stage = STREAM_FILL_SECOND;     // At least half of queue is full
            }
            break;

        case STREAM_FILL_SECOND:
            if (queue_count(&stream->queue) < stream->nsamples){
                DBG_AUD_WRN("usb is unable to keep up");
            }
            break;
    }

    if(queue_push(&stream->queue, (uint16_t*)buffer, nsamples) != nsamples){
        DBG_AUD_WRN("Out stream buffer full");
    }
#endif
}

/**
  * @brief  callback from audio_class to get data from I2S bus
  *
  * @param  buffer: Output buffer
  * @retval Number of bytes placed on buffer
  */
 uint32_t audio_dequeue_data(uint8_t *buffer)
 {
    audio_stream_t *stream = &audio_driver.mic;
    uint16_t nsamples = stream->nsamples;
#ifdef AUDIO_SYNCHRONOUS_MODE
    switch (stream->stage)
    {
        case STREAM_PAUSED:
            memset16((uint16_t*)buffer, 0, nsamples);
            break;
        case STREAM_FILL_FIRST:
            memcpy16( (uint16_t*)buffer, stream->dma_buffer, nsamples);
            break;
        case STREAM_FILL_SECOND:
            memcpy16( (uint16_t*)buffer, stream->dma_buffer + nsamples, nsamples);
            break;
    }
#else
    switch (stream->stage)
    {
        case STREAM_INIT:
            queue_flush(&stream->queue);
            memset16((uint16_t*)buffer, 0, nsamples);
            stream->threshold = queue_size(&stream->queue) / 2;
            stream->stage = STREAM_FILL_FIRST;
            DBG_AUD_INF("Starting in stream");
            break;

        case STREAM_FILL_FIRST:
            if (queue_count(&stream->queue) >= stream->threshold){
                queue_pop(&stream->queue, (uint16_t*)buffer, nsamples);
                stream->stage = STREAM_FILL_SECOND;
                DBG_AUD_INF("In stream half full");
            }else{
                memset16((uint16_t*)buffer, 0, nsamples);
            }
            break;

        case STREAM_FILL_SECOND:
            nsamples = queue_pop(&stream->queue, (uint16_t*)buffer, nsamples);
            break;
    }
#endif
    return nsamples << 1;
}

/**
  * @brief  audio codec init
  * @param  none
  * @retval error status
  */
audio_status_t audio_init(const audio_codec_t *codec)
{
    audio_status_t res;
    i2s_config_t i2s_cfg;

    if(!codec){
        return AUDIO_ERROR_CODEC;
    }

    if(audio_driver.codec == NULL){
        audio_driver.freq = AUDIO_DEFAULT_FREQ;
        audio_driver.bitw = AUDIO_DEFAULT_BITW;
        audio_driver.mode = AUDIO_DEFAULT_MODE;
        audio_driver.spk.nchannels = AUDIO_SPK_CHANEL_NUM;
        audio_driver.mic.nchannels = AUDIO_MIC_CHANEL_NUM;
    }

    audio_driver.codec = codec;

    memset16(spk_buffer, 0, SPK_BUFFER_SIZE);
    memset16(mic_buffer, 0, MIC_BUFFER_SIZE);
    memset16(spk_dma_buffer, 0, DMA_BUFFER_SIZE);
    memset16(mic_dma_buffer, 0, DMA_BUFFER_SIZE);

    stream_cfg_nsamples(&audio_driver.spk, audio_driver.freq);
    stream_cfg_nsamples(&audio_driver.mic, audio_driver.freq);

#ifndef AUDIO_SYNCHRONOUS_MODE
    //assert(audio_driver.spk.nsamples <= SPK_BUFFER_SIZE);
    //assert(audio_driver.mic.nsamples <= MIC_BUFFER_SIZE);

    audio_queue_init(&audio_driver.spk.queue, spk_buffer, audio_driver.spk.nsamples, SPK_BUFFER_SIZE);
    audio_queue_init(&audio_driver.mic.queue, mic_buffer, audio_driver.mic.nsamples, MIC_BUFFER_SIZE);
#endif
    audio_driver.mic.dma_buffer = mic_dma_buffer;
    audio_driver.spk.dma_buffer = spk_dma_buffer;

    audio_driver.spk.nchannels = AUDIO_SPK_CHANEL_NUM;
    audio_driver.mic.nchannels = AUDIO_MIC_CHANEL_NUM;

    audio_driver.spk.stage = STREAM_INIT;
    audio_driver.mic.stage = STREAM_INIT;

    i2s_cfg.freq = audio_driver.freq;
    i2s_cfg.bitw = audio_driver.bitw;
    i2s_cfg.mode = audio_driver.mode;
    i2s_cfg.dma_buf_tx_size = audio_driver.spk.nsamples;
    i2s_cfg.dma_buf_rx_size = audio_driver.spk.nsamples;
    i2s_cfg.dma_buf_tx = audio_driver.spk.dma_buffer;
    i2s_cfg.dma_buf_rx = audio_driver.mic.dma_buffer;

    res = bus_i2s_init(&i2s_cfg);

    if(res != AUDIO_OK){
        DBG_AUD_ERR("Fail to configure i2s bus");
        return res;
    }

    bus_i2s_mclk(AUDIO_DEFAULT_MCLK_FREQ, AUDIO_DEFAULT_MCLK_SRC, audio_driver.codec->Config(CDC_GET_MCLK, 0));

    uint8_t cdc_addr = codec->Config(CDC_GET_I2C_ADDR, 0);

    if(!codec->Init(cdc_addr)){
        return AUDIO_ERROR_CODEC;
    }

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
  * @brief  audio codec loop
  * @param  none
  * @retval none
  */
audio_status_t audio_loop(void)
{
    delay_ms(10);
    return AUDIO_OK;
}

/**
 * @brief  This dma handler is called when dma has reach half transfer or
 * ended the transfer from memory to I2S peripheral.
 * @param  none
 * @retval none
 */
void DMA1_Channel3_IRQHandler(void)
{
    audio_stream_t *stream = &audio_driver.spk;
#ifdef AUDIO_SYNCHRONOUS_MODE
    if(stream->stage == STREAM_PAUSED){
        DMA1->clr = DMA1_HDT3_FLAG | DMA1_FDT3_FLAG;
    }else{
        if(DMA1->sts & DMA1_HDT3_FLAG){
            stream->stage = STREAM_FILL_FIRST;
            DMA1->clr = DMA1_HDT3_FLAG;
        }else if(DMA1->sts & DMA1_FDT3_FLAG){
            DMA1->clr = DMA1_FDT3_FLAG;
            stream->stage = STREAM_FILL_SECOND;
        }else{
            // it should not get here
            DMA1->clr = DMA1->sts;
        }
    }
#else
    uint16_t half_size = stream->nsamples;  // nsamples is half buffer on dma transfer
    uint16_t *pdst;

    if (dma_flag_get(DMA1_HDT3_FLAG) == SET)
    {
        pdst = stream->dma_buffer;
        dma_flag_clear(DMA1_HDT3_FLAG);
    }
    else if (dma_flag_get(DMA1_FDT3_FLAG) == SET)
    {
        pdst = stream->dma_buffer + half_size;
        dma_flag_clear(DMA1_FDT3_FLAG);
    }else{
        DMA1->clr = DMA1->sts;
        return;
    }

    switch (stream->stage)
    {
        case STREAM_INIT:
        case STREAM_FILL_FIRST:
            memset16(pdst, 0, half_size);
            break;

        case STREAM_FILL_SECOND:
            if (queue_count(&stream->queue) < half_size)
            {
                stream->stage = STREAM_INIT;
                DBG_AUD_WRN("Out stream buffer empty");
                break;
            }
            queue_pop(&stream->queue, pdst, half_size);
            break;
    }
#endif
}

/**
 * @brief  this function handles dma1 channel4 interrupt.
 * @param  none
 * @retval none
 */
void DMA1_Channel4_IRQHandler(void)
{
    audio_stream_t *stream = &audio_driver.mic;
#ifdef AUDIO_SYNCHRONOUS_MODE
    if(stream->stage == STREAM_PAUSED){
        DMA1->clr = DMA1_HDT4_FLAG | DMA1_FDT4_FLAG;
    }else{
        if(DMA1->sts & DMA1_HDT4_FLAG){
            stream->stage = STREAM_FILL_FIRST;
            DMA1->clr = DMA1_HDT4_FLAG;
        }else if(DMA1->sts & DMA1_FDT4_FLAG){
            DMA1->clr = DMA1_FDT4_FLAG;
            stream->stage = STREAM_FILL_SECOND;
        }else{
            DMA1->clr = DMA1->sts;
        }
    }
#else
    uint16_t *psrc;
    uint16_t half_size = stream->nsamples;
    if (dma_flag_get(DMA1_HDT4_FLAG) == SET)
    {
        psrc = stream->dma_buffer + half_size;
        dma_flag_clear(DMA1_HDT4_FLAG);
    }
    else if (dma_flag_get(DMA1_FDT4_FLAG) == SET)
    {
        psrc = stream->dma_buffer;
        dma_flag_clear(DMA1_FDT4_FLAG);
    }else{
        DMA1->clr = DMA1->sts;
        return;
    }

    switch (stream->stage)
    {
        case STREAM_INIT:
        case STREAM_FILL_FIRST:
        memset16(psrc, 0xCCCC, half_size);
        break;

        case STREAM_FILL_SECOND:
        break;
    }

    if (queue_push(&stream->queue, psrc, half_size) != half_size){
        //stream->stage = STREAM_INIT;
        DBG_AUD_WRN("In stream buffer full");
    }
#endif
}
