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
#include "audio_conf.h"
#include "audio.h"
#include "audio_desc.h"
#include "board.h"
#include "debug.h"

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

#define HICK_TRIM           50 // TODO: check functionality

typedef enum stream_stage_e{
    STREAM_INIT = 0,
    STREAM_FILL_FIRST,
    STREAM_FILL_SECOND
}stream_stage_e;

static audio_driver_t audio_driver;
static uint16_t spk_dma_buffer[DMA_BUFFER_SIZE];
static uint16_t mic_dma_buffer[DMA_BUFFER_SIZE];
static uint16_t spk_buffer[SPK_BUFFER_SIZE];        // spk queue buffer for usb
static uint16_t mic_buffer[MIC_BUFFER_SIZE];        // mic queue buffer for usb

static void bus_i2s_reset(void);
static audio_status_t bus_i2s_init(audio_driver_t *audio);

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

static void queue_flush(audio_queue_t *q)
{
    q->read = q->write = q->start;
}

static uint16_t queue_push(audio_queue_t *q, const uint16_t *data, uint16_t len) {
    uint16_t count = 0;
    while(count < len){
        *q->write++ = *data++;
        if(q->write >= q->end){
            q->write = q->start;
        }
        count++;
        if(q->write == q->read){
            break;
        }
    }
    return count;
}

static uint16_t queue_pop(audio_queue_t *q, uint16_t *data, uint16_t len) {
    uint16_t count = 0;
    while(count < len){
        *data++ = *q->read++;
        if(q->read >= q->end){
            q->read = q->start;
        }
        count++;
        if(q->read == q->write){
            break;
        }
    }
    return count;
}

static uint16_t queue_count(audio_queue_t *q)
{
    return (q->write >= q->read)
        ? (q->write - q->read)
        : ((q->end - q->start) - (q->read - q->write));
}

/**
 * @brief Returns maximum number of elements queue can hold.
 * This differs of q->size that holds the maximum elements
 * of buffer
 *
 * @param q
 * @return uint16_t
 */
static uint16_t queue_size(audio_queue_t *q)
{
    return (q->end - q->start);
}

/**
  * @brief  audio codec modify freq
  * @param  freq: freq
  * @retval none
  */
void audio_set_freq(uint32_t freq)
{
    if(audio_driver.freq != freq)
    {
        audio_driver.freq = freq;
        audio_driver.codec->Disable();
        bus_i2s_reset();
        bus_i2s_init(&audio_driver);
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
    DBG_AUD_INF("%s :%lu", __func__, alt_seting);
}

/**
  * @brief  audio codec microphone alt setting config
  * @param  none
  * @retval none
  */
void audio_mic_alt_setting(uint32_t alt_seting)
{
    DBG_AUD_INF("%s :%lu", __func__, alt_seting);
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
  * @param  data: data buffer
  * @param  len: data length
  * @retval none
  */
void audio_enqueue_data(uint8_t *data, uint32_t len)
{
    audio_stream_t *stream = &audio_driver.spk;
    uint16_t nsamples = len >> 1;

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
                DBG_AUD_INF("Out stream buffer half full");
            }
            break;

        case STREAM_FILL_SECOND:
            if (queue_count(&stream->queue) < stream->nsamples){
                DBG_AUD_WRN("usb is anable to keep up");

            }
            break;
    }

    if(queue_push(&stream->queue, (uint16_t*)data, nsamples) != nsamples){
        DBG_AUD_WRN("Out stream buffer full");
    }
}

/**
  * @brief  callback from audio_class to get codec microphone data
  *
  * @param  data: data buffer
  * @retval data len
  */
 uint32_t audio_dequeue_data(uint8_t *buffer)
 {
    audio_stream_t *stream = &audio_driver.mic;
    uint16_t nsamples = stream->nsamples;

    return 0;

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
            stream->stage = STREAM_FILL_SECOND;
            DBG_AUD_INF("In stream half full");
            break;
    }

    return nsamples << 1;
}

static void audio_stream_init(audio_stream_t *stream, uint32_t freq, uint8_t bitw)
{
    // Calculate the number of samples per millisecond unit
    stream->nsamples = ((freq / 1000) * (bitw / 8) * stream->nchannels) / sizeof(*stream->queue.start);
    // Find queue end for miliseconds units, this varies depending frequency, channels and bit width
    stream->queue.end = stream->queue.start;
    while(stream->queue.end + stream->nsamples < stream->queue.start + stream->queue.size){
        stream->queue.end += stream->nsamples;
    }
}

/**
  * @brief  audio codec i2s reset
  * @param  none
  * @retval none
  */
static void bus_i2s_reset(void)
{
    i2s_enable(SPI1, FALSE);
    i2s_enable(SPI2, FALSE);
    dma_channel_enable(DMA1_CHANNEL3, FALSE);
    dma_channel_enable(DMA1_CHANNEL4, FALSE);
}

/**
  * @brief  audio codec i2s init
  * @param  audio
  * @retval error status
  */
static audio_status_t bus_i2s_init(audio_driver_t *audio)
{
    gpio_init_type gpio_init_struct;
    dma_init_type dma_init_struct;
    i2s_init_type i2s_init_struct;
    i2s_data_channel_format_type format;

    if(audio->bitw == AUDIO_BITW_16){
        format = I2S_DATA_16BIT_CHANNEL_32BIT;
    }else if(audio->bitw == AUDIO_BITW_32){
        format = I2S_DATA_32BIT_CHANNEL_32BIT;
    }else{
        return AUDIO_ERROR_BITW;
    }

    crm_periph_clock_enable(I2S1_GPIO_CRM_CLK, TRUE);
    crm_periph_clock_enable(I2S2_GPIO_CRM_CLK, TRUE);
    crm_periph_clock_enable(CRM_DMA1_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_SPI1_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_SPI2_PERIPH_CLOCK, TRUE);

    CRM->ctrl_bit.hicktrim = HICK_TRIM; // this should be different for each board

    audio_stream_init(&audio->spk, audio->freq, audio->bitw);
    audio_stream_init(&audio->mic, audio->freq, audio->bitw);

    gpio_default_para_init(&gpio_init_struct);

    /* Config TX I2S1 */
    spi_i2s_reset(SPI1);
    i2s_default_para_init(&i2s_init_struct);
    i2s_init_struct.audio_protocol = I2S_AUDIO_PROTOCOL_MSB;
    //i2s_init_struct.audio_protocol = I2S_AUDIO_PROTOCOL_PHILLIPS;
    i2s_init_struct.data_channel_format = format;
    i2s_init_struct.audio_sampling_freq = (i2s_audio_sampling_freq_type)audio->freq;
    i2s_init_struct.clock_polarity = I2S_CLOCK_POLARITY_LOW;
    i2s_init_struct.operation_mode = (audio->mode == AUDIO_MODE_MASTER) ? I2S_MODE_MASTER_TX : I2S_MODE_SLAVE_TX;
    if(audio->codec->Config(CDC_GET_MCLK, 0)){
        gpio_init_struct.gpio_pins = I2S1_MCK_PIN;
        gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
        gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_MAXIMUM;
        i2s_init_struct.mclk_output_enable = TRUE;
        gpio_init(I2S1_MCK_GPIO, &gpio_init_struct);
    }else{
        i2s_init_struct.mclk_output_enable = FALSE;
    }
    i2s_init(SPI1, &i2s_init_struct);

    /* Config RX I2S2 */
    spi_i2s_reset(SPI2);
    i2s_default_para_init(&i2s_init_struct);
    i2s_init_struct.audio_protocol = I2S_AUDIO_PROTOCOL_MSB;
    //i2s_init_struct.audio_protocol = I2S_AUDIO_PROTOCOL_PHILLIPS;
    i2s_init_struct.data_channel_format = format;
    i2s_init_struct.mclk_output_enable = FALSE;
    i2s_init_struct.audio_sampling_freq = (i2s_audio_sampling_freq_type)audio->freq;
    i2s_init_struct.clock_polarity = I2S_CLOCK_POLARITY_LOW;
    i2s_init_struct.operation_mode = (audio->mode == AUDIO_MODE_MASTER) ? I2S_MODE_MASTER_RX : I2S_MODE_SLAVE_RX;
    i2s_init(SPI2, &i2s_init_struct);

     /* dma config */
    dma_reset(DMA1_CHANNEL3);
    dma_reset(DMA1_CHANNEL4);

    /* dma1 channel3: speaker i2s1 tx */
    dma_default_para_init(&dma_init_struct);
    dma_init_struct.buffer_size = audio->spk.nsamples << 1;   // use double buffering
    dma_init_struct.direction = DMA_DIR_MEMORY_TO_PERIPHERAL;
    dma_init_struct.memory_base_addr = (uint32_t)audio->spk.dma_buffer;
    dma_init_struct.memory_data_width = DMA_MEMORY_DATA_WIDTH_HALFWORD;
    dma_init_struct.memory_inc_enable = TRUE;
    dma_init_struct.peripheral_base_addr = (uint32_t)I2S1_DT_ADDRESS;
    dma_init_struct.peripheral_data_width = DMA_PERIPHERAL_DATA_WIDTH_HALFWORD;
    dma_init_struct.peripheral_inc_enable = FALSE;
    dma_init_struct.priority = DMA_PRIORITY_HIGH;
    dma_init_struct.loop_mode_enable = TRUE;
    dma_init(DMA1_CHANNEL3, &dma_init_struct);
    dma_interrupt_enable(DMA1_CHANNEL3, DMA_FDT_INT, TRUE);
    dma_interrupt_enable(DMA1_CHANNEL3, DMA_HDT_INT, TRUE);
    NVIC_EnableIRQ(DMA1_Channel3_IRQn);

    /* dma1 channel4: microphone i2s2 rx */
    dma_default_para_init(&dma_init_struct);
    dma_init_struct.buffer_size = audio->mic.nsamples << 1;
    dma_init_struct.direction = DMA_DIR_PERIPHERAL_TO_MEMORY;
    dma_init_struct.memory_base_addr = (uint32_t)audio->mic.dma_buffer;
    dma_init_struct.memory_data_width = DMA_MEMORY_DATA_WIDTH_HALFWORD;
    dma_init_struct.memory_inc_enable = TRUE;
    dma_init_struct.peripheral_base_addr = (uint32_t)I2S2_DT_ADDRESS;
    dma_init_struct.peripheral_data_width = DMA_PERIPHERAL_DATA_WIDTH_HALFWORD;
    dma_init_struct.peripheral_inc_enable = FALSE;
    dma_init_struct.priority = DMA_PRIORITY_HIGH;
    dma_init_struct.loop_mode_enable = TRUE;
    dma_init(DMA1_CHANNEL4, &dma_init_struct);
    dma_interrupt_enable(DMA1_CHANNEL4, DMA_FDT_INT, TRUE);
    dma_interrupt_enable(DMA1_CHANNEL4, DMA_HDT_INT, TRUE);
    NVIC_EnableIRQ(DMA1_Channel4_IRQn);

    /* Config gpio's */
    if(audio->mode == AUDIO_MODE_MASTER){
        /* i2s1 ck, ws, tx pins */
        gpio_init_struct.gpio_mode           = GPIO_MODE_MUX;
        gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_MAXIMUM;
        gpio_init_struct.gpio_pull           = GPIO_PULL_UP;
        gpio_init_struct.gpio_pins           = I2S1_WS_PIN | I2S1_SD_PIN | I2S1_CK_PIN;
        gpio_init(I2S1_GPIO, &gpio_init_struct);
    }else{
        /* i2s1 ws pins */
        gpio_init_struct.gpio_pull           = GPIO_PULL_UP;
        gpio_init_struct.gpio_mode           = GPIO_MODE_INPUT;
        gpio_init_struct.gpio_pins           = I2S1_WS_PIN;
        gpio_init(I2S1_GPIO, &gpio_init_struct);

        /* i2s1 ck pins */
        gpio_init_struct.gpio_pull           = GPIO_PULL_DOWN;
        gpio_init_struct.gpio_mode           = GPIO_MODE_INPUT;
        gpio_init_struct.gpio_pins           = I2S1_CK_PIN;
        gpio_init(I2S1_GPIO, &gpio_init_struct);

        /* i2s1 sd pins slave tx */
        gpio_init_struct.gpio_pull           = GPIO_PULL_UP;
        gpio_init_struct.gpio_mode           = GPIO_MODE_MUX;
        gpio_init_struct.gpio_pins           = I2S1_SD_PIN;
        gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_MODERATE;
        gpio_init(I2S1_GPIO, &gpio_init_struct);
    }

    if(audio->mode == AUDIO_MODE_MASTER){
        /* i2s2 ws, ck pins */
        gpio_init_struct.gpio_mode           = GPIO_MODE_MUX;
        gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_MODERATE;
        gpio_init_struct.gpio_pins           = I2S2_WS_PIN | I2S2_CK_PIN;
        gpio_init(I2S2_GPIO, &gpio_init_struct);

        /* i2s2 sd pins slave rx */
        gpio_init_struct.gpio_pull           = GPIO_PULL_UP;
        gpio_init_struct.gpio_mode           = GPIO_MODE_INPUT;
        gpio_init_struct.gpio_pins           = I2S2_SD_PIN;
        gpio_init(I2S2_GPIO, &gpio_init_struct);
    }else{
        /* i2s2 ws pins */
        gpio_init_struct.gpio_pull           = GPIO_PULL_UP;
        gpio_init_struct.gpio_mode           = GPIO_MODE_INPUT;
        gpio_init_struct.gpio_pins           = I2S2_WS_PIN;
        gpio_init(I2S2_GPIO, &gpio_init_struct);

        /* i2s2 ck pins */
        gpio_init_struct.gpio_pull           = GPIO_PULL_DOWN;
        gpio_init_struct.gpio_mode           = GPIO_MODE_INPUT;
        gpio_init_struct.gpio_pins           = I2S2_CK_PIN;
        gpio_init(I2S2_GPIO, &gpio_init_struct);

        /* i2s2 sd pins slave rx */
        gpio_init_struct.gpio_pull           = GPIO_PULL_UP;
        gpio_init_struct.gpio_mode           = GPIO_MODE_INPUT;
        gpio_init_struct.gpio_pins           = I2S2_SD_PIN;
        gpio_init(I2S2_GPIO, &gpio_init_struct);
    }

    /* Start I2S */
    spi_i2s_dma_transmitter_enable(SPI1, TRUE);
    spi_i2s_dma_receiver_enable(SPI2, TRUE);
    i2s_enable(SPI1, TRUE);
    i2s_enable(SPI2, TRUE);

    dma_channel_enable(DMA1_CHANNEL3, TRUE);
    dma_channel_enable(DMA1_CHANNEL4, TRUE);

    return AUDIO_OK;
}

/**
  * @brief  Config PA3 to output mclk using TMR2
  * @param  freq    Desired mclk frequency
  * @param  enable  Enable mclk output
  * @retval none
  */
void audio_cfg_mclk(uint32_t freq, uint32_t enable)
{
    gpio_init_type gpio_init_struct;
    tmr_output_config_type tmr_oc_init_structure;
    uint16_t prescaler_value;
    crm_clocks_freq_type clocks;

    if(!enable){
        gpio_init_struct.gpio_pins = GPIO_PINS_3;
        gpio_init_struct.gpio_mode = GPIO_MODE_INPUT;
        gpio_init(GPIOA, &gpio_init_struct);

        crm_periph_reset(CRM_TMR2_PERIPH_CLOCK, TRUE);
        return;
    }

    crm_clocks_freq_get(&clocks);

    prescaler_value = (uint16_t)(clocks.apb1_freq / freq) - 1;

    crm_periph_clock_enable(CRM_TMR2_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_IOMUX_PERIPH_CLOCK, TRUE);

    gpio_default_para_init(&gpio_init_struct);

    gpio_init_struct.gpio_pins = GPIO_PINS_3;
    gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
    gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_MAXIMUM;
    gpio_init(GPIOA, &gpio_init_struct);

    tmr_base_init(TMR2, 1, prescaler_value);
    tmr_cnt_dir_set(TMR2, TMR_COUNT_UP);
    tmr_clock_source_div_set(TMR2, TMR_CLOCK_DIV1);

    tmr_output_default_para_init(&tmr_oc_init_structure);
    tmr_oc_init_structure.oc_mode = TMR_OUTPUT_CONTROL_PWM_MODE_A;
    tmr_oc_init_structure.oc_idle_state = FALSE;
    tmr_oc_init_structure.oc_polarity = TMR_OUTPUT_ACTIVE_HIGH;
    tmr_oc_init_structure.oc_output_state = TRUE;
    tmr_output_channel_config(TMR2, TMR_SELECT_CHANNEL_4, &tmr_oc_init_structure);
    tmr_channel_value_set(TMR2, TMR_SELECT_CHANNEL_4, 1);
    tmr_output_channel_buffer_enable(TMR2, TMR_SELECT_CHANNEL_4, TRUE);

    /* tmr enable counter */
    tmr_counter_enable(TMR2, TRUE);
    tmr_output_enable(TMR2, TRUE);
}

/**
  * @brief  audio codec init
  * @param  none
  * @retval error status
  */
audio_status_t audio_init(const audio_codec_t *codec)
{
    audio_status_t res;

    if(!codec){
        return AUDIO_ERROR_CODEC;
    }

    if(audio_driver.codec == NULL){
        audio_driver.freq = AUDIO_DEFAULT_FREQ;
        audio_driver.bitw = AUDIO_DEFAULT_BITW;
        audio_driver.mode = AUDIO_DEFAULT_MODE;
    }

    audio_driver.codec = codec;

    memset16(spk_buffer, 0, SPK_BUFFER_SIZE);
    memset16(mic_buffer, 0, MIC_BUFFER_SIZE);

    audio_driver.spk.queue.start = spk_buffer;
    audio_driver.mic.queue.start = mic_buffer;
    audio_driver.mic.dma_buffer = mic_dma_buffer;
    audio_driver.spk.dma_buffer = spk_dma_buffer;

    audio_driver.spk.queue.size = SPK_BUFFER_SIZE;
    audio_driver.mic.queue.size = MIC_BUFFER_SIZE;

    audio_driver.spk.nchannels = AUDIO_SPK_CHANEL_NUM;
    audio_driver.mic.nchannels = AUDIO_MIC_CHANEL_NUM;

    audio_driver.spk.stage = STREAM_INIT;
    audio_driver.mic.stage = STREAM_INIT;

    res = bus_i2s_init(&audio_driver);

    if(res != AUDIO_OK){
        DBG_AUD_ERR("Fail to configure i2s bus");
        return res;
    }

    uint8_t cdc_addr = audio_driver.codec->Config(CDC_GET_I2C_ADDR, 0);

    if(!audio_driver.codec->Init(cdc_addr)){
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
        // it should not get here
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
}

/**
 * @brief  this function handles dma1 channel4 interrupt.
 * @param  none
 * @retval none
 */
void DMA1_Channel4_IRQHandler(void)
{
    audio_stream_t *stream = &audio_driver.mic;
    uint16_t half_size = stream->nsamples;
    uint16_t *psrc;

    if (dma_flag_get(DMA1_HDT4_FLAG) == SET)
    {
        psrc = stream->dma_buffer;
        dma_flag_clear(DMA1_HDT4_FLAG);
    }
    else if (dma_flag_get(DMA1_FDT4_FLAG) == SET)
    {
        psrc = stream->dma_buffer + half_size;
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
        //DBG_AUD_WRN("In stream buffer full");
    }
}
