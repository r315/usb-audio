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
#include "audio_conf.h"

#define MIC_BUFFER_SIZE   1024
#define SPK_BUFFER_SIZE   4096
#define DMA_BUFFER_SIZE   288

static audio_codec_t *audio_codec;
static uint16_t spk_dma_buffer[DMA_BUFFER_SIZE];
static uint16_t mic_dma_buffer[DMA_BUFFER_SIZE];
static uint16_t spk_buffer[SPK_BUFFER_SIZE];
static uint16_t mic_buffer[MIC_BUFFER_SIZE];

static audio_codec_t audio_nocodec;

static void bus_i2s_reset(void);
static audio_status bus_i2s_init(audio_codec_t *audio);

static void memset16(uint16_t *buffer, uint32_t set, uint32_t len)
{
    while(len--){
        *buffer++ = (uint16_t)set;
    }
}

/**
  * @brief  audio codec modify freq
  * @param  freq: freq (wm8988 microphone and speaker must as same freq)
  * @retval none
  */
static void audio_set_freq(uint32_t freq)
{
}


/**
  * @brief  audio codec set microphone freq
  * @param  freq: freq (wm8988 microphone and speaker must same freq)
  * @retval none
  */
void audio_set_mic_freq(uint32_t freq)
{
    if(audio_codec->freq != freq)
    {
        audio_codec->freq = freq;
        audio_set_freq(freq);
        bus_i2s_reset();        
        bus_i2s_init(audio_codec);
    }
}

/**
  * @brief  audio codec set speaker freq
  * @param  freq: freq(wm8988 microphone and speaker must same freq)
  * @retval none
  */
void audio_set_spk_freq(uint32_t freq)
{
    printf("%s :%lu\n", __func__, freq);
}


/**
  * @brief  audio codec speaker alt setting config
  * @param  none
  * @retval none
  */
void audio_spk_alt_setting(uint32_t alt_seting)
{
    //printf("%s :%lu\n", __func__, alt_seting);
}

/**
  * @brief  audio codec microphone alt setting config
  * @param  none
  * @retval none
  */
void audio_mic_alt_setting(uint32_t alt_seting)
{
    //printf("%s :%lu\n", __func__, alt_seting);
}

/**
  * @brief  audio codec set microphone mute
  * @param  mute: mute state
  * @retval none
  */
void audio_set_mic_mute(uint8_t mute)
{
    printf("%s :%d\n", __func__, mute);
}


/**
  * @brief  audio codec set speaker mute
  * @param  mute: mute state
  * @retval none
  */
void audio_set_spk_mute(uint8_t mute)
{
    printf("%s :%d\n", __func__, mute);
}


/**
  * @brief  audio codec set microphone volume
  * @param  volume: the new volume
  * @retval none
  */
void audio_set_mic_volume(uint16_t volume)
{
    printf("%s :%d\n", __func__, volume);
}

/**
  * @brief  audio codec set speaker volume
  * @param  volume: the new volume
  * @retval none
  */
void audio_set_spk_volume(uint16_t volume)
{
    printf("%s :%d\n", __func__, volume);
}

/**
  * @brief  codec speaker feedback
  * @param  feedback: data buffer
  * @retval feedback len
  */
uint8_t audio_spk_feedback(uint8_t *feedback)
{
    uint32_t feedback_value = (audio_codec->spk.freq);
    feedback_value = ((feedback_value/1000)<<14)|((feedback_value%1000)<<4);
    feedback[0] = (uint8_t)(feedback_value);
    feedback[1] = (uint8_t)(feedback_value >> 8);
    feedback[2] = (uint8_t)(feedback_value >> 16);
    
  return 3;
}

/**
  * @brief  callback from audio_class to write codec speaker write fifo
  * 
  * @param  data: data buffer
  * @param  len: data length
  * @retval none
  */
void audio_enqueue_data(uint8_t *data, uint32_t len)
{
    uint16_t ulen = len / 2, i;
    uint16_t *u16data = (uint16_t *)data;

    switch (audio_codec->spk.stage)
    {
        case 0:
        audio_codec->spk.woff = audio_codec->spk.roff = audio_codec->spk.buffer;
        audio_codec->spk.wtotal = audio_codec->spk.rtotal = 0;
        audio_codec->spk.stage = 1;
        audio_codec->spk.threshold = SPK_BUFFER_SIZE / 2;
        break;

        case 1:
        if (audio_codec->spk.wtotal >= SPK_BUFFER_SIZE / 2)
        {
            audio_codec->spk.stage = 2;
        }
        break;
        
        case 2:
        break;
    }
  
    for (i = 0; i < ulen; ++i)
    {
        *(audio_codec->spk.woff++) = *u16data++;
        if (audio_codec->spk.woff >= audio_codec->spk.end)
        {
            audio_codec->spk.woff = audio_codec->spk.buffer;
        }
    }

    audio_codec->spk.wtotal += ulen;
}

/**
  * @brief  callback from audio_class to get codec microphone data
  * 
  * @param  data: data buffer
  * @retval data len
  */
uint32_t audio_dequeue_data(uint8_t *buffer)
{
    // copy_buff((uint16_t *)buffer, audio_codec.mic_buffer + audio_codec.mic_hf_status, audio_codec.mic_rx_size);
    uint16_t len = audio_codec->mic.size << 1;
    uint16_t i;
    uint16_t *u16buf = (uint16_t *)buffer;

    switch (audio_codec->mic.stage)
    {
        case 0:
        audio_codec->mic.stage = 1;
        memset(buffer, 0, len);
        return len;

        case 1:
        if ((audio_codec->mic.wtotal - audio_codec->mic.rtotal) >= (MIC_BUFFER_SIZE / 2)){
            audio_codec->mic.stage = 2;
        }
        return len;
    }
    switch (audio_codec->mic.adj_stage)
    {
        case 0:
        break;

        case 1:
        if (audio_codec->mic.adj_count >= 4){
            len += 4;
            audio_codec->mic.adj_count -= 4;
        }
        break;

        case 2:
        if (audio_codec->mic.adj_count >= 4){
            len -= 4;
            audio_codec->mic.adj_count -= 4;
        }
        break;
    }

    for (i = 0; i < len / 2; ++i){
        *u16buf++ = *(audio_codec->mic.roff++);

        if (audio_codec->mic.roff >= audio_codec->mic.end){
            audio_codec->mic.roff = audio_codec->mic.buffer;
        }
    }

    if (audio_codec->mic.wtotal <= audio_codec->mic.rtotal)
    { // should not happen
        while (1)
            ;
    }
    audio_codec->mic.rtotal += len / 2;
    audio_codec->mic.delta += len / 2;

    return len;
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
  * @param  freq: audio sampling freq
  * @param  bitw_format: bit width
  * @retval error status
  */
static audio_status bus_i2s_init(audio_codec_t *audio)
{
    gpio_init_type gpio_init_struct;
    dma_init_type dma_init_struct;
    i2s_init_type i2s_init_struct;
    i2s_data_channel_format_type format;

    if(audio->bitw == AUDIO_BITW_16){
        format = I2S_DATA_16BIT_CHANNEL_16BIT;
    }else if(audio->bitw == AUDIO_BITW_32){    
        format = I2S_DATA_24BIT_CHANNEL_32BIT;
    }else{
        return AUDIO_ERROR_BITW;
    }

    crm_periph_clock_enable(I2S1_GPIO_CRM_CLK, TRUE);
    crm_periph_clock_enable(I2S2_GPIO_CRM_CLK, TRUE);
    crm_periph_clock_enable(CRM_DMA1_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_SPI1_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_SPI2_PERIPH_CLOCK, TRUE);

    audio->spk.freq = audio->freq;

    audio->spk.size = (audio->freq / 1000) * (audio->bitw / 8) * AUDIO_SPK_CHANEL_NUM / 2;
    audio->mic.size = (audio->freq / 1000) * (audio->bitw / 8) * AUDIO_MIC_CHANEL_NUM / 2;

    audio->mic.end = audio->mic.woff = audio->mic.roff = audio->mic.buffer;
    audio->spk.end = audio->spk.woff = audio->spk.roff = audio->spk.buffer;

    while(audio->mic.end < audio->mic.buffer + MIC_BUFFER_SIZE){
        audio->mic.end += audio->mic.size;
    }
    audio->mic.end -= audio->mic.size;

    while(audio->spk.end < audio->spk.buffer + SPK_BUFFER_SIZE){
        audio->spk.end += audio->spk.size;
    }
    audio->spk.end -= audio->spk.size;

    /* Config TX I2S1 */
    spi_i2s_reset(SPI1);
    i2s_default_para_init(&i2s_init_struct);
    i2s_init_struct.audio_protocol = I2S_AUDIO_PROTOCOL_PHILLIPS;
    i2s_init_struct.data_channel_format = format;
    i2s_init_struct.mclk_output_enable = FALSE;
    i2s_init_struct.audio_sampling_freq = (i2s_audio_sampling_freq_type)audio->freq;
    i2s_init_struct.clock_polarity = I2S_CLOCK_POLARITY_LOW;
    i2s_init_struct.operation_mode = (audio->mode == AUDIO_MODE_MASTER) ? I2S_MODE_MASTER_TX : I2S_MODE_SLAVE_TX;
    i2s_init(SPI1, &i2s_init_struct);

    /* Config RX I2S2 */
    spi_i2s_reset(SPI2);
    i2s_default_para_init(&i2s_init_struct);
    i2s_init_struct.audio_protocol = I2S_AUDIO_PROTOCOL_PHILLIPS;
    i2s_init_struct.data_channel_format = format;
    i2s_init_struct.mclk_output_enable = FALSE;
    i2s_init_struct.audio_sampling_freq = (i2s_audio_sampling_freq_type)audio->freq;
    i2s_init_struct.clock_polarity = I2S_CLOCK_POLARITY_LOW;
    i2s_init_struct.operation_mode = I2S_MODE_SLAVE_RX;
    i2s_init(SPI2, &i2s_init_struct);

     /* dma config */
    dma_reset(DMA1_CHANNEL3);
    dma_reset(DMA1_CHANNEL4);

    /* dma1 channel3: speaker i2s1 tx */
    dma_default_para_init(&dma_init_struct);
    dma_init_struct.buffer_size = audio->spk.size << 1;
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
    nvic_irq_enable(DMA1_Channel3_IRQn, 1, 0);

    /* dma1 channel4: microphone i2s2 rx */
    dma_default_para_init(&dma_init_struct);
    dma_init_struct.buffer_size = audio->mic.size << 1;
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
    nvic_irq_enable(DMA1_Channel4_IRQn, 2, 0);   

    /* Config gpio's */
    gpio_default_para_init(&gpio_init_struct);

    if(audio->mode == AUDIO_MODE_MASTER){
        /* i2s1 ck, ws, tx pins */
        gpio_init_struct.gpio_mode           = GPIO_MODE_MUX;
        gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_MODERATE;
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
  * @brief  mclk clock conifg
  * @param  none
  * @retval none
  */
void audio_cfg_mclk(uint32_t freq, uint32_t enable)
{

}
/**
  * @brief  audio codec init
  * @param  none
  * @retval error status
  */
audio_status audio_init(void)
{
    audio_codec = &audio_nocodec;

    audio_codec->freq = AUDIO_DEFAULT_FREQ;
    audio_codec->bitw = AUDIO_DEFAULT_BITW;
    audio_codec->mode = AUDIO_DEFAULT_MODE;

    memset16(spk_buffer, 0, SPK_BUFFER_SIZE);
    memset16(mic_buffer, 0, MIC_BUFFER_SIZE);

    audio_codec->spk.buffer = spk_buffer;
    audio_codec->mic.buffer = mic_buffer;
    audio_codec->mic.dma_buffer = mic_dma_buffer;
    audio_codec->spk.dma_buffer = spk_dma_buffer;

    audio_cfg_mclk(AUDIO_DEFAULT_MCLK_FREQ, AUDIO_DEFAULT_MCLK);

    return bus_i2s_init(audio_codec);
}

/**
  * @brief  audio codec loop
  * @param  none
  * @retval none
  */
audio_status audio_loop(void)
{
    return AUDIO_OK;
}

/**
 * @brief  this function handles dma1 channel3 interrupt.
 * @param  none
 * @retval none
 */
void DMA1_Channel3_IRQHandler(void)
{
    uint16_t half_size = audio_codec->spk.size;
    uint16_t *pdst;

    if (dma_flag_get(DMA1_HDT3_FLAG) == SET)
    {
        // copy_buff(audio_codec.spk_buffer, audio_codec.spk_tx_fifo + audio_codec.r_pos, half_size);
        pdst = spk_dma_buffer;
        dma_flag_clear(DMA1_HDT3_FLAG);
    }
    else if (dma_flag_get(DMA1_FDT3_FLAG) == SET)
    {
        // copy_buff(&audio_codec.spk_buffer[half_size], audio_codec.spk_tx_fifo + audio_codec.r_pos, half_size);
        pdst = spk_dma_buffer + half_size;
        dma_flag_clear(DMA1_FDT3_FLAG);
    }else{
        // it should not get here
        DMA1->clr = DMA1->sts;
        return;
    }

    switch (audio_codec->spk.stage)
    {
        case 0:
        case 1:
        memset(pdst, 0, half_size << 1);
        break;
        
        case 2:
        if (audio_codec->spk.wtotal >= audio_codec->spk.rtotal + SPK_BUFFER_SIZE)
        {
            while (1)
                ; // should not happen;
        }
        if (audio_codec->spk.rtotal >= audio_codec->spk.wtotal)
        {
            audio_codec->spk.stage = 0;
            audio_codec->spk.woff = audio_codec->spk.roff = audio_codec->spk.buffer;
            audio_codec->spk.rtotal = audio_codec->spk.wtotal = 0;
        }
        else
        {
            memcpy(pdst, audio_codec->spk.roff, half_size << 1);
            audio_codec->spk.roff += half_size;
            audio_codec->spk.rtotal += half_size;
            if (audio_codec->spk.roff >= audio_codec->spk.end)
            {
                audio_codec->spk.roff = audio_codec->spk.buffer;
            }
            if (++audio_codec->spk.calc == 256)
            {
                uint16_t delta = audio_codec->spk.wtotal - audio_codec->spk.rtotal;
                audio_codec->spk.calc = 0;
                if (delta < audio_codec->spk.threshold - half_size)
                {
                    audio_codec->spk.threshold -= half_size;
                    audio_codec->spk.freq += audio_codec->freq / 1024;
                }
                else if (delta > audio_codec->spk.threshold + half_size)
                {
                    audio_codec->spk.threshold += half_size;
                    audio_codec->spk.freq -= audio_codec->freq / 1024;
                }
                if (audio_codec->spk.rtotal > 0x20000000)
                {
                    audio_codec->spk.rtotal -= 0x10000000;
                    audio_codec->spk.wtotal -= 0x10000000;
                }
            }
        }
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
    uint16_t *psrc;
    uint16_t len = audio_codec->mic.size << 1;

    if (dma_flag_get(DMA1_HDT4_FLAG) == SET)
    {
        dma_flag_clear(DMA1_HDT4_FLAG);
        psrc = mic_dma_buffer;
    }
    else if (dma_flag_get(DMA1_FDT4_FLAG) == SET)
    {
        psrc = mic_dma_buffer + audio_codec->mic.size;
        dma_flag_clear(DMA1_FDT4_FLAG);
    }else{
        DMA1->clr = DMA1->sts;
        return;
    }

    if (audio_codec->mic.stage)
    {
        memcpy(audio_codec->mic.woff, psrc, len);
        audio_codec->mic.woff += len / 2;
        audio_codec->mic.wtotal += len / 2;
        if (audio_codec->mic.woff >= audio_codec->mic.end)
        {
            audio_codec->mic.woff = audio_codec->mic.buffer;
        }
        if (audio_codec->mic.stage == 2)
        {
            if (1024 == ++audio_codec->mic.calc)
            {
                uint32_t size_estimate = 1024 * audio_codec->mic.size;
                audio_codec->mic.calc = 0;
                if (audio_codec->mic.delta > size_estimate)
                {
                    audio_codec->mic.adj_count = audio_codec->mic.delta - size_estimate;
                    audio_codec->mic.adj_stage = 2;
                }
                else if (audio_codec->mic.delta < size_estimate)
                {
                    audio_codec->mic.adj_count = size_estimate - audio_codec->mic.delta;
                    audio_codec->mic.adj_stage = 1;
                }
                else
                {
                    audio_codec->mic.adj_count = 0;
                    audio_codec->mic.adj_stage = 0;
                }
                audio_codec->mic.delta = 0;
                if (audio_codec->mic.rtotal >= 0x80000000)
                {
                    audio_codec->mic.rtotal -= 0x80000000;
                    audio_codec->mic.wtotal -= 0x80000000;
                }
            }
            if (audio_codec->mic.wtotal >= audio_codec->mic.rtotal + MIC_BUFFER_SIZE)
            {
                audio_codec->mic.delta = audio_codec->mic.wtotal = audio_codec->mic.rtotal = 0;
                audio_codec->mic.woff = audio_codec->mic.roff = audio_codec->mic.buffer;
                audio_codec->mic.stage = 0;
                audio_codec->mic.adj_stage = 0;
                audio_codec->mic.adj_count = 0;
                audio_codec->mic.calc = 0;
            }
        }
    }
}
