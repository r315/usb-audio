#include <sys/times.h>
#include "board.h"
#include "audio_conf.h"
#include "cli_simple.h"
#include "at32f415_clock.h"
#include "i2c_application.h"
#include "usb_conf.h"
#include "usb_core.h"
#include "usbd_int.h"
#include "audio_class.h"
#include "audio_desc.h"
#include "debug.h"

#ifndef ENABLE_DBG_APP
#define DBG_TAG             "BOARD : "
#define DBG_BOARD_PRINT       DBG_PRINT
#define DBG_BOARD_INF(...)    DBG_INF(DBG_TAG __VA_ARGS__)
#define DBG_BOARD_ERR(...)    DBG_ERR(DBG_TAG __VA_ARGS__)
#else
#define DBG_BOARD_PRINT(...)
#define DBG_BOARD__ERR(...)
#define DBG_BOARD__INF(...)
#endif

static otg_core_type otg_core_struct;
static i2c_handle_type hi2cx;

static void usb_init(void);

/**
 * @brief
 *
 */
void board_init(void)
{
	SystemInit();
    system_clock_config();
	system_core_clock_update();

	SysTick_Config((SystemCoreClock / 1000) - 1);

    CRM->ctrl_bit.hicktrim = HICK_TRIM; // this should be different for each board

#ifdef AUDIO_SYNCHRONOUS_MODE
    NVIC_SetPriority(OTG_IRQ, 3);
    NVIC_SetPriority(DMA1_Channel3_IRQn, 1);
    NVIC_SetPriority(DMA1_Channel4_IRQn, 1);
#else
    //TODO priority fot asynchronous mode
#endif
    serial_init();

    usb_init();

    /* i2c init */
    hi2cx.i2cx = I2Cx_PORT;
    i2c_config(&hi2cx);

    LED1_PIN_INIT;
}

#if (USE_TIMER_SYSTICK == 1)
#else
static volatile uint32_t ticms;

void SysTick_Handler(void){
    ticms++;
}

void delay_ms(uint32_t ms){
    volatile uint32_t end = ticms + ms;
    while (ticms < end){ }
}

uint32_t ElapsedTicks(uint32_t start_ticks){
	int32_t delta = GetTick() - start_ticks;
    return (delta < 0) ? -delta : delta;
}

inline uint32_t GetTick(void)
{
    return ticms;
}

void usb_delay_ms(uint32_t ms)
{
  delay_ms(ms);
}
#endif

#ifdef USB_LOW_POWER_WAKUP
/**
  * @brief  usb low power wakeup interrupt config
  * @param  none
  * @retval none
  */
void usb_low_power_wakeup_config(void)
{
  exint_init_type exint_init_struct;

  exint_default_para_init(&exint_init_struct);

  exint_init_struct.line_enable = TRUE;
  exint_init_struct.line_mode = EXINT_LINE_INTERRUPUT;
  exint_init_struct.line_select = OTG_WKUP_EXINT_LINE;
  exint_init_struct.line_polarity = EXINT_TRIGGER_RISING_EDGE;
  exint_init(&exint_init_struct);

  nvic_irq_enable(OTG_WKUP_IRQ, 0, 0);
}
#endif

/**
  * @brief  this function config gpio.
  * @param  none
  * @retval none
  */
static void usb_gpio_config(void)
{
  gpio_init_type gpio_init_struct;

  crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);

  gpio_default_para_init(&gpio_init_struct);

  gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
  gpio_init_struct.gpio_out_type  = GPIO_OUTPUT_PUSH_PULL;

#ifdef USB_SOF_OUTPUT_ENABLE
  crm_periph_clock_enable(OTG_PIN_SOF_GPIO_CLOCK, TRUE);
  gpio_init_struct.gpio_pins = OTG_PIN_SOF;
  gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
  gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
  gpio_init(OTG_PIN_SOF_GPIO, &gpio_init_struct);
#endif

  /* otgfs use vbus pin */
#ifndef USB_VBUS_IGNORE
  gpio_init_struct.gpio_pins = OTG_PIN_VBUS;
  gpio_init_struct.gpio_pull = GPIO_PULL_DOWN;
  gpio_init_struct.gpio_mode = GPIO_MODE_INPUT;
  gpio_init(OTG_PIN_GPIO, &gpio_init_struct);
#endif
}

static void usb_init(void)
{
#ifdef USB_LOW_POWER_WAKUP
  usb_low_power_wakeup_config();
#endif
    /* enable otgfs clock */
    crm_periph_clock_enable(OTG_CLOCK, TRUE);

    /* select usb 48m clcok source */
    crm_usb_clock_source_select(CRM_USB_CLOCK_SOURCE_HICK);

    /* enable otgfs irq */
    NVIC_EnableIRQ(OTG_IRQ);
}

void disconnect_usb(void)
{
    usbd_disconnect(&otg_core_struct.dev);
}

void connect_usb(void)
{
    usbd_init(&otg_core_struct,
            USB_FULL_SPEED_CORE_ID,
            USB_ID,
            &audio_class_handler,
            &audio_desc_handler);

    usb_gpio_config();
}

/**
  * @brief  Override function for i2c peripheral initialization.
  * @param hi2c
  * @retval none
  */
void i2c_lowlevel_init(i2c_handle_type* hi2c)
{
  gpio_init_type gpio_init_structure;

  if(hi2c->i2cx == I2Cx_PORT)
  {
    /* i2c periph clock enable */
    crm_periph_clock_enable(I2Cx_CLK, TRUE);
    crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_IOMUX_PERIPH_CLOCK, TRUE);
    gpio_pin_remap_config(I2C1_MUX, TRUE);

    /* configure i2c pins: scl */
    gpio_init_structure.gpio_out_type       = GPIO_OUTPUT_OPEN_DRAIN;
    gpio_init_structure.gpio_pull           = GPIO_PULL_NONE;
    gpio_init_structure.gpio_mode           = GPIO_MODE_MUX;
    gpio_init_structure.gpio_drive_strength = GPIO_DRIVE_STRENGTH_MAXIMUM;

    gpio_init_structure.gpio_pins           = I2Cx_SCL_GPIO_PIN;
    gpio_init(I2Cx_SCL_GPIO_PORT, &gpio_init_structure);

    /* configure i2c pins: sda */
    gpio_init_structure.gpio_pins           = I2Cx_SDA_GPIO_PIN;
    gpio_init(I2Cx_SDA_GPIO_PORT, &gpio_init_structure);

    /* config i2c */
    i2c_init(hi2c->i2cx, I2C_FSMODE_DUTY_2_1, I2Cx_SPEED);

    i2c_own_address1_set(hi2c->i2cx, I2C_ADDRESS_MODE_7BIT, I2Cx_ADDRESS);
  }
}

uint32_t I2C_Master_Write(uint8_t device, const uint8_t* data, uint32_t len)
{
    i2c_status_type res = i2c_master_transmit(&hi2cx, device << 1, (uint8_t*)data, len, 1000);

    if(res != I2C_OK){
        DBG_BOARD_ERR("%s : %u", __func__, res);
        return 0;
    }

    return len;
}

uint32_t I2C_Master_Read(uint8_t device, uint8_t* data, uint32_t len)
{
    i2c_status_type res = i2c_master_receive(&hi2cx, device << 1, data, len, 1000);

    if(res != I2C_OK){
        DBG_BOARD_ERR("%s : %u", __func__, res);
        return 0;
    }

    return len;
}

uint32_t I2C_Master_Scan(uint8_t device)
{
    return i2c_master_scan_addr(&hi2cx, device << 1, 1000);
}

/**
  * @brief  audio codec i2s reset
  * @param  none
  * @retval none
  */
void bus_i2s_reset(void)
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
int bus_i2s_init(i2s_config_t *cfg)
{
    gpio_init_type gpio_init_struct;
    dma_init_type dma_init_struct;
    i2s_init_type i2s_init_struct;
    i2s_data_channel_format_type format;

    if(cfg->bitw == AUDIO_BITW_16){
        format = I2S_DATA_16BIT_CHANNEL_32BIT;
    }else if(cfg->bitw == AUDIO_BITW_32){
        format = I2S_DATA_32BIT_CHANNEL_32BIT;
    }else{
        return 1;
    }

    crm_periph_clock_enable(I2S1_GPIO_CRM_CLK, TRUE);
    crm_periph_clock_enable(I2S2_GPIO_CRM_CLK, TRUE);
    crm_periph_clock_enable(CRM_DMA1_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_SPI1_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_SPI2_PERIPH_CLOCK, TRUE);

    gpio_default_para_init(&gpio_init_struct);

    /* Config TX I2S1 */
    spi_i2s_reset(SPI1);
    i2s_default_para_init(&i2s_init_struct);
    i2s_init_struct.audio_protocol = I2S_AUDIO_PROTOCOL_MSB;
    //i2s_init_struct.audio_protocol = I2S_AUDIO_PROTOCOL_PHILLIPS;
    i2s_init_struct.data_channel_format = format;
    i2s_init_struct.audio_sampling_freq = (i2s_audio_sampling_freq_type)cfg->freq;
    i2s_init_struct.clock_polarity = I2S_CLOCK_POLARITY_LOW;
    i2s_init_struct.operation_mode = (cfg->mode == AUDIO_MODE_MASTER) ? I2S_MODE_MASTER_TX : I2S_MODE_SLAVE_TX;
    i2s_init_struct.mclk_output_enable = TRUE;
    i2s_init(SPI1, &i2s_init_struct);

    /* Config RX I2S2 */
    spi_i2s_reset(SPI2);
    i2s_default_para_init(&i2s_init_struct);
    i2s_init_struct.audio_protocol = I2S_AUDIO_PROTOCOL_MSB;
    //i2s_init_struct.audio_protocol = I2S_AUDIO_PROTOCOL_PHILLIPS;
    i2s_init_struct.data_channel_format = format;
    i2s_init_struct.audio_sampling_freq = (i2s_audio_sampling_freq_type)cfg->freq;
    i2s_init_struct.clock_polarity = I2S_CLOCK_POLARITY_LOW;
    i2s_init_struct.operation_mode = (cfg->mode == AUDIO_MODE_MASTER) ? I2S_MODE_MASTER_RX : I2S_MODE_SLAVE_RX;
    i2s_init_struct.mclk_output_enable = FALSE;
    i2s_init(SPI2, &i2s_init_struct);

     /* dma config */
    dma_reset(DMA1_CHANNEL3);
    dma_reset(DMA1_CHANNEL4);

    /* dma1 channel3: speaker i2s1 tx */
    dma_default_para_init(&dma_init_struct);
    dma_init_struct.buffer_size = cfg->dma_buf_tx_size << 1;   // use double buffering
    dma_init_struct.direction = DMA_DIR_MEMORY_TO_PERIPHERAL;
    dma_init_struct.memory_base_addr = (uint32_t)cfg->dma_buf_tx;
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
    dma_init_struct.buffer_size = cfg->dma_buf_rx_size << 1;
    dma_init_struct.direction = DMA_DIR_PERIPHERAL_TO_MEMORY;
    dma_init_struct.memory_base_addr = (uint32_t)cfg->dma_buf_rx;
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
    if(cfg->mode == AUDIO_MODE_MASTER){
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

    if(cfg->mode == AUDIO_MODE_MASTER){
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

    return 0;
}


/**
  * @brief  Config PA3 to output mclk using TMR2
  * @param  freq    Desired mclk frequency
  * @param  enable  Enable mclk output
  * @retval none
  */
void bus_i2s_mclk(uint32_t freq, uint8_t type, uint32_t enable)
{
    gpio_init_type gpio_init_struct;
    tmr_output_config_type tmr_oc_init_structure;
    uint16_t prescaler_value;
    crm_clocks_freq_type clocks;

    if(!enable){
        if(type){
            gpio_init_struct.gpio_pins = GPIO_PINS_3;
            gpio_init_struct.gpio_mode = GPIO_MODE_INPUT;
            gpio_init(GPIOA, &gpio_init_struct);
            crm_periph_reset(CRM_TMR2_PERIPH_CLOCK, TRUE);
        }else{
            gpio_init_struct.gpio_pins = I2S1_MCK_PIN;
            gpio_init_struct.gpio_mode = GPIO_MODE_INPUT;
            gpio_init(I2S1_MCK_GPIO, &gpio_init_struct);
            SPI1->i2sclk_bit.i2smclkoe = FALSE;
        }
        return;
    }

    if(!type){
        gpio_init_struct.gpio_pins = I2S1_MCK_PIN;
        gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
        gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_MAXIMUM;
        gpio_init(I2S1_MCK_GPIO, &gpio_init_struct);
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
 * @brief
 *
 */
void SW_Reset(void)
{
    NVIC_SystemReset();
}

/**
 * @brief
 *
 */
void __debugbreak(void){
	 asm volatile
    (
        "bkpt #01 \n"
    );
}

/**
  * @brief  this function handles otgfs interrupt.
  * @param  none
  * @retval none
  */
void OTG_IRQ_HANDLER(void)
{
  usbd_irq_handler(&otg_core_struct);
}

#ifdef USB_LOW_POWER_WAKUP
/**
  * @brief  this function handles otgfs wakup interrupt.
  * @param  none
  * @retval none
  */
void OTG_WKUP_HANDLER(void)
{
  exint_flag_clear(OTG_WKUP_EXINT_LINE);
}
#endif
