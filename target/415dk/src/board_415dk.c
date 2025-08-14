#include <sys/times.h>
#include "board.h"
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
    nvic_irq_enable(OTG_IRQ, 0, 0);
    nvic_priority_group_config(NVIC_PRIORITY_GROUP_4);
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
 * @brief
 *
 */
void board_init(void)
{
	SystemInit();
    system_clock_config();
	system_core_clock_update();

	SysTick_Config((SystemCoreClock / 1000) - 1);

    serial_init();

    usb_init();

    /* i2c init */
    hi2cx.i2cx = I2Cx_PORT;
    i2c_config(&hi2cx);

    LED1_PIN_INIT;
}

void SW_Reset(void)
{
    NVIC_SystemReset();
}

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
