/**
  **************************************************************************
  * @file     main.c
  * @brief    main program
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

#include <string.h>
#include <stdio.h>
#include "board.h"
#include "at32f415_clock.h"
#include "usb_conf.h"
#include "usb_core.h"
#include "usbd_int.h"
#include "audio_class.h"
#include "audio_desc.h"
#include "audio.h"
#include "cli_simple.h"
#include "i2c_application.h"
#include "tas2563.h"
//#include "max98374.h"
#include "ak4619.h"
#include "amux.h"

typedef struct {
    const char *name;
    const audio_codec_t *cdc;
}cdc_list_t;

static const cdc_list_t codecs [] = {
    {"none", NULL},
    {"ak4619", &ak4619},
    {"tas2563", &tas2563}
};

static uint8_t user_button_state;
static const audio_codec_t *codec;

static i2c_handle_type hi2cx;
static otg_core_type otg_core_struct;

#ifdef __AUDIO_HID_CLASS_H
ALIGNED_HEAD  uint8_t report_buf[USBD_AUHID_IN_MAXPACKET_SIZE] ALIGNED_TAIL;
#endif

/* usb global struct define */
void usb_clock48m_select(usb_clk48_s clk_s);
void usb_gpio_config(void);
void usb_low_power_wakeup_config(void);

#if ENABLE_CLI
static int codecCmd(int argc, char **argv) 
{
    if( !strcmp("help", argv[1]) || argc < 2){
        printf("\tvol\n");
        printf("\trr <reg>\n");
        printf("\twr <reg> <val>\n");
        printf("\tinit\n");
        printf("\tenable\n");
        printf("\tdisable\n");
        printf("\ttdm\n");
        printf("\ti2s\n");
        printf("\tscan\n");
        printf("\tselect\n");
        return CLI_OK;
    }

    if( !strcmp("select", argv[1])){
        if(argc < 3){
            printf("Available codecs:\n");
            for(uint8_t i = 0; i < sizeof(codecs)/sizeof(cdc_list_t); i++){
                printf("\t%s %c\n", codecs[i].name, (codecs[i].cdc == codec) ? '*' : ' ');
            }
        }else{            
            for(uint8_t idx = 0; idx < sizeof(codecs)/sizeof(cdc_list_t); idx++){
                if(!strcmp(codecs[idx].name, argv[2])){
                    codec = codecs[idx].cdc;
                }
            }
        }
        return CLI_OK;
    }

    if( !strcmp("scan", argv[1])){
        printf("\n   ");
        
        for(int i = 0; i < 16; i++){
            printf("%02X ", i);
        }           
        
        uint8_t dummy;
        
        for(int i = 0; i < 128; i++){
            if( (i & 15) == 0) 
                printf("\n%02X ", i & 0xF0);
            
            if(i2c_master_transmit(&hi2cx, (i << 1), &dummy, 1, 1000) == I2C_OK){
                printf("%02X ", i);
            }else{
                printf("-- ");
            }
            
            delay_ms(1);
        }
        putchar('\n');

        return CLI_OK;
    }

    if(codec == NULL){
        printf("No codec selected\n");
        return CLI_OK;
    }

    if( !strcmp("vol", argv[1])){
        int32_t val;
        if(CLI_Ia2i(argv[2], &val)){
            audio_set_spk_volume(val);
        }        
        return CLI_OK;
    }
  
    if( !strcmp("rr", argv[1])){
        uint32_t val;
        if(CLI_Ha2i(argv[2], &val)){
            codec->ReadReg(val, (uint8_t*)&val);
            printf("%02X\n", (uint8_t)val);
        }        
        return CLI_OK;
    }

    if( !strcmp("wr", argv[1])){
        uint32_t val, val2;
        if(CLI_Ha2i(argv[2], &val)){
            if(CLI_Ha2i(argv[3], &val2)){
                codec->WriteReg(val, val2);
            }
        }        
        return CLI_OK;
    }

    if( !strcmp("init", argv[1])){
        audio_deinit();
        printf("%d\n", audio_init(codec));
        return CLI_OK;
    }

    if( !strcmp("enable", argv[1])){
        codec->Enable();   
        return CLI_OK;
    }

    if( !strcmp("disable", argv[1])){
        codec->Disable(); 
        return CLI_OK;
    }

    if( !strcmp("tdm", argv[1])){
        codec->Config(CDC_DEV_DAI, CDC_CFG_DAI_TDM);
        return CLI_OK;
    }

    if( !strcmp("i2s", argv[1])){
        codec->Config(CDC_DEV_DAI, CDC_CFG_DAI_I2S);
        return CLI_OK;
    }   

    return CLI_BAD_PARAM;
}

#ifdef __AUDIO_HID_CLASS_H
static int hidCmd(int argc, char **argv)
{
    user_button_state = USER_BUTTON;
    return CLI_OK;
}
#endif

static int audioCmd(int argc, char **argv)
{
    if(!strcmp("mclk", argv[0])){
        uint32_t val;
        if(CLI_Ha2i(argv[1], &val)){
            audio_cfg_mclk(AUDIO_DEFAULT_MCLK_FREQ, val & 1);
        }
    }

    if(!strcmp("freq", argv[0])){
        int32_t val;
        if(CLI_Ia2i(argv[1], &val)){
            audio_set_freq(val);
        }
    }
    
    if(!strcmp("disable", argv[0])){
        audio_deinit();
        usbd_disconnect(&otg_core_struct.dev);
    }
    return CLI_OK;
}

static int resetCmd(int argc, char **argv)
{
    SW_Reset();
    return CLI_OK;
}

static int clearCmd(int argc, char **argv)
{
    CLI_Clear();
    return CLI_OK;
}

/**
 * Note: Index of register to be read is sent on buf[0]
*/
static i2c_status_type readMux(i2c_handle_type *handle, uint32_t addr, uint8_t *buf, uint32_t count)
{
    if(i2c_master_transmit(handle, addr, buf, 1, 1000) == I2C_OK){
        if(i2c_master_receive(handle, addr, buf, count, 1000) != I2C_OK){
            printf("Failed to read");
        }else{
            for(int i = 0; i < count; i ++){
                if( (i & 15) == 0) 
                    printf("\n%02X: ", i & 0xF0);                
                printf("%02X ", buf[i]);
            }        
        }
        putchar('\n');
        return I2C_OK;
    }

    printf("Failed to write reg address\n");

    return I2C_ERR_ACKFAIL;   
}

static int muxCmd(int argc, char **argv)
{
    static uint8_t mux_addr = AMUX_I2C_ADDR;
    uint32_t count = 16 + 128;
    uint32_t value;
    uint8_t regs_buf[count] __attribute__ ((aligned (4)));

    if( !strcmp("help", argv[1]) || argc < 2){
        printf("\tregs\n");
        printf("\trr <reg>\n");
        printf("\twr <reg> <val>\n");
        printf("\tmute <all | name> <1|0>, Name cxsy\n");
        printf("\treset\n");        
        printf("\troute <in name> <out name> <1|0>\n");
        return CLI_OK;
    }

    if( !strcmp("regs", argv[1])){
        regs_buf[0] = 0;
        if(readMux(&hi2cx, mux_addr, regs_buf, count) == I2C_OK)
            return CLI_OK;
    }

    if( !strcmp("wr", argv[1])){
        if(CLI_Ha2i(argv[2], &value)){
            regs_buf[0] = value;
            if(CLI_Ha2i(argv[3], &value)){
                regs_buf[1] = value;
                if(i2c_master_transmit(&hi2cx, mux_addr, regs_buf, 2, 1000) == I2C_OK){
                    return CLI_OK;
                }
            }
        }
    }

    if( !strcmp("rr", argv[1])){
        if(CLI_Ha2i(argv[2], (uint32_t*)&regs_buf)){
            if(CLI_Ha2i(argv[3], &count)){
                if(readMux(&hi2cx, mux_addr, regs_buf, count) == I2C_OK)
                return CLI_OK;
            }
        }
    }

    /**
     * mute <all, cxsy 1|0>
    */
    if(!strcmp("mute", argv[1])){
        if(!strcmp("all", argv[2])){
            amux_MuteAll();
            return CLI_OK;
        }

        amux_Mute(argv[2][1] - '0', argv[2][3] - '0', argv[3][0] - '0');
        return CLI_OK;
    }

    if( !strcmp("init", argv[1])){
        amux_Init();
        return CLI_OK;
    }

    if( !strcmp("addr", argv[1])){
        if(CLI_Ha2i(argv[2], &value)){
            mux_addr = value;
            return CLI_OK;
        }
    }

    /**
     * route <cxsy> <cxsy> 1|0
    */
    if( !strcmp("route", argv[1])){

        if(argc < 5){
            return CLI_MISSING_ARGS;
        }        

        amux_Route(argv[2][1] - '0', argv[2][3] - '0', 
                   argv[3][1] - '0', argv[3][3] - '0', 
                  (argv[4][0] == '1') ? 1 : 0);

        return CLI_OK;
    }

    return CLI_BAD_PARAM;
}

cli_command_t cli_cmds [] = {
    {"help", ((int (*)(int, char**))CLI_Commands)},
    {"cdc", codecCmd},
    {"reset", resetCmd},
    {"clear", clearCmd},
    {"mclk", audioCmd},
    {"freq", audioCmd},
    {"disable", audioCmd},
    {"mux", muxCmd},
};
#endif

static int at32_button_press(void)
{
    if(user_button_state){
        user_button_state = 0;
        return USER_BUTTON;
    }

    return 0;
}

/**
  * @brief  main function.
  * @param  none
  * @retval none
  */
int main(void)
{
  board_init();

  nvic_priority_group_config(NVIC_PRIORITY_GROUP_4);

  system_clock_config();

#if ENABLE_CLI
  serial_init();

  CLI_Init("Audio >");
  CLI_RegisterCommand(cli_cmds, sizeof(cli_cmds) / sizeof(cli_command_t));
#endif  

  /* i2c init */
  hi2cx.i2cx = I2Cx_PORT;
  i2c_config(&hi2cx);

  /* audio init */
  
  codec = &ak4619; //NULL;
  printf("%d\n", audio_init(codec));

  user_button_state = 0;

#ifdef USB_LOW_POWER_WAKUP
  usb_low_power_wakeup_config();
#endif

  /* enable otgfs clock */
  crm_periph_clock_enable(OTG_CLOCK, TRUE);

  /* select usb 48m clcok source */
  //usb_clock48m_select(USB_CLK_HICK);
  crm_usb_clock_source_select(CRM_USB_CLOCK_SOURCE_HICK);

  /* enable otgfs irq */
  nvic_irq_enable(OTG_IRQ, 0, 0);

  /* init usb */
  usbd_init(&otg_core_struct,
            USB_FULL_SPEED_CORE_ID,
            USB_ID,
            &audio_class_handler,
            &audio_desc_handler);

  /* usb gpio config */
  usb_gpio_config();

  while(1)
  {
    audio_loop();

    if(at32_button_press() == USER_BUTTON)
    {
      #ifdef __AUDIO_HID_CLASS_H
      report_buf[0] = HID_REPORT_ID_5;
      report_buf[1] = (~report_buf[1]) & 0x1;
      audio_hid_class_send_report(&otg_core_struct.dev, report_buf, USBD_AUHID_IN_MAXPACKET_SIZE);
      #endif
    }

    #ifndef USB_SOF_OUTPUT_ENABLE
    LED1_TOGGLE;
    #endif
    
    delay_ms(50);

    #if ENABLE_CLI
    if(CLI_ReadLine()){
        CLI_HandleLine();
    }
    #endif
  }
}

/**
  * @brief  usb 48M clock select
  * @param  clk_s:USB_CLK_HICK, USB_CLK_HEXT
  * @retval none
  */
void usb_clock48m_select(usb_clk48_s clk_s)
{
  switch(system_core_clock)
  {
    /* 48MHz */
    case 48000000:
      crm_usb_clock_div_set(CRM_USB_DIV_1);
      break;

    /* 72MHz */
    case 72000000:
      crm_usb_clock_div_set(CRM_USB_DIV_1_5);
      break;

    /* 96MHz */
    case 96000000:
      crm_usb_clock_div_set(CRM_USB_DIV_2);
      break;

    /* 120MHz */
    case 120000000:
      crm_usb_clock_div_set(CRM_USB_DIV_2_5);
      break;

    /* 144MHz */
    case 144000000:
      crm_usb_clock_div_set(CRM_USB_DIV_3);
      break;

    default:
      break;

  }
}

/**
  * @brief  this function config gpio.
  * @param  none
  * @retval none
  */
void usb_gpio_config(void)
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


/**
  * @brief  this function handles otgfs interrupt.
  * @param  none
  * @retval none
  */
void OTG_IRQ_HANDLER(void)
{
  usbd_irq_handler(&otg_core_struct);
}

/**
  * @brief  usb delay millisecond function.
  * @param  ms: number of millisecond delay
  * @retval none
  */
void usb_delay_ms(uint32_t ms)
{
  /* user can define self delay function */
  delay_ms(ms);
}



/**
  * @brief  initializes peripherals used by the i2c.
  * @param  none
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
    i2c_status_type res = i2c_master_transmit(&hi2cx, device, (uint8_t*)data, len, 1000);

    if(res != I2C_OK){
        printf("%s : Error %u\n", __func__, res);
        return 0;
    }

    return len;
}

uint32_t I2C_Master_Read(uint8_t device, uint8_t* data, uint32_t len)
{
    i2c_status_type res = i2c_master_receive(&hi2cx, device, data, len, 1000);

    if(res != I2C_OK){
        printf("%s : Error %u\n", __func__, res);
        return 0;
    }

    return len;
}

/**
  * @}
  */

/**
  * @}
  */
