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
#include "debug.h"


#ifndef ENABLE_DBG_APP
#define DBG_TAG             "APP : "
#define DBG_APP_PRINT(...)  DBG_PRINT
#define DBG_APP_INF(...)    DBG_INF(DBG_TAG __VA_ARGS__)
#define DBG_APP_ERR(...)    DBG_ERR(DBG_TAG __VA_ARGS__)
#else
#define DBG_APP_PRINT(...)
#define DBG_APP_ERR(...)
#define DBG_APP_INF(...)
#endif


typedef struct {
    const char *name;
    const audio_codec_t *cdc;
    uint8_t i2c_addr;
}cdc_list_t;

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

static uint8_t dummy_Init (uint8_t addr){ return 0; }// fail init to allow scan for other
static uint8_t dummy_Config (uint8_t DevID, uint8_t Mode) { return (DevID == CDC_GET_MCLK) ? CDC_MCLK_256FS : 0;}
static void    dummy_SampleRate (uint32_t Rate){}
static void    dummy_Enable (void) {}
static void    dummy_Disable (void) {}
static void    dummy_Volume (uint8_t DevID, uint8_t Volume){}
static void    dummy_Mute (uint8_t DevID, uint8_t Mode) {}
static uint8_t dummy_writeReg (uint16_t reg, uint8_t val) {return 1;}
static uint8_t dummy_readReg (uint16_t reg, uint8_t *val) {return 1;}

static const audio_codec_t dummy_codec = {
    dummy_Init,
    dummy_Config,
    dummy_SampleRate,
    dummy_Enable,
    dummy_Disable,
    dummy_Volume,
    dummy_Mute,
    dummy_writeReg,
    dummy_readReg
};

static const cdc_list_t codecs [] = {
    {"none", &dummy_codec, 0},
    {"ak4619", &ak4619, AK4619_I2C_ADDR},
    {"tas2563", &tas2563, TAS2563_I2C_ADDR0},
    {"tas2563", &tas2563, TAS2563_I2C_ADDR1},
    {"tas2563", &tas2563, TAS2563_I2C_ADDR2},
    {"tas2563", &tas2563, TAS2563_I2C_ADDR3}
};

#if ENABLE_CLI

static const stdinout_t uart_ops = {
    .available = serial_available,
    .read = serial_read,
    .write = serial_write
};

static void dump_buf(uint8_t *buf, uint32_t count);

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
            uint8_t addr = codec->Config(CDC_GET_I2C_ADDR, 0);
            for(uint8_t i = 0; i < sizeof(codecs)/sizeof(cdc_list_t); i++){
                printf("\t%d [%c] %s (%X)\n", i, (codecs[i].i2c_addr == addr) ? '*' : ' ', codecs[i].name, codecs[i].i2c_addr);
            }
        }else{
            int32_t val;
            if(CLI_Ia2i(argv[2], &val)){
                codec = codecs[val].cdc;
                codec->Config(CDC_SET_I2C_ADDR, codecs[val].i2c_addr);
                audio_set_codec(codec);
            }
        }
        return CLI_OK;
    }

    if( !strcmp("scan", argv[1])){
        printf("\n   ");

        for(int i = 0; i < 16; i++){
            printf("%02X ", i);
        }

        for(int i = 0; i < 128; i++){
            if( (i & 15) == 0)
                printf("\n%02X ", i & 0xF0);

            if(i >= 3 && i <= 0x77){ // default range from i2cdetect
                if(i2c_master_scan_addr(&hi2cx, (i << 1), 1000) == I2C_OK){
                    printf("%02X ", i);
                }else{
                    printf("-- ");
                }
                delay_ms(1);
            }else{
               printf("   ");
            }
        }
        putchar('\n');

        i2c_config(&hi2cx);

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
        }else{
            printf("%d\n", audio_get_spk_volume());
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

    if( !strcmp("regs", argv[1])){
        uint8_t buf[32];
        buf [0] = 0;
        if(i2c_master_transmit(&hi2cx, 16 << 1, buf, 1, 1000) == I2C_OK){
            if(i2c_master_receive(&hi2cx, 16 << 1, buf, 32, 1000) != I2C_OK){
                printf("Failed to read");
            }else{
                return I2C_OK;
            }
        }
        dump_buf(buf, 32);
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

static int trimCmd(int argc, char **argv)
{
    int32_t val;
    crm_clocks_freq_type clocks;

    if(argc < 2){
      printf("Usage: trim [trim] [cal]\n");
      printf("\t cal, 0-255\n");
      printf("\t trim, 0-63\n\n");

      crm_clocks_freq_get(&clocks);
      printf("SCLK: %luHz\n", clocks.sclk_freq);
      printf("HICKCAL: %d\n", CRM->ctrl_bit.hickcal);
      printf("HICKTRIM: %d\n", CRM->ctrl_bit.hicktrim);
      return CLI_OK;
    }

    if(CLI_Ia2i(argv[1], &val)){
        val &= 0x3F;
        CRM->ctrl_bit.hicktrim = val;
    }

    if(CLI_Ia2i(argv[2], &val)){
        val &= 0xFF;
        CRM->misc1_bit.hickcal_key = 0x5a;
        CRM->ctrl_bit.hickcal = val;
    }

    return CLI_OK;
}

static void dump_buf(uint8_t *buf, uint32_t count)
{
    for(int i = 0; i < count; i ++){
        if( (i & 15) == 0)
            printf("\n%02X: ", i & 0xF0);
        printf("%02X ", buf[i]);
    }
    putchar('\n');
}
#if 0
/**
 *
*/
static i2c_status_type i2c_imm_read(i2c_handle_type *handle, uint8_t slv_addr, uint8_t *buf, uint32_t count, uint32_t timeout)
{
    /* enable ack */
    i2c_ack_enable(handle->i2cx, TRUE);

    /* generate start condtion */
    i2c_start_generate(handle->i2cx);

    /* wait for the start flag to be set */
    if(i2c_wait_flag(handle, I2C_STARTF_FLAG, I2C_EVENT_CHECK_NONE, timeout) != I2C_OK)
    {
        handle->error_code = I2C_ERR_START;

        goto i2c_err;
    }

    /* send slave address */
    i2c_7bit_address_send(handle->i2cx, slv_addr, I2C_DIRECTION_RECEIVE);

    /* wait for the addr7 flag to be set */
    if(i2c_wait_flag(handle, I2C_ADDR7F_FLAG, I2C_EVENT_CHECK_ACKFAIL, timeout) != I2C_OK)
    {
        handle->error_code = I2C_ERR_ADDR;

        goto i2c_err;
    }

i2c_stop_generate(handle->i2cx);

    return I2C_OK;

i2c_err:
    i2c_stop_generate(handle->i2cx);

    return handle->error_code;
}
#endif

#ifdef ENABLE_AMUX
static i2c_status_type readMux(i2c_handle_type *handle, uint8_t reg, uint8_t *buf, uint32_t count)
{
    if(i2c_master_transmit(handle, AMUX_I2C_ADDR << 1, &reg, 1, 1000) == I2C_OK){
        if(i2c_master_receive(handle, AMUX_I2C_ADDR << 1, buf, count, 1000) != I2C_OK){
            printf("Failed to read");
        }else{
            return I2C_OK;
        }
    }

    printf("Failed to write reg address\n");

    return I2C_ERR_ACKFAIL;
}

const uint16_t tone500[] = {
0x0000, 0x18f8, 0x30fb, 0x471c, 0x5a81, 0x6a6c, 0x7640, 0x7d89,
0x7fff, 0x7d89, 0x7640, 0x6a6c, 0x5a81, 0x471c, 0x30fb, 0x18f8,
0xffff, 0xe707, 0xcf04, 0xb8e3, 0xa57e, 0x9593, 0x89bf, 0x8276,
0x8001, 0x8276, 0x89bf, 0x9593, 0xa57e, 0xb8e3, 0xcf04, 0xe707,
};

const uint16_t tone400[] = {
0x0001, 0x075C, 0x0E7A, 0x1518, 0x1AFF, 0x1FFB, 0x23E0, 0x268A,
0x27E6, 0x27E7, 0x268B, 0x23DF, 0x1FFC, 0x1AFD, 0x151B, 0x0E77,
0x0760, 0xFFFD, 0xF8A5, 0xF187, 0xEAE5, 0xE504, 0xE003, 0xDC21,
0xD977, 0xD817, 0xD81A, 0xD977, 0xDC1D, 0xE00A, 0xE4FC, 0xEAEB,
0xF185, 0xF8A3
};

static int muxCmd(int argc, char **argv)
{
    uint32_t count = 256; //16 + 128 + 64;
    uint32_t value;
    uint8_t regs_buf[count] __attribute__ ((aligned (4)));

    if( !strcmp("help", argv[1]) || argc < 2){
        printf("\tinit\n");
        printf("\tregs\n");
        printf("\treset\n");
        printf("\tver\n");
        printf("\trr <reg>\n");
        printf("\twr <reg> <val>\n");
        printf("\tpage <nr>\n");
        printf("\troute <scr> <dst> [func]\n"
               "\t  src/dst, 0-31\n"
               "\t  func, 0-3,\n"
               "\t    0: disabled\n"
               "\t    1: enable\n"
               "\t    2: apply pre-gain\n"
               "\t    3: feedback\n"
               "\t    4: feedback + pre-gain\n");
        printf("\tatt <value>\n");
        printf("\tgain <dst> <value>\n");
        printf("\tagc <en>\n");
        printf("\ttone_load\n");
        printf("\ttone_pattern <idx> <duration> <pause> <reps> <interval>\n");
        return CLI_OK;
    }

    if( !strcmp("regs", argv[1])){
        if(readMux(&hi2cx, 0, regs_buf, count) == I2C_OK){
            dump_buf(regs_buf, count);
            return CLI_OK;
        }
    }

    if( !strcmp("ver", argv[1])){
        amux_GetVer(regs_buf);
        printf("v%d.%d.%d", regs_buf[0], regs_buf[1], regs_buf[2]);
        return CLI_OK_LF;
    }

    if( !strcmp("wr", argv[1])){
        if(CLI_Ha2i(argv[2], &value)){
            regs_buf[0] = value;
            if(CLI_Ha2i(argv[3], &value)){
                regs_buf[1] = value;
                if(i2c_master_transmit(&hi2cx, AMUX_I2C_ADDR << 1, regs_buf, 2, 1000) == I2C_OK){
                    return CLI_OK;
                }
            }
        }
    }

    if( !strcmp("rr", argv[1])){
        if(CLI_Ha2i(argv[2], (uint32_t*)&regs_buf)){
            count = 1;
            CLI_Ha2i(argv[3], &count);
            if(readMux(&hi2cx, regs_buf[0], regs_buf, count) == I2C_OK){
                dump_buf(regs_buf, count);
                return CLI_OK;
            }
        }
    }


    if( !strcmp("init", argv[1])){
        printf("%s", amux_Init() ? "ok" : "fail");
        return CLI_OK_LF;
    }

    if(!strcmp("reset", argv[1])){
        amux_Reset();
        return CLI_OK;
    }

    /**
     * route <cxsy> <cxsy> 1|0
    */
    if( !strcmp("route", argv[1])){

        if(argc < 4){
            return CLI_MISSING_ARGS;
        }

        int32_t src, dst;

        if(CLI_Ia2i(argv[2], &src)){
            if(CLI_Ia2i(argv[3], &dst)){
                if(CLI_Ha2i(argv[4], &value)){
                    amux_Route(src, dst, value);
                }else{
                    printf("Route state: %d\n", amux_RouteState(src, dst));
                }
                return CLI_OK;
            }
        }
    }

    if( !strcmp("mclk_pha", argv[1])){
        if(CLI_Ia2i(argv[2], (int32_t*)&value)){
            amux_MclkPha(value);
            return CLI_OK;
        }
    }

    if( !strcmp("gain", argv[1])){
        int32_t slot;
        if(CLI_Ia2i(argv[2], &slot)){
            if(CLI_Ha2i(argv[3], (uint32_t*)&value)){
                amux_gain(slot, value);
                return CLI_OK;
            }
        }
    }

    if( !strcmp("agc", argv[1])){
        if(CLI_Ia2i(argv[2], (int32_t*)&value)){
            amux_agc(value);
            return CLI_OK;
        }
    }

    if( !strcmp("tone_load", argv[1])){
        printf("Loading tone..\n");
        amux_tone_load(tone400, sizeof(tone400) / 2);
        printf("done\n");
        return CLI_OK;
    }

    if( !strcmp("tone_pattern", argv[1])){
        CLI_Ia2i(argv[2], (int32_t*)&value);
        regs_buf[0] = AMUX_TONE_P0_CTL0 + (value << 1);
        CLI_Ia2i(argv[3], (int32_t*)&value);
        regs_buf[1] = (value & 7) << 5; // duration
        CLI_Ia2i(argv[4], (int32_t*)&value);
        regs_buf[1] = regs_buf[1] | (value & 0x1F); // pause
        CLI_Ia2i(argv[5], (int32_t*)&value);
        regs_buf[2] = (value & 3) << 6; // repetition
        CLI_Ia2i(argv[6], (int32_t*)&value);
        regs_buf[2] = regs_buf[2] | (value & 0x3F); // interval

        amux_page(3);

        printf("ctl0: %x, ctl1: %x\n", regs_buf[1], regs_buf[2]);

        if(i2c_master_transmit(&hi2cx, AMUX_I2C_ADDR << 1, regs_buf, 3, 1000) == I2C_OK){
            return CLI_OK;
        }
    }

    if( !strcmp("page", argv[1])){
        if(CLI_Ia2i(argv[2], (int32_t*)&value)){
            regs_buf[0] = AMUX_PAGE_REG;
            regs_buf[1] = value;
            if(i2c_master_transmit(&hi2cx, AMUX_I2C_ADDR << 1, regs_buf, 2, 1000) == I2C_OK){
                return CLI_OK;
            }
        }
    }

    return CLI_BAD_PARAM;
}
#endif
cli_command_t cli_cmds [] = {
    {"help", ((int (*)(int, char**))CLI_Commands)},
    {"cdc", codecCmd},
    {"reset", resetCmd},
    {"clear", clearCmd},
    {"mclk", audioCmd},
    {"freq", audioCmd},
    {"disable", audioCmd},
#ifdef ENABLE_AMUX
    {"mux", muxCmd},
#endif
    {"trim", trimCmd},
};
#endif

/**
  * @brief  main function.
  * @param  none
  * @retval none
  */
int main(void)
{
  board_init();

#if ENABLE_CLI
  CLI_Init("Audio >", &uart_ops);
  CLI_RegisterCommand(cli_cmds, sizeof(cli_cmds) / sizeof(cli_command_t));
#endif

#ifdef ENABLE_DEBUG
    dbg_init(&uart_ops);
#endif

  /* i2c init */
  hi2cx.i2cx = I2Cx_PORT;
  i2c_config(&hi2cx);

  codec = NULL;

  /* audio init */
  DBG_APP_PRINT("\n\n");
  uint8_t found_idx = 0;
  for(uint8_t idx = 0; idx < sizeof(codecs)/sizeof(cdc_list_t); idx++){
     const cdc_list_t *pcdc = &codecs[idx];
     if(pcdc->cdc){
        uint8_t res = pcdc->cdc->Init(pcdc->i2c_addr);
        if(res){
            DBG_APP_INF("Found codec: %s", pcdc->name);
            if(found_idx == 0){
                found_idx = idx; // here idx should never be zero
            }
        }
     }
  }

  codec = codecs[found_idx].cdc;
  DBG_APP_INF("Using codec: %s :%d\n", codecs[found_idx].name, audio_init(codec));

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
    i2c_status_type res = i2c_master_transmit(&hi2cx, device << 1, (uint8_t*)data, len, 1000);

    if(res != I2C_OK){
        DBG_APP_ERR("%s : %u", __func__, res);
        return 0;
    }

    return len;
}

uint32_t I2C_Master_Read(uint8_t device, uint8_t* data, uint32_t len)
{
    i2c_status_type res = i2c_master_receive(&hi2cx, device << 1, data, len, 1000);

    if(res != I2C_OK){
        DBG_APP_ERR("%s : %u", __func__, res);
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
