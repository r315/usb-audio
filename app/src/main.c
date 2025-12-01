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
#include "audio.h"
#include "cli_simple.h"
#include "i2c_application.h"
#include "tas2563.h"
//#include "max98374.h"
#include "ak4619.h"
#include "debug.h"

#ifdef ENABLE_AMUX
#include "amux.h"
#endif

#ifndef ENABLE_DBG_APP
#define DBG_TAG             "APP : "
#define DBG_APP_PRINT       DBG_PRINT
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

#ifdef __AUDIO_HID_CLASS_H
ALIGNED_HEAD  uint8_t report_buf[USBD_AUHID_IN_MAXPACKET_SIZE] ALIGNED_TAIL;
#endif

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
                if(I2C_Master_Scan(i) == I2C_OK){
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

        //i2c_config(&hi2cx);

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

        if(I2C_Master_Write(16, buf, 1)){
            if(!I2C_Master_Read(16, buf, 32)){
                printf("Failed to read");
            }else{
                dump_buf(buf, 32);
                return I2C_OK;
            }
        }
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
            bus_i2s_mclk(AUDIO_DEFAULT_MCLK_FREQ, AUDIO_DEFAULT_MCLK_SRC, val & 1);
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
        disconnect_usb();
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

#ifdef ENABLE_AMUX
static i2c_status_type readMux(uint8_t reg, uint8_t *buf, uint32_t count)
{
    if(!I2C_Master_Write(AMUX_I2C_ADDR, &reg, 1)){
        printf("Failed to write reg address\n");
        return I2C_ERR_ACKFAIL;
    }

    if(!I2C_Master_Read(AMUX_I2C_ADDR, buf, count)){
        printf("Failed to read");
        return I2C_ERR_ACKFAIL;
    }

    return I2C_OK;
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
        printf("\troute <scr> <dst> [mix]\n"
               "\t  src/dst, 0-31\n"
               "\t  mix,     0-3,\n"
               "\t        0: disabled\n"
               "\t        1: in[src] -> out[dst]\n"
               "\t        2: out[src] -> out[dst]\n"
               "\t        3: in[src] + out[src] -> out[dst]\n");
        printf("\tatt <value>\n");
        printf("\tgain <dst> <value>\n");
        printf("\tagc <en>\n");
        printf("\ttone_load\n");
        printf("\ttone_pattern <idx> <duration> <pause> <reps> <interval>\n");
        return CLI_OK;
    }

    if( !strcmp("regs", argv[1])){
        if(readMux(0, regs_buf, count) == I2C_OK){
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
                if(I2C_Master_Write(AMUX_I2C_ADDR, regs_buf, 2) == 2){
                    return CLI_OK;
                }
            }
        }
    }

    if( !strcmp("rr", argv[1])){
        if(CLI_Ha2i(argv[2], (uint32_t*)&regs_buf)){
            count = 1;
            CLI_Ha2i(argv[3], &count);
            if(readMux(regs_buf[0], regs_buf, count) == I2C_OK){
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
            if(CLI_Ia2i(argv[3], (int32_t*)&value)){
                amux_gain(slot, value);
            }else{
                printf("gain: %u\n", amux_gain_get(slot));
            }
            return CLI_OK;
        }
    }

    if( !strcmp("agc", argv[1])){
        if(CLI_Ia2i(argv[2], (int32_t*)&value)){
            amux_agc(value);
        }else{
            printf("%s\n", amux_agc_get() ? "Enabled" : "Disabled");
        }
        return CLI_OK;
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

        if(I2C_Master_Write(AMUX_I2C_ADDR, regs_buf, 3) == 3){
            return CLI_OK;
        }
    }

    if( !strcmp("page", argv[1])){
        if(CLI_Ia2i(argv[2], (int32_t*)&value)){
            regs_buf[0] = AMUX_PAGE_REG;
            regs_buf[1] = value;
            if(I2C_Master_Write(AMUX_I2C_ADDR, regs_buf, 2)){
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
  DBG_APP_INF("Using codec: %s :%d", codecs[found_idx].name, audio_init(codec));

  user_button_state = 0;

  connect_usb();

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
  * @}
  */

/**
  * @}
  */
