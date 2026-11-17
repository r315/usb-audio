#include "codec_null.h"


uint8_t dummy_Init (uint8_t addr){ (void)addr; return 0; }// fail init to allow scan for other
uint32_t dummy_Config (uint8_t DevID, uint8_t Mode) {(void)Mode; return (DevID == CDC_CTRL_MCLK_GET) ? CDC_MCLK_256FS : 0;}
void    dummy_SampleRate (uint32_t Rate){(void)Rate;}
void    dummy_Enable (void) {}
void    dummy_Disable (void) {}
void    dummy_Volume (uint8_t DevID, uint8_t Volume){(void)DevID;(void)Volume;}
void    dummy_Mute (uint8_t DevID, uint8_t Mode) {(void)DevID;(void)Mode;}
uint8_t dummy_writeReg (uint16_t reg, uint8_t val) {(void)reg;(void)val;return 1;}
uint8_t dummy_readReg (uint16_t reg, uint8_t *val) {(void)reg;(void)val;return 1;}

const codec_drv_t dummy_codec = {
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
