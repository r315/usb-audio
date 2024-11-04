#ifndef _AMUX_H_
#define _AMUX_H_

#include <stdint.h>

#define AMUX_I2C_ADDR           0x14

#define AMUX_CTRL_REG           0
#define AMUX_MCLK_PHA_REG       1
#define AMUX_ATT_MASK0          4
#define AMUX_ATT_MASK1          5
#define AMUX_ATT_MASK2          6
#define AMUX_ATT_MASK3          7
#define AMUX_ATT_REG            8
#define AMUX_ID_REG             11
#define AMUX_VER_REG            13
#define AMUX_ROUTE_REG          16
#define AMUX_GAIN_REG           0x90

// Control register bits
#define AMUX_CTRL_RESET         (1 << 0)
#define AMUX_CTRL_MUTE_ALL      (1 << 1)
#define AMUX_CTRL_AGC           (1 << 2)
#define AMUX_CTRL_BCLK_ST       (1 << 4)
#define AMUX_CTRL_FS_ST         (1 << 5)

uint8_t amux_Init(void);
uint8_t amux_Route(uint8_t src, uint8_t dst, uint8_t en);
uint8_t amux_RouteState(uint8_t src, uint8_t dst);
uint8_t amux_MuteAll(void);
uint8_t amux_Mute(uint8_t slot, uint8_t mute);
uint8_t amux_Reset(void);
uint8_t amux_GetVer(uint8_t *buf);
uint8_t amux_GetStatus(void);
uint8_t amux_MclkPha(uint8_t pha);
uint8_t amux_att(uint8_t slot, uint16_t att);
uint8_t amux_gain(uint8_t slot, uint16_t gain);
uint8_t amux_agc(uint8_t slot, uint8_t en);
#endif
