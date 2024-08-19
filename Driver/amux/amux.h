#ifndef _AMUX_H_
#define _AMUX_H_

#include <stdint.h>

#define AMUX_I2C_ADDR           0x14

#define AMUX_MUTE_REG           0
#define AMUX_CTRL_REG           4
#define AMUX_MCLK_PHA_REG       5
#define AMUX_ID_REG             10
#define AMUX_ROUTE_REG          16
#define AMUX_VER_REG            13

#define AMUX_CTRL_FS_ST         (1 << 5)
#define AMUX_CTRL_BCLK_ST       (1 << 4)

#define AMUX_CFG_MUTE_ALL       1
#define AMUX_CFG_RESET          2

#define AMUX_MAX_SLOTS          32

#define AMUX_GET_SLOT(cx,sy)    ((((cx) & 7) << 2) + ((sy) & 3)) // This assumes that each channel has only 4 slots

uint8_t amux_Init(void);
uint8_t amux_Route(uint8_t src, uint8_t dst, uint8_t en);
uint8_t amux_MuteAll(void);
uint8_t amux_Mute(uint8_t slot, uint8_t mute);
uint8_t amux_Reset(void);
uint8_t amux_GetVer(uint8_t *buf);
uint8_t amux_MclkPha(uint8_t pha);
uint8_t amux_GetStatus(void);
#endif
