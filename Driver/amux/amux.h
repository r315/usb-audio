#ifndef _AMUX_H_
#define _AMUX_H_

#include <stdint.h>

#define AMUX_I2C_ADDR           (0x14 << 1)

#define AMUX_MUTE_REG           0
#define AMUX_CFG_REG            4
#define AMUX_SLOT_SUM_REG       16


#define AMUX_CFG_MUTE_ALL       1
#define AMUX_CFG_RESET          2

#define AMUX_MAX_SLOTS          32

#define AMUX_GET_SLOT(cx,sy)    ((((cx) & 7) << 2) + ((sy) & 3))

uint8_t amux_Init(void);
uint8_t amux_Route(uint8_t src_ch, uint8_t src_sl, uint8_t dst_ch, uint8_t dst_sl, uint8_t en);
uint8_t amux_MuteAll(void);
uint8_t amux_Mute(uint8_t ch, uint8_t sl, uint8_t mute);
#endif