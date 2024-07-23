#include "amux.h"
#include "board.h"

/**
 * \returns 0 on success
*/
static uint8_t amux_ReadReg(uint16_t Reg, uint8_t *Val)
{
    if(I2C_Master_Write(AMUX_I2C_ADDR, (const uint8_t*)&Reg, 1) != 1)
        return 1;

    return I2C_Master_Read(AMUX_I2C_ADDR, Val, 1) == 0;
}

static uint8_t amux_WriteReg(uint16_t Reg, uint8_t Val)
{
    uint8_t data[2];

    data[0] = Reg;
    data[1] = Val;
    
    return I2C_Master_Write(AMUX_I2C_ADDR, (const uint8_t*)data, 2) == 0;
}

uint8_t amux_Init(void)
{
    return amux_WriteReg(AMUX_CFG_REG, AMUX_CFG_RESET);
}

/**
 * \param src_ch source channel 1-8
 * \param src_sl source slot 1-4
 * 
 * \returns 0 on success
*/
uint8_t amux_Route(uint8_t src_ch, uint8_t src_sl, uint8_t dst_ch, uint8_t dst_sl, uint8_t en)
{
    uint8_t reg_data;
    uint8_t in_slot = AMUX_GET_SLOT(src_ch - 1, src_sl - 1);
    uint8_t out_slot = AMUX_GET_SLOT(dst_ch - 1, dst_sl - 1);

    uint8_t reg_offset = (in_slot << 2) + (out_slot >> 3);
    reg_offset += 0x10;
    
    amux_ReadReg(reg_offset, &reg_data);

    if(en){
        reg_data |= (1 << (out_slot & 7));
    }else{
        reg_data &= ~(1 << (out_slot & 7));
    }

    return amux_WriteReg(reg_offset, reg_data);
}

uint8_t amux_MuteAll(void)
{
    return amux_WriteReg(AMUX_CFG_REG, AMUX_CFG_MUTE_ALL);
}

uint8_t amux_Mute(uint8_t ch, uint8_t sl, uint8_t mute)
{
    uint8_t dsp_slot = AMUX_GET_SLOT(ch - 1, sl - 1);
    uint8_t slot_reg = dsp_slot / 8;
    uint8_t slot_mask = (1 << (dsp_slot & 7));
    uint8_t mute_data;

    amux_ReadReg(slot_reg, &mute_data);

    mute_data = (mute == 0) ? mute_data & ~slot_mask : mute_data | slot_mask;

    return amux_WriteReg(slot_reg, mute_data);
}
