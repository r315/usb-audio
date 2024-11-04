#include "amux.h"
#include "board.h" // Includes I2C function prototypes

/**
 * Internal functions, not needed to be exposed
*/
static uint8_t amux_ReadReg(uint16_t Reg, uint8_t *Val)
{
    if(!I2C_Master_Write(AMUX_I2C_ADDR, (const uint8_t*)&Reg, 1))
        return 0;

    return I2C_Master_Read(AMUX_I2C_ADDR, Val, 1) > 0;
}

static uint8_t amux_WriteReg(uint16_t Reg, uint8_t Val)
{
    uint8_t data[2];

    data[0] = Reg;
    data[1] = Val;

    return I2C_Master_Write(AMUX_I2C_ADDR, (const uint8_t*)data, 2) > 0;
}

static uint8_t amux_ReadRegN(uint16_t Reg, uint8_t *Buffer, uint8_t len)
{
    if(!I2C_Master_Write(AMUX_I2C_ADDR, (const uint8_t*)&Reg, 1))
        return 0;

    return I2C_Master_Read(AMUX_I2C_ADDR, Buffer, len) > 0;
}

static uint8_t amux_WriteRegN(uint16_t Reg, uint8_t *Buffer, uint8_t Len)
{
    uint8_t data[Len + 1];

    data[0] = Reg;

    for(uint8_t i = 1; i <= Len; i++){
        data[i] = *Buffer++;
    }

    return I2C_Master_Write(AMUX_I2C_ADDR, (const uint8_t*)data, Len + 1) > 0;
}


/**
 * Init is a simple reset and check
 * for 'AM' identifier
 */
uint8_t amux_Init(void)
{
    uint8_t Buffer[2];

    if(!amux_Reset()){
        return 0;
    }

    if(!amux_ReadRegN(AMUX_ID_REG, Buffer, 2)){
        return 0;
    }

    if(Buffer[0] == 'A' && Buffer[1] == 'M'){
        return 1;
    }

    return 0;
}

/**
 * Buffer must have at size of least 3 bytes
 *
 * \returns Buffer[0]  Major
 *          Buffer[1]  Minor
 *          Buffer[2]  Patch
 */
uint8_t amux_GetVer(uint8_t *Buffer)
{
    return amux_ReadRegN(AMUX_VER_REG, Buffer, 3);
}


/**
 * If enabled on FPGA, returns if fs and blck
 * signals are present
 *
 */
uint8_t amux_GetStatus(void)
{
    uint8_t Status;

    if(amux_ReadReg(AMUX_CTRL_REG, &Status)){
        return 0;
    }

    return Status & (AMUX_CTRL_FS_ST | AMUX_CTRL_BCLK_ST);
}

/**
 * \param src source slot 0-31
 * \param dst destination slot 0-31
 * \param en  1: enable 0: disabled
 * \return 1 on success, 0 otherwise
*/
uint8_t amux_Route(uint8_t src, uint8_t dst, uint8_t en)
{
    uint8_t reg_data, reg_offset;

    if(src > 31 || dst > 31){
        return 0;
    }

#if VER1
    reg_offset = (src << 2) + (dst >> 3);
    reg_offset += 0x10;

    if(!amux_ReadReg(reg_offset, &reg_data)){
        return 0;
    }

    if(en){
        reg_data |= (1 << (dst & 7));
    }else{
        reg_data &= ~(1 << (dst & 7));
    }
#else
    reg_offset = (dst << 2) + (src >> 3);
    reg_offset += 0x10;

    if(!amux_ReadReg(reg_offset, &reg_data)){
        return 0;
    }

    if(en){
        reg_data |= (1 << (src & 7));
    }else{
        reg_data &= ~(1 << (src & 7));
    }
#endif
    return amux_WriteReg(reg_offset, reg_data);
}

/**
 * \param src source slot 0-31
 * \param dst destination slot 0-31
 * \return 1 if route is enable, 0 otherwise
*/
uint8_t amux_RouteState(uint8_t src, uint8_t dst)
{
    uint8_t reg_data, reg_offset;

    if(src > 31 || dst > 31){
        return 0;
    }
#if VER1
    reg_offset = (src << 2) + (dst >> 3);
    reg_offset += 0x10;

    if(!amux_ReadReg(reg_offset, &reg_data)){
       return 0;
    }
    return !!(reg_data & (1 << (dst & 7)));
#else
    reg_offset = (dst << 2) + (src >> 3);
    reg_offset += 0x10;

    if(!amux_ReadReg(reg_offset, &reg_data)){
       return 0;
    }

    return !!(reg_data & (1 << (src & 7)));
#endif
}

/**
 * Mute all channels, routes are not affected
 */
uint8_t amux_MuteAll(void)
{
    return amux_WriteReg(AMUX_CTRL_REG, AMUX_CTRL_MUTE_ALL);
}

/**
 * Reset all internal register to default value.
 * All routes are lost and all channels are muted
 */
uint8_t amux_Reset(void)
{
    return amux_WriteReg(AMUX_CTRL_REG, AMUX_CTRL_RESET);
}

/**
 * Mutes a single slot
 * \param slot  slot to be muted 0-31
 * \param mute  1: Muted, 0: normal
 */
uint8_t amux_Mute(uint8_t slot, uint8_t mute)
{
    if(slot > 31){
        return 0;
    }

    uint8_t slot_reg = slot / 8;
    uint8_t slot_mask = (1 << (slot & 7));
    uint8_t mute_data;

    if(!amux_ReadReg(slot_reg, &mute_data)){
        return 0;
    }

    mute_data = (mute == 0) ? mute_data & ~slot_mask : mute_data | slot_mask;

    return amux_WriteReg(slot_reg, mute_data);
}

/**
 * Controls PLL generated MCLK output phase.
 * \param Pha   0-15
 */
uint8_t amux_MclkPha(uint8_t Pha)
{
    uint8_t Dut;
    Pha &= 15;
    Dut = (Pha + 8) & 15;
    Pha = (Pha << 4) | Dut;
    return amux_WriteReg(AMUX_MCLK_PHA_REG, Pha);
}

/**
 * Enable Limiter/agc on a given slot
 *
 * \param slot
 * \param en     1: Active, 0:passthrough
 */
uint8_t amux_agc(uint8_t slot, uint8_t en)
{
    return amux_WriteReg(AMUX_CTRL_REG, en & 1);
}

uint8_t amux_att(uint8_t slot, uint16_t att)
{
    uint8_t RegVal;

    uint8_t offset = slot >> 3;

    if(!amux_ReadReg(AMUX_ATT_MASK0 + offset, &RegVal)){
        return 0;
    }

    RegVal |= (1 << (slot & 7));

    if(!amux_WriteReg(AMUX_ATT_MASK0 + offset, RegVal)){
        return 0;
    }

    return amux_WriteRegN(AMUX_ATT_REG, (uint8_t*)&att, 2);
}

uint8_t amux_gain(uint8_t slot, uint16_t gain)
{
    uint8_t offset = slot << 1;
    return amux_WriteRegN(AMUX_GAIN_REG + offset, (uint8_t*)&gain, 2);
}