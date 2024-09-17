#include "tas2563.h"
#include "board.h"

#define ADC1L_EN     (1 << 0)
#define ADC1R_EN     (1 << 1)
#define ADC2L_EN     (1 << 2)
#define ADC2R_EN     (1 << 3)

#define DAC1L_EN     (1 << 4)
#define DAC1R_EN     (1 << 5)
#define DAC2L_EN     (1 << 6)
#define DAC2R_EN     (1 << 7)

static uint8_t tas2563_i2c_addr;
static uint8_t tas2563_WriteReg(uint16_t, uint8_t);
static uint8_t tas2563_ReadReg(uint16_t, uint8_t*);

const audio_codec_t tas2563 = {
    tas2563_Init,
    tas2563_Config,
    tas2563_SampleRate,
    tas2563_Enable,
    tas2563_Disable,
    tas2563_Volume,
    tas2563_Mute,
    tas2563_WriteReg,
    tas2563_ReadReg
};

/**
 * returns 0 on fail, 2 if successful
 */
static uint8_t tas2563_WriteReg (uint16_t Register, uint8_t Value)
{
    uint8_t Data[2];
    Data[0] = Register;
    Data[1] = Value;

    return I2C_Master_Write (tas2563_i2c_addr, (const uint8_t*)&Data, 2);
}

/**
 * return 0 on fail, 1 on success
 */
static uint8_t tas2563_ReadReg (uint16_t Register, uint8_t *Value)
{
   //return I2C_Mem_Read (tas2563_i2c_addr, Register, Val, 1);
   if(I2C_Master_Write(tas2563_i2c_addr, (const uint8_t*)&Register, 1) != 1)
        return 1;

    return I2C_Master_Read(tas2563_i2c_addr, Value, 1);
}

static void tas2563_set_i2c_addr(uint8_t addr)
{
    if(addr < TAS2563_I2C_ADDR0 || addr > TAS2563_I2C_ADDR3){
        if(tas2563_i2c_addr == 0)
            addr = TAS2563_I2C_ADDR0;
        return;
    }
    tas2563_i2c_addr = addr;
}

uint8_t tas2563_Init (uint8_t addr)
{
   uint8_t Retries;
   uint8_t RegValue;


   tas2563_set_i2c_addr(addr);

   // Change to page 0

   // Verify if DAC is present by trying to set page

   if(tas2563_WriteReg (TAS2563_PAGE_REG, 0) == 0)
   {
      return 0;
   }

   // Software reset. Bit is self clearing

   tas2563_WriteReg (TAS2563_SW_RESET_REG, 1);

   Retries = 100;

   while(Retries--)
   {
      if( tas2563_ReadReg (TAS2563_SW_RESET_REG, &RegValue) == 0)
      {
         break;
      }
   }

   if(Retries == 0){
      // Timeout on reset
      return 0;
   }

   // Disable global address

   tas2563_WriteReg (TAS2563_MISC_CFG2_REG, 0x20);

   // Default to 16kHz, frame start at rising edge of WS

   tas2563_WriteReg (TAS2563_TDM_CFG0_REG, 0x02);

   // Boost passthrough

   tas2563_WriteReg (TAS2563_BOOST_CFG1_REG, 0xC4);

   // Enable retry after current event

   tas2563_WriteReg (TAS2563_MISC_CFG1_REG, 0xE6);

   // Disable Brown out prevention

   tas2563_WriteReg (TAS2563_BOP_CFG0_REG, 0x0);

   // Disable PDM

   tas2563_WriteReg (TAS2563_PDM_CFG0_REG, 0x1);


   // Configure slot based on i2c address,
   // slot length to 32bit and word length to 16bit
   tas2563_WriteReg (TAS2563_TDM_CFG2_REG, 0x2);

   tas2563_Enable ();

   return 1;
}

void tas2563_Enable (void)
{
   uint8_t RegVal;

   // Mask active power mode bits

   tas2563_ReadReg(TAS2563_PWR_CTL_REG, &RegVal);

   tas2563_WriteReg(TAS2563_PWR_CTL_REG, RegVal & 0xFC);
}

void tas2563_Disable (void)
{
   uint8_t RegVal;

   tas2563_ReadReg(TAS2563_PWR_CTL_REG, &RegVal);

   // Software shutdown, registers keep their value

   tas2563_WriteReg(TAS2563_PWR_CTL_REG, (RegVal & 0xFC) | 2);
}

uint8_t tas2563_Config (uint8_t DevID, uint8_t Cfg)
{
   switch(DevID)
   {
      case CDC_SET_I2C_ADDR:
        tas2563_set_i2c_addr(Cfg);
        break;

      case CDC_GET_I2C_ADDR:
        return tas2563_i2c_addr;

      case CDC_GET_MCLK:
        return 1;

      case CDC_DEV_DAI:
      case CDC_DEV_DAC1:
      break;
   }

   return 0;
}

void tas2563_Mute (uint8_t DevID, uint8_t Mute)
{
   uint8_t RegVal;

   // Mask MODE bits

   tas2563_ReadReg(TAS2563_PWR_CTL_REG, &RegVal);

   // Set mute mode

   RegVal = (Mute) ? RegVal | 1 : RegVal & 0xFC;

   tas2563_WriteReg(TAS2563_PWR_CTL_REG, RegVal);
}


void tas2563_Volume (uint8_t DevID, uint8_t Volume)
{
   uint8_t RegVal;

   if (Volume > 100)
   {
      return;
   }

   if (Volume == 0)
   {
      tas2563_Mute(0, 1);
      return;
   }

   tas2563_Mute(0, 0);

   // Mask AMP_LEVEL bits

   tas2563_ReadReg(TAS2563_PB_CFG1_REG, &RegVal);

   // Map Volume to AMP_LEVEL [1:0x1C]

   Volume = (Volume * 0x1C ) / 100;

   RegVal = (RegVal & 0xC1) | (Volume << 1);

   tas2563_WriteReg(TAS2563_PB_CFG1_REG, RegVal);
}

void tas2563_SampleRate (uint32_t Rate)
{
    uint8_t RegVal;

    switch(Rate)
    {
        case CDC_SR_8K:
            Rate = 0;
            break;

        case CDC_SR_16K:
            Rate = 1;
            break;

        case CDC_SR_22K:
        case CDC_SR_24K:
            Rate = 2;
            break;

        case CDC_SR_32K:
            Rate = 3;
            break;

        case CDC_SR_44K:
        case CDC_SR_48K:
            Rate = 4;
            break;

        case CDC_SR_88K:
        case CDC_SR_96K:
            Rate = 5;
            break;

        /*
        case CDC_SR_176K:
        case CDC_SR_192K:
            Rate = 6;
            break;
        */

        default:
            return;
    }

    // Read register and mask SAMP_RATE bits

    tas2563_ReadReg(TAS2563_TDM_CFG0_REG, &RegVal);

    RegVal = (RegVal & 0x0E) | (Rate << 1);

    tas2563_WriteReg(TAS2563_TDM_CFG0_REG, RegVal);

    tas2563_Enable();
}
