// =============================================================================
/*!
 * \file       tas2563.c
 *
 * This file contains the implementation of the DAC driver for the TAS2563
 *
 * \author     Hugo Reis <hugo.reis@bithium.com>
 *
 * \version    x.x.x
 *
 * \copyright  Copyright &copy; &nbsp; 2023 Bithium S.A.
 */
// =============================================================================

#include <stdio.h>
#include <string.h>
#include "codec.h"
#include "tas2563.h"
#include "board.h"


const CDC_Type tas2563 = 
{
   TAS2563_Init,
   TAS2563_Config,
   TAS2563_SampleRate,
   TAS2563_Enable,
   TAS2563_Disable,
   TAS2563_Volume,
   TAS2563_Mute,
   TAS2563_WriteReg,
   TAS2563_ReadReg
};

// =============================================================================
// TAS2563_WriteReg
// =============================================================================
/*!
 *
 * Write a DAC Register.
 *
 * \param Addr - Address to write to
 * \param Data - Data to write (8-bit only)
 */
// =============================================================================

uint8_t TAS2563_WriteReg (uint8_t Register, uint8_t Val)
{
   uint8_t Data[2];

   Data[0] = Register;
   Data[1] = Val;

   return I2C_Master_Write (TAS2563_I2C_ADDR, Data, 2);
}

// =============================================================================
// TAS2563_ReadReg
// =============================================================================
/*!
 *
 * Read a DAC Register.
 *
 * \param Addr - Address to read from
 *
 * \return register value (8-bit only)
 */
// =============================================================================

uint8_t TAS2563_ReadReg (uint8_t Register)
{
   uint8_t Data;

   I2C_Master_Write (TAS2563_I2C_ADDR, &Register, 1);
   I2C_Master_Read(TAS2563_I2C_ADDR, &Data, 1);

   return Data;
}

// =============================================================================
// TAS2563_Enable
// =============================================================================
/*!
 *
 * Enable DAC (power up).
 *
 */
// =============================================================================

void TAS2563_Enable (void)
{
   uint8_t RegVal;

   // Mask active power mode

   RegVal = TAS2563_ReadReg(TAS2563_PWR_CTL_REG) & 0xFC;

   TAS2563_WriteReg(TAS2563_PWR_CTL_REG, RegVal);
}


// =============================================================================
// TAS2563_Disable
// =============================================================================
/*!
 *
 * Disable DAC (power down).
 *
 */
// =============================================================================

void TAS2563_Disable (void)
{
   uint8_t RegVal;

   RegVal = TAS2563_ReadReg(TAS2563_PWR_CTL_REG) & 0xFC;

   // Software shutdown, registers keep their value

   TAS2563_WriteReg(TAS2563_PWR_CTL_REG, RegVal | 2);
}


// =============================================================================
// TAS2563_Init
// =============================================================================
/*!
 *
 * Initialize DAC. DAC is disabled when exiting this function.
 *
 */
// =============================================================================

uint8_t TAS2563_Init (void)
{
   // Change to page 0

   // Verify if DAC is present by trying to set page

   if(TAS2563_WriteReg (TAS2563_PAGE_REG, 0) == 0)
   {
      return 0;
   }

   // Reset

   TAS2563_WriteReg (TAS2563_SW_RESET_REG, 1);
   
   delay_ms(100);

   // Disable global address

   TAS2563_WriteReg (TAS2563_MISC_CFG2_REG, 0x20);  

   // Default to 16kHz, frame start at rising edge of FS and disable AUTO_RATE
   
   TAS2563_WriteReg (TAS2563_TDM_CFG0_REG, 0x02);

   TAS2563_Enable ();

   return 1;
}


// =============================================================================
// TAS2563_Config
// =============================================================================
/*!
 *
 * Configure CODEC inputs and outputs
 *
 * \param DevID - ID of the input/output to configure
 * \param Cfg  - Configuration mode
 *
 */
// =============================================================================

void TAS2563_Config (uint8_t DevID, uint8_t Cfg)
{
   switch(DevID)
   {
      case CDC_CFG_SPK:
      case CDC_CFG_DAI:        
      break;
   }
}


// =============================================================================
// TAS2563_Volume
// =============================================================================
/*!
 *
 * Set volume levels for inputs and outputs
 *
 * \param DevID - ID of the input/output to configure
 * \param Volume  - Volume level (0 - mute to 15 - max)
 *
 */
// =============================================================================

void TAS2563_Volume (uint8_t DevID, uint8_t Volume)
{
   uint8_t RegVal;

   if (Volume > 15)
   {
      return;
   }

   if (Volume)
   {
      TAS2563_Mute(0, 0);

      // Mask AMP_LEVEL bits
   
      RegVal = TAS2563_ReadReg(TAS2563_PB_CFG1_REG) & 0xC1;

      // Map Volume to AMP_LEVEL [1:0x1C]

      Volume = (Volume * 0x1C ) / 15;

      RegVal |= Volume << 1;

      TAS2563_WriteReg(TAS2563_PB_CFG1_REG, RegVal);

      return;
   }
   
   TAS2563_Mute(0, 1);
}


// =============================================================================
// TAS2563_SampleRate
// =============================================================================
/*!
 *
 * Set CODEC sample rate
 *
 * \param Rate - Sample rate (using CDC_SR_xx) constants
 *
 */
// =============================================================================

void TAS2563_SampleRate (uint8_t Rate)
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

        default:
            return;
    }

    // Read register and mask SAMP_RATE bits

    RegVal = TAS2563_ReadReg(TAS2563_TDM_CFG0_REG) & 0x0E;

    RegVal |= (Rate << 1);

    TAS2563_WriteReg(TAS2563_TDM_CFG0_REG, RegVal);

    TAS2563_Enable();
}


// =============================================================================
// TAS2563_Mute
// =============================================================================
/*!
 *
 * Mute/Unmute CODEC inputs or outputs
 *
 * \param DevID - ID of the channel (unused this DAC is mono)
 * \param Mute  - 0: Un-mute, Muted otherwise
 *
 */
// =============================================================================

void TAS2563_Mute (uint8_t DevID, uint8_t Mute)
{
   uint8_t RegVal;

   // Mask MODE bits

   RegVal = TAS2563_ReadReg(TAS2563_PWR_CTL_REG) & 0xFC;

   // Set mute mode

   if(Mute)
   {
      RegVal |= 1;
   }

   TAS2563_WriteReg(TAS2563_PWR_CTL_REG, RegVal);  
}

