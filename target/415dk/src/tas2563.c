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

#define TAS2563_I2C_ADDR       0x98 // 8-bit address

const CDC_Type tas2563 = 
{
   TAS2563_Init,
   TAS2563_Config,
   TAS2563_SampleRate,
   TAS2563_Enable,
   TAS2563_Disable,
   TAS2563_Volume,
   TAS2563_Mute
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

static uint8_t TAS2563_WriteReg (uint8_t Register, uint8_t Val)
{
   uint8_t Data[2];

   Data[0] = Register;
   Data[1] = Val;

   return I2C_Master_Write (TAS2563_I2C_ADDR, Data, 2) == 0;
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

static uint8_t TAS2563_ReadReg (uint8_t Register)
{
   uint8_t Data;

   I2C_Mem_Read (TAS2563_I2C_ADDR, Register, &Data, 1);

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
   // Power up DAC

   // Put it in mute
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
   // Power down DAC
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
   if( TAS2563_WriteReg (TAS2563_SW_RESET_REG, 1) == 0)
   {
      return 0;
   }

   TAS2563_Disable ();

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
 * \param Mode  - Configuration mode
 *
 */
// =============================================================================

void TAS2563_Config (uint8_t DevID, uint8_t Mode)
{
   
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
   if (Volume > 15)
   {
      return;
   }

   if (Volume)
   {
      // Unmute
   }
   else
   {
      // Mute
   }
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
}


// =============================================================================
// TAS2563_Mute
// =============================================================================
/*!
 *
 * Mute/Unmute CODEC inputs or outputs
 *
 * \param DevID - ID of the channel (unused this DAC is mono)
 * \param Mode  - 0: Un-mute, Muted otherwise
 *
 */
// =============================================================================

void TAS2563_Mute (uint8_t DevID, uint8_t Mode)
{
  
}

