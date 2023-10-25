// =============================================================================
/*!
 * \file       tas2563.h
 *
 * This file contains the definitions for the DAC driver for the TAS2563
 *
 * \author     Hugo Reis <hugo.reis@bithium.com>
 *
 * \version    x.x.x
 *
 * \copyright  Copyright &copy; &nbsp; 2023 Bithium S.A.
 */
// =============================================================================

#ifndef TAS2563_H
#define TAS2563_H

#include "codec.h"

/* Register Address Map */

#define TAS2563_PAGE_REG         0x00
#define TAS2563_SW_RESET_REG     0x01
#define TAS2563_PWR_CTL_REG      0x02
#define TAS2563_PB_CFG1_REG      0x03
#define TAS2563_MISC_CFG1_REG    0x04
#define TAS2563_MISC_CFG2_REG    0x05
#define TAS2563_TDM_CFG0_REG     0x06
#define TAS2563_TDM_CFG1_REG     0x07
#define TAS2563_TDM_CFG2_REG     0x08
#define TAS2563_TDM_CFG3_REG     0x09
#define TAS2563_TDM_CFG4_REG     0x0A
#define TAS2563_TDM_CFG5_REG     0x0B
#define TAS2563_TDM_CFG6_REG     0x0C
#define TAS2563_TDM_CFG7_REG     0x0D
#define TAS2563_TDM_CFG8_REG     0x0E
#define TAS2563_TDM_CFG9_REG     0x0F
#define TAS2563_TDM_CFG10_REG    0x10
#define TAS2563_TDM_DET_REG      0x11
#define TAS2563_LIM_CFG0_REG     0x12
#define TAS2563_LIM_CFG1_REG     0x13
#define TAS2563_BOP_CFG0_REG     0x14
#define TAS2563_BOP_CFG1_REG     0x15
#define TAS2563_BIL_ICLA_CFG0_REG   0x16
#define TAS2563_BIL_ICLA_CFG1_REG   0x17
#define TAS2563_GAIN_ICLA_CFG0_REG  0x18
#define TAS2563_ICLA_CFG1_REG    0x19
#define TAS2563_INT_MASK0_REG    0x1A
#define TAS2563_INT_MASK1_REG    0x1B
#define TAS2563_INT_MASK2_REG    0x1C
#define TAS2563_INT_MASK3_REG    0x1D
#define TAS2563_INT_LIVE0_REG    0x1F
#define TAS2563_INT_LIVE1_REG    0x20
#define TAS2563_INT_LIVE3_REG    0x21
#define TAS2563_INT_LIVE4_REG    0x22
#define TAS2563_INT_LTCH0_REG    0x24
#define TAS2563_INT_LTCH1_REG    0x25
#define TAS2563_INT_LTCH3_REG    0x26
#define TAS2563_INT_LTCH4_REG    0x27
#define TAS2563_VBAT_MSB_REG     0x2A
#define TAS2563_VBAT_LSB_REG     0x2B
#define TAS2563_TEMP_REG         0x2C
#define TAS2563_INT_CLK_CFG_REG  0x30
#define TAS2563_DIN_PD_REG       0x31
#define TAS2563_MISC0_REG        0x32
#define TAS2563_BOOST_CFG1_REG   0x33
#define TAS2563_BOOST_CFG2_REG   0x34
#define TAS2563_BOOST_CFG3_REG   0x35
#define TAS2563_MISC1_REG        0x3B
#define TAS2563_TG_CFG0_REG      0x3F
#define TAS2563_BST_ILIM_CFG0_REG   0x40
#define TAS2563_PDM_CFG0_REG     0x41
#define TAS2563_PDM_CFG3_REG     0x42
#define TAS2563_ASI2_CFG0_REG    0x43
#define TAS2563_ASI2_CFG1_REG    0x44
#define TAS2563_ASI2_CFG2_REG    0x45
#define TAS2563_ASI2_CFG3_REG    0x46
#define TAS2563_PVDD_MSB_DSP_REG 0x49
#define TAS2563_PVDD_LSB_DSP_REG 0x4A
#define TAS2563_REV_ID_REG       0x7D
#define TAS2563_I2C_CKSUM_REG    0x7E
#define TAS2563_BOOK_REG         0x7F


// =============================================================================
// DAC Interface functions
// =============================================================================

extern void TAS2563_Enable (void);

extern void TAS2563_Disable (void);

extern uint8_t TAS2563_Init (void);

extern void TAS2563_Config (uint8_t DevID, uint8_t Mode);

extern void TAS2563_Volume (uint8_t DevID, uint8_t Volume);

extern void TAS2563_SampleRate (uint8_t Rate);

extern void TAS2563_Mute (uint8_t DevID, uint8_t Mode);

// =============================================================================
// DAC Global variables
// =============================================================================

extern const CDC_Type tas2563;

// =============================================================================

#endif   // TAS2563_H
