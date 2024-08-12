#ifndef _tas2563_
#define _tas2563_

#include <stdint.h>
#include "audio.h"

#define TAS2563_I2C_ADDR_GLOBAL         0x48   // 7-bit, 
#define TAS2563_I2C_ADDR0               0x4C
#define TAS2563_I2C_ADDR1               0x4D
#define TAS2563_I2C_ADDR2               0x4E
#define TAS2563_I2C_ADDR3               0x4F

/* Register Address Map */

#define TAS2563_PAGE_REG                0x00
#define TAS2563_SW_RESET_REG            0x01
#define TAS2563_PWR_CTL_REG             0x02
#define TAS2563_PB_CFG1_REG             0x03
#define TAS2563_MISC_CFG1_REG           0x04
#define TAS2563_MISC_CFG2_REG           0x05

#define TAS2563_TDM_CFG0_REG            0x06
#define TAS2563_TDM_CFG1_REG            0x07
#define TAS2563_TDM_CFG2_REG            0x08
#define TAS2563_TDM_CFG3_REG            0x09
#define TAS2563_TDM_CFG4_REG            0x0A
#define TAS2563_TDM_CFG5_REG            0x0B
#define TAS2563_TDM_CFG6_REG            0x0C
#define TAS2563_TDM_CFG7_REG            0x0D
#define TAS2563_TDM_CFG8_REG            0x0E
#define TAS2563_TDM_CFG9_REG            0x0F
#define TAS2563_TDM_CFG10_REG           0x10
#define TAS2563_TDM_DET_REG             0x11    // RO

#define TAS2563_LIM_CFG0_REG            0x12
#define TAS2563_LIM_CFG1_REG            0x13
#define TAS2563_BOP_CFG0_REG            0x14
#define TAS2563_BOP_CFG1_REG            0x15
#define TAS2563_BIL_ICLA_CFG0_REG       0x16
#define TAS2563_BIL_ICLA_CFG1_REG       0x17
#define TAS2563_GAIN_ICLA_CFG0_REG      0x18    // RO
#define TAS2563_ICLA_CFG1_REG           0x19    // RO

#define TAS2563_INT_MASK0_REG           0x1A
#define TAS2563_INT_MASK1_REG           0x1B
#define TAS2563_INT_MASK2_REG           0x1C
#define TAS2563_INT_MASK3_REG           0x1D
#define TAS2563_INT_LIVE0_REG           0x1F    // RO
#define TAS2563_INT_LIVE1_REG           0x20    // RO
#define TAS2563_INT_LIVE3_REG           0x21    // RO
#define TAS2563_INT_LIVE4_REG           0x22    // RO
#define TAS2563_INT_LTCH0_REG           0x24    // RO
#define TAS2563_INT_LTCH1_REG           0x25    // RO
#define TAS2563_INT_LTCH3_REG           0x26    // RO
#define TAS2563_INT_LTCH4_REG           0x27    // RO

#define TAS2563_VBAT_MSB_REG            0x2A    // RO
#define TAS2563_VBAT_LSB_REG            0x2B    // RO
#define TAS2563_TEMP_REG                0x2C    // RO
#define TAS2563_INT_CLK_CFG_REG         0x30
#define TAS2563_DIN_PD_REG              0x31
#define TAS2563_MISC0_REG               0x32

#define TAS2563_BOOST_CFG1_REG          0x33
#define TAS2563_BOOST_CFG2_REG          0x34
#define TAS2563_BOOST_CFG3_REG          0x35

#define TAS2563_MISC1_REG               0x3B
#define TAS2563_TG_CFG0_REG             0x3F
#define TAS2563_BST_ILIM_CFG0_REG       0x40
#define TAS2563_PDM_CFG0_REG            0x41
#define TAS2563_PDM_CFG3_REG            0x42
#define TAS2563_ASI2_CFG0_REG           0x43
#define TAS2563_ASI2_CFG1_REG           0x44
#define TAS2563_ASI2_CFG2_REG           0x45
#define TAS2563_ASI2_CFG3_REG           0x46
#define TAS2563_PVDD_MSB_DSP_REG        0x49    // RO
#define TAS2563_PVDD_LSB_DSP_REG        0x4A    // RO
#define TAS2563_REV_ID_REG              0x7D    // RO
#define TAS2563_I2C_CKSUM_REG           0x7E
#define TAS2563_BOOK_REG                0x7F

// =============================================================================
// DAC Interface functions
// =============================================================================

extern uint8_t tas2563_Init (uint8_t Addr);
extern uint8_t tas2563_Config (uint8_t DevID, uint8_t Mode);
extern void tas2563_SampleRate (uint32_t Rate);
extern void tas2563_Enable (void);
extern void tas2563_Disable (void);
extern void tas2563_Volume (uint8_t DevID, uint8_t Volume);
extern void tas2563_Mute (uint8_t DevID, uint8_t Mode);

// =============================================================================
// DAC Global variables
// =============================================================================

extern const audio_codec_t tas2563;

// =============================================================================

#endif   // TAS2563_H