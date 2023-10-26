// =============================================================================
/*!
 * \file       app/codec/codec.c
 *
 * Definitions and prototypes of the codec driver for the DA7211
 *
 * \author     Alexandre Sousa <alexandre.sousa@bithium.com>
 *
 * \version    2.0.1
 *
 * \copyright  Copyright &copy; &nbsp; 2014 Bithium S.A.
 */
// =============================================================================

#ifndef CODEC_H
#define CODEC_H

#include <stdint.h>

// =============================================================================
// General Definitions
// =============================================================================

#define CDC_CFG_MIC              0x00
#define CDC_CFG_MIC1             0x00
#define CDC_CFG_MIC2             0x01
#define CDC_CFG_SPK              0x02
#define CDC_CFG_SPKL             0x02
#define CDC_CFG_SPKR             0x03
#define CDC_CFG_DAI              0x04
#define CDC_CFG_MIC_BIAS         0x05
#define CDC_CFG_MIC1_GAIN        0x06
#define CDC_CFG_MIC2_GAIN        0x07

#define CDC_MODE_OFF             0x00
#define CDC_MODE_DIFF            0x01
#define CDC_MODE_SE_P            0x02
#define CDC_MODE_SE_N            0x03
#define CDC_MODE_PDM             0x04
#define CDC_MODE_I2S             0x05

// =============================================================================
// CODEC Sample rates
// =============================================================================

#define CDC_SR_8K                0x01
#define CDC_SR_11K               0x02
#define CDC_SR_12K               0x03
#define CDC_SR_16K               0x05
#define CDC_SR_22K               0x06
#define CDC_SR_24K               0x07
#define CDC_SR_32K               0x09
#define CDC_SR_44K               0x0A
#define CDC_SR_48K               0x0B
#define CDC_SR_88K               0x0E
#define CDC_SR_96K               0x0F

// =============================================================================
// CODEC Interface Structure
// =============================================================================

typedef struct CDC_Type_s
{
   uint8_t (*Init) (void);
   void    (*Config) (uint8_t DevID, uint8_t Mode);
   void    (*SampleRate) (uint8_t Rate);
   void    (*Enable) (void);
   void    (*Disable) (void);
   void    (*Volume) (uint8_t DevID, uint8_t Volume);
   void    (*Mute) (uint8_t DevID, uint8_t Mode);
   uint8_t (*WriteReg) (uint8_t Register, uint8_t Val);
   uint8_t (*ReadReg) (uint8_t Register);
}CDC_Type;

// =============================================================================

#endif // CODEC_H