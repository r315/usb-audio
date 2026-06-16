/**
  **************************************************************************
  * @file     audio_class.h
  * @brief    usb audio class file
  **************************************************************************
  *                       Copyright notice & Disclaimer
  *
  * The software Board Support Package (BSP) that is made available to
  * download from Artery official website is the copyrighted work of Artery.
  * Artery authorizes customers to use, copy, and distribute the BSP
  * software and its related documentation for the purpose of design and
  * development in conjunction with Artery microcontrollers. Use of the
  * software is governed by this copyright notice and the following disclaimer.
  *
  * THIS SOFTWARE IS PROVIDED ON "AS IS" BASIS WITHOUT WARRANTIES,
  * GUARANTEES OR REPRESENTATIONS OF ANY KIND. ARTERY EXPRESSLY DISCLAIMS,
  * TO THE FULLEST EXTENT PERMITTED BY LAW, ALL EXPRESS, IMPLIED OR
  * STATUTORY OR OTHER WARRANTIES, GUARANTEES OR REPRESENTATIONS,
  * INCLUDING BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY,
  * FITNESS FOR A PARTICULAR PURPOSE, OR NON-INFRINGEMENT.
  *
  **************************************************************************
  */

 /* define to prevent recursive inclusion -------------------------------------*/
#ifndef __AUDIO_CLASS_H
#define __AUDIO_CLASS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "usb_std.h"
#include "usbd_core.h"
#include "audio_conf.h"

/**
  * @brief endpoint define
  */
#define USBD_AUDIO_MIC_IN_EPT            0x81
#define USBD_AUDIO_SPK_OUT_EPT           0x02
#define USBD_AUDIO_FEEDBACK_EPT          0x83

/**
  * @brief endpoint support max size
  */
#define AUDIO_REMAIN_SIZE                40
#define AUDIO_MIC_IN_MAXPACKET_SIZE      (AUDIO_SUPPORT_MAX_FREQ * AUDIO_MIC_CHANEL_NUM * (AUDIO_MIC_DEFAULT_BITW / 8) + AUDIO_REMAIN_SIZE)
#define AUDIO_SPK_OUT_MAXPACKET_SIZE     (AUDIO_SUPPORT_MAX_FREQ * AUDIO_SPK_CHANEL_NUM * (AUDIO_SPK_DEFAULT_BITW / 8) + AUDIO_REMAIN_SIZE)
#define AUDIO_FEEDBACK_MAXPACKET_SIZE    0x3
#define FEEDBACK_REFRESH_TIME            0x8
/**
  * @brief request type define
  */
#define AUDIO_REQ_CONTROL_INTERFACE      0x01
#define AUDIO_REQ_CONTROL_ENDPOINT       0x02
#define AUDIO_REQ_CONTROL_MASK           0x03

/**
  * @brief audio set cur type define
  */
#define AUDIO_MUTE_CONTROL               0x01
#define AUDIO_VOLUME_CONTROL             0x02
#define AUDIO_FREQ_SET_CONTROL           0x03

/**
  * @brief audio descriptor type
  */
#define AUDIO_DESCRIPTOR_TYPE             0x21
#define AUDIO_DESCRIPTOR_SIZE             0x09

typedef struct {
    uint16_t volume;
    uint16_t volume_limits[3]; /*min, max, resolution */
    uint32_t freq;
    uint32_t alt_setting;
    uint8_t data[AUDIO_SPK_OUT_MAXPACKET_SIZE]; /* different size for mic? */
    uint8_t mute;
}audio_channel_type;

/**
  * @brief usb audio control struct
  */
typedef struct
{
  uint8_t enpd;
  uint8_t interface;
  uint8_t request_no;
  audio_channel_type spk;
  audio_channel_type mic;

  uint8_t audio_cmd;
  uint32_t audio_cmd_len;

  uint8_t g_audio_cur[64];
  uint8_t audio_feedback[AUDIO_FEEDBACK_MAXPACKET_SIZE+1];
   __IO uint16_t audio_feedback_state;
   __IO uint8_t audio_spk_out_stage;
}usb_audio_type;

extern usbd_class_handler audio_class_handler;

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */
#ifdef __cplusplus
}
#endif

#endif
