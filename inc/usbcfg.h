/*----------------------------------------------------------------------------
 *      U S B  -  K e r n e l
 *----------------------------------------------------------------------------
 * Name:    usbcfg.h
 * Purpose: USB Custom Configuration
 * Version: V1.20
 *----------------------------------------------------------------------------
 *      This software is supplied "AS IS" without any warranties, express,
 *      implied or statutory, including but not limited to the implied
 *      warranties of fitness for purpose, satisfactory quality and
 *      noninfringement. Keil extends you a royalty-free right to reproduce
 *      and distribute executable files created using this software for use
 *      on NXP Semiconductors LPC family microcontroller devices only. Nothing 
 *      else gives you the right to use this software.
 *
 * Copyright (c) 2009 Keil - An ARM Company. All rights reserved.
 *----------------------------------------------------------------------------
 * History:
 *          V1.20 Added vendor specific support
 *          V1.00 Initial Version
 *----------------------------------------------------------------------------*/

#ifndef __USBCFG_H__
#define __USBCFG_H__


#define USB_POWER           0
#define USB_IF_NUM          4
#define USB_EP_NUM          32
#define USB_MAX_PACKET0     64
#define USB_DMA_EP          0x00000040  // Physical Ep 6 (Logical 3 Out)

#define USB_POWER_EVENT     0
#define USB_RESET_EVENT     1
#define USB_SUSPEND_EVENT   0
#define USB_RESUME_EVENT    0
#define USB_WAKEUP_EVENT    0
#define USB_SOF_EVENT       1
#define USB_ERROR_EVENT     0
#define USB_EP_EVENT        0x0009
#define USB_CONFIGURE_EVENT 0
#define USB_INTERFACE_EVENT 0
#define USB_FEATURE_EVENT   0

#define USB_CLASS           1
#define USB_HID             0
#define USB_HID_IF_NUM      0
#define USB_MSC             0
#define USB_MSC_IF_NUM      0
#define USB_AUDIO           1
#define USB_ADC_CIF_NUM     0
#define USB_ADC_SIF1_NUM    1
#define USB_ADC_SIF2_NUM    2
#define USB_CDC  			0
#define USB_CDC_CIF_NUM     0
#define USB_CDC_DIF_NUM     1
#define USB_CDC_BUFSIZE     64

#define USB_VENDOR          0

#define USB_PID             0x4002
#define USB_VID             0x1FC9


#endif  /* __USBCFG_H__ */
