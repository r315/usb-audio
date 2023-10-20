/*----------------------------------------------------------------------------
 *      U S B  -  K e r n e l
 *----------------------------------------------------------------------------
 * Name:    usbuser.c
 * Purpose: USB Custom User Module
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
 *---------------------------------------------------------------------------*/

#include "type.h"
#include "lpc17xx_hal.h"
#include "usb.h"
#include "usbcfg.h"
#include "usbcore.h"
#include "usbhw.h"
#include "usbuser.h"

#include "audio.h"

#if USB_DMA
static struct dma_pkt_info packet_info[P_C] __attribute__((section(".ep_ram")));
#endif

/*
 *  USB Power Event Callback
 *   Called automatically on USB Power Event
 *    Parameter:       power: On(TRUE)/Off(FALSE)
 */

#if USB_POWER_EVENT
void USB_Power_Event (uint32_t power) {}
#endif


/*
 *  USB Reset Event Callback
 *   Called automatically on USB Reset Event
 */

#if USB_RESET_EVENT
void USB_Reset_Event (void)
{
   USB_ResetCore ();
}
#endif


/*
 *  USB Suspend Event Callback
 *   Called automatically on USB Suspend Event
 */

#if USB_SUSPEND_EVENT
void USB_Suspend_Event (void) {}
#endif


/*
 *  USB Resume Event Callback
 *   Called automatically on USB Resume Event
 */

#if USB_RESUME_EVENT
void USB_Resume_Event (void) {}
#endif


/*
 *  USB Remote Wakeup Event Callback
 *   Called automatically on USB Remote Wakeup Event
 */

#if USB_WAKEUP_EVENT
void USB_WakeUp_Event (void) {}
#endif


/*
 *  USB Start of Frame Event Callback
 *   Called automatically on USB Start of Frame Event
 */

#if USB_SOF_EVENT
void USB_SOF_Event (void)
{
#if USB_DMA == 0
    if (USB_ReadEP(0x03, AUDIO_GetBuffer())){
        /* Data Available */
        AUDIO_AdvanceBuffer(P_S);
    }else {
        /* No Data */
        AUDIO_FlushBuffer();
    }
#endif
}
#endif


/*
 *  USB Error Event Callback
 *   Called automatically on USB Error Event
 *    Parameter:       error: Error Code
 */

#if USB_ERROR_EVENT
void USB_Error_Event (uint32_t error) {}
#endif


/*
 *  USB Set Configuration Event Callback
 *   Called automatically on USB Set Configuration Request
 */

#if USB_CONFIGURE_EVENT
void USB_Configure_Event (void)
{

   if (USB_Configuration)
   {  /* Check if USB is configured */
      /* add your code here */
   }
}
#endif


/*
 *  USB Set Interface Event Callback
 *   Called automatically on USB Set Interface Request
 */

#if USB_INTERFACE_EVENT
void USB_Interface_Event (void) {}
#endif


/*
 *  USB Set/Clear Feature Event Callback
 *   Called automatically on USB Set/Clear Feature Request
 */

#if USB_FEATURE_EVENT
void USB_Feature_Event (void) {}
#endif

/*
 *  USB Endpoint 1 Event Callback
 *   Called automatically on USB Endpoint 1 Event
 *    Parameter:       event
 */

void USB_EndPoint1 (uint32_t event) {}


/*
 *  USB Endpoint 2 Event Callback
 *   Called automatically on USB Endpoint 2 Event
 *    Parameter:       event
 */

void USB_EndPoint2 (uint32_t event) {}

/*
 *  USB Endpoint 3 Event Callback
 *   Called automatically on USB Endpoint 3 Event
 *    Parameter:       event
 */

void USB_EndPoint3 (uint32_t event)
{
#if USB_DMA
    struct dma_descriptor dd = {0};
    DBG_PIN_TOGGLE;
    switch(event){
        case USB_EVT_OUT_DMA_EOT:
            /* End of Transfer */
            if (USB_DMA_BufAdr (0x03) != (uint32_t)AUDIO_GetBuffer()){
                /* Data Available */
                AUDIO_AdvanceBuffer( P_C * P_S);
            }else{
                /* No Data */
                AUDIO_FlushBuffer();
            }
            break;

        case USB_EVT_OUT_DMA_ERR:
            /* Invalid DD on EP_RAM, UDCA may be corrupted */
            break;

        case USB_EVT_OUT_DMA_NDR:
            break;        

        default:
            return;
    }

    dd.w1.bits.length  = P_C;                   /* DMA Packet Count */
    dd.w1.bits.iso = 1;
    dd.buffer  = (uint32_t*)AUDIO_GetBuffer();  /* DMA Buffer Address */
    dd.w3.val = 0;                              /* Initial DMA Configuration */
    dd.infbuf = (uint32_t*)packet_info;         /* Packet Info Buffer Address */

    USB_DMA_Setup (0x03, &dd);                  /* Setup DMA */
    USB_DMA_Enable (0x03);                      /* Enable DMA */

#else
    event = event;
#endif
}


/*
 *  USB Endpoint 4 Event Callback
 *   Called automatically on USB Endpoint 4 Event
 *    Parameter:       event
 */

void USB_EndPoint4 (uint32_t event) {}


/*
 *  USB Endpoint 5 Event Callback
 *   Called automatically on USB Endpoint 5 Event
 *    Parameter:       event
 */

void USB_EndPoint5 (uint32_t event) {}


/*
 *  USB Endpoint 6 Event Callback
 *   Called automatically on USB Endpoint 6 Event
 *    Parameter:       event
 */

void USB_EndPoint6 (uint32_t event) {}


/*
 *  USB Endpoint 7 Event Callback
 *   Called automatically on USB Endpoint 7 Event
 *    Parameter:       event
 */

void USB_EndPoint7 (uint32_t event) {}


/*
 *  USB Endpoint 8 Event Callback
 *   Called automatically on USB Endpoint 8 Event
 *    Parameter:       event
 */

void USB_EndPoint8 (uint32_t event) {}


/*
 *  USB Endpoint 9 Event Callback
 *   Called automatically on USB Endpoint 9 Event
 *    Parameter:       event
 */

void USB_EndPoint9 (uint32_t event) {}


/*
 *  USB Endpoint 10 Event Callback
 *   Called automatically on USB Endpoint 10 Event
 *    Parameter:       event
 */

void USB_EndPoint10 (uint32_t event) {}


/*
 *  USB Endpoint 11 Event Callback
 *   Called automatically on USB Endpoint 11 Event
 *    Parameter:       event
 */

void USB_EndPoint11 (uint32_t event) {}


/*
 *  USB Endpoint 12 Event Callback
 *   Called automatically on USB Endpoint 12 Event
 *    Parameter:       event
 */

void USB_EndPoint12 (uint32_t event) {}


/*
 *  USB Endpoint 13 Event Callback
 *   Called automatically on USB Endpoint 13 Event
 *    Parameter:       event
 */

void USB_EndPoint13 (uint32_t event) {}


/*
 *  USB Endpoint 14 Event Callback
 *   Called automatically on USB Endpoint 14 Event
 *    Parameter:       event
 */

void USB_EndPoint14 (uint32_t event) {}


/*
 *  USB Endpoint 15 Event Callback
 *   Called automatically on USB Endpoint 15 Event
 *    Parameter:       event
 */

void USB_EndPoint15 (uint32_t event) {}
