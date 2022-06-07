/*----------------------------------------------------------------------------
 * Name:    usbmain.c
 * Purpose: USB Audio Class Demo
 * Version: V1.20
 *----------------------------------------------------------------------------
 *      This software is supplied "AS IS" without any warranties, express,
 *      implied or statutory, including but not limited to the implied
 *      warranties of fitness for purpose, satisfactory quality and
 *      noninfringement. Keil extends you a royalty-free right to reproduce
 *      and distribute executable files created using this software for use
 *      on NXP Semiconductors LPC microcontroller devices only. Nothing else
 *      gives you the right to use this software.
 *
 * Copyright (c) 2009 Keil - An ARM Company. All rights reserved.
 *---------------------------------------------------------------------------*/

// https://www.engineersgarage.com/usb-audio-using-lpc1768-part-20-21/

#include <stdint.h>
#include "LPC17xx.h"
#include "type.h"
#include "usb.h"
#include "usbcfg.h"
#include "usbcore.h"
#include "usbhw.h"
#include "usbuser.h"
#include "audio.h"
#include "uart.h"

#include "logger.h"

static serialbus_t uart;

int pstring(const char* str){
   while(*str){
      UART_PutChar(&uart, *str++);
   }
   return 0;
}

/*****************************************************************************
**   Main Function  main()
******************************************************************************/
int main (void)
{
   SystemInit ();
   /* SystemClockUpdate() updates the SystemCoreClock variable */
   SystemClockUpdate ();

   AUDIO_Init();

   DBG_PIN_INIT;
   
   uart.bus = UART_BUS0;
   uart.speed = 115200;
   UART_Init(&uart);

   USB_Init ();        /* USB Initialization */
   USB_Connect (TRUE); /* USB Connect */

   /********* The main Function is an endless loop ***********/
   log("\e[2J\rStarting\n\n");
   
   while (1){
      log_flush(pstring);
   }
}

/******************************************************************************
**                            End Of File
******************************************************************************/
