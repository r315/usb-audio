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
#include "usbaudio.h"
#include "uart.h"

#include "logger.h"

static serialbus_t uart;

uint8_t Mute;    /* Mute State */
uint32_t Volume; /* Volume Level */

static uint16_t audio_buffer[B_S];

short *DataBuf;

uint16_t DataOut; /* Data Out Index */
uint16_t DataIn;  /* Data In Index */

uint8_t DataRun; /* Data Stream Run State */

uint32_t VUM;    /* VU Meter */
uint32_t Tick;   /* Time Tick */


/*
 * Get Potenciometer Value
 */
uint16_t get_potval (void)
{
#if 0
   uint32_t val;

   LPC_ADC->CR |= 0x01000000; /* Start A/D Conversion */
   do
   {
      val = LPC_ADC->GDR;             /* Read A/D Data Register */
   } while ((val & 0x80000000) == 0); /* Wait for end of A/D Conversion */
   LPC_ADC->CR &= ~0x01000000;        /* Stop A/D Conversion */
   return (val >> 8) & 0xF8) +     /* Extract Potenciometer Value */
            ((val >> 7) & 0x08);
#else
   return 128;
#endif
}

int pstring(const char* str){
   while(*str){
      UART_PutChar(&uart, *str++);
   }
   return 0;
}

/*
 * Timer Counter 0 Interrupt Service Routine
 *   executed each 31.25us (32kHz frequency)
 */

void TIMER0_IRQHandler (void)
{
   long val;
   uint32_t cnt;

   LPC_TIM0->IR = 1; /* Clear Interrupt Flag */
   //DBG_PIN_TOGGLE;
   
   if (DataRun)
   {                                        /* Data Stream is running */
      val = DataBuf[DataOut];               /* Get Audio Sample */
      cnt = (DataIn - DataOut) & (B_S - 1); /* Buffer Data Count */

      if (cnt == (B_S - P_C * P_S))
      {             /* Too much Data in Buffer */
         DataOut++; /* Skip one Sample */
      }
      
      if (cnt > (P_C * P_S))
      {             /* Still enough Data in Buffer */
         DataOut++; /* Update Data Out Index */
      }
      
      DataOut &= B_S - 1; /* Adjust Buffer Out Index */
      
      if (val < 0)
         VUM -= val; /* Accumulate Neg Value */
      else
         VUM += val; /* Accumulate Pos Value */
      
      val *= Volume; /* Apply Volume Level */
      val >>= 16;    /* Adjust Value */
      val += 0x8000; /* Add Bias */
      val &= 0xFFFF; /* Mask Value */
   }
   else
   {
      val = 0x8000; /* DAC Middle Point */
   }

   if (Mute)
   {
      val = 0x8000; /* DAC Middle Point */
   }

   LPC_DAC->CR = val & 0xFFC0; /* Set Speaker Output */

   if ((Tick++ & 0x03FF) == 0)
   {                 /* On every 1024th Tick */
      if (VolCur == 0x8000)
      {              /* Check for Minimum Level */
         Volume = 0; /* No Sound */
      }
      else
      {
         Volume = VolCur * get_potval (); /* Chained Volume Level and multiply by pot value*/
      }

      val = VUM >> 20; /* Scale Accumulated Value */
      VUM = 0;         /* Clear VUM */

      if (val > 7)
         val = 7; /* Limit Value */
   }
}


/*****************************************************************************
**   Main Function  main()
******************************************************************************/
int main (void)
{
   volatile uint32_t pclkdiv, pclk;

   SystemInit ();
   /* SystemClockUpdate() updates the SystemCoreClock variable */
   SystemClockUpdate ();

   DataOut = 0;
   DataIn  = 0;
   DataRun = 0;
   DataBuf = (short *)audio_buffer;

   LPC_PINCON->PINSEL1 &= ~((0x03 << 18) | (0x03 << 20));
   /* P0.25, A0.0, function 01, P0.26 AOUT, function 10 */
   LPC_PINCON->PINSEL1 |= ((0x01 << 18) | (0x02 << 20));

   /* Enable CLOCK into ADC controller */
   LPC_SC->PCONP |= (1 << 12);

   LPC_ADC->CR = 0x00200E04; /* ADC: 10-bit AIN2 @ 4MHz */
   LPC_DAC->CR = 0x00008000; /* DAC Output set to Middle Point */

   /* By default, the PCLKSELx value is zero, thus, the PCLK for
   all the peripherals is 1/4 of the SystemCoreClock. */
   /* Bit 2~3 is for TIMER0 */
   pclkdiv     = (LPC_SC->PCLKSEL0 >> 2) & 0x03;
   switch (pclkdiv)
   {
      case 0x00:
      default:
         pclk = SystemCoreClock / 4;
         break;
      case 0x01:
         pclk = SystemCoreClock;
         break;
      case 0x02:
         pclk = SystemCoreClock / 2;
         break;
      case 0x03:
         pclk = SystemCoreClock / 8;
         break;
   }

   LPC_TIM0->MR0 = pclk / USB_AUDIO_DATA_FREQ - 1; /* TC0 Match Value 0 */
   LPC_TIM0->MCR = 3;                    /* TCO Interrupt and Reset on MR0 */
   LPC_TIM0->TCR = 1;                    /* TC0 Enable */

   NVIC_EnableIRQ (TIMER0_IRQn);

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
