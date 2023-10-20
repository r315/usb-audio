/*----------------------------------------------------------------------------
 *      Name:    usbaudio.h
 *      Purpose: USB Audio Demo Definitions
 *      Version: V1.10
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

#define EXT_DAC_MONO        0

void AUDIO_Init(void);
uint8_t *AUDIO_GetBuffer(void);
void AUDIO_AdvanceBuffer(uint32_t cnt);
void AUDIO_FlushBuffer(void);
void AUDIO_SetVolume(uint16_t vol);
uint16_t AUDIO_GetVolume(void);
uint8_t AUDIO_GetMute(void);
void AUDIO_SetMute(uint8_t);

