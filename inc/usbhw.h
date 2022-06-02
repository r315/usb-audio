/*----------------------------------------------------------------------------
 *      U S B  -  K e r n e l
 *----------------------------------------------------------------------------
 * Name:    usbhw.h
 * Purpose: USB Hardware Layer Definitions
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
 *          V1.20 Added USB_ClearEPBuf
 *          V1.00 Initial Version
 *----------------------------------------------------------------------------*/

#ifndef __USBHW_H__
#define __USBHW_H__

/* USB RAM Definitions */
//#define USB_RAM_ADR 0x20080000 /* USB RAM Start Address */
//#define USB_RAM_SZ 0x00004000  /* USB RAM Size (4kB) */

/* DMA Endpoint Descriptors */
#define UDCA_SZ 128
#define DD_NISO_CNT 16                        /* Non-Iso EP DMA Descr. Count (max. 32) */
#define DD_ISO_CNT 8                          /* Iso EP DMA Descriptor Count (max. 32) */
#define DD_NISO_SZ 16                         /* Non-Iso DMA Descriptor size in bytes */
#define DD_ISO_SZ 20                          /* Iso DMA Descriptor size in bytes */
#define DD_NISO_RAM_SZ (DD_NISO_CNT * DD_NISO_SZ)         /* Non-Iso DMA Descr. Size */
#define DD_ISO_RAM_SZ (DD_ISO_CNT * DD_ISO_SZ)            /* Iso DMA Descriptor Size */
#define DD_RAM_SZ (UDCA_SZ + DD_NISO_RAM_SZ + DD_ISO_RAM_SZ)  /* Descr. Size */

/* USB Error Codes */
#define USB_ERR_PID 0x0001    /* PID Error */
#define USB_ERR_UEPKT 0x0002  /* Unexpected Packet */
#define USB_ERR_DCRC 0x0004   /* Data CRC Error */
#define USB_ERR_TIMOUT 0x0008 /* Bus Time-out Error */
#define USB_ERR_EOP 0x0010    /* End of Packet Error */
#define USB_ERR_B_OVRN 0x0020 /* Buffer Overrun */
#define USB_ERR_BTSTF 0x0040  /* Bit Stuff Error */
#define USB_ERR_TGL 0x0080    /* Toggle Bit Error */

/* USB DMA Status Codes */
#define USB_DMA_INVALID 0x0000   /* DMA Invalid - Not Configured */
#define USB_DMA_IDLE 0x0001      /* DMA Idle - Waiting for Trigger */
#define USB_DMA_BUSY 0x0002      /* DMA Busy - Transfer in progress */
#define USB_DMA_DONE 0x0003      /* DMA Transfer Done (no Errors)*/
#define USB_DMA_OVER_RUN 0x0004  /* Data Over Run */
#define USB_DMA_UNDER_RUN 0x0005 /* Data Under Run (Short Packet) */
#define USB_DMA_ERROR 0x0006     /* Error */
#define USB_DMA_UNKNOWN 0xFFFF   /* Unknown State */

/* USB DMA Descriptor */
struct dma_descriptor{
    struct dma_descriptor *ddp;   // W0: Next_DD_pointer
    union{
        struct {
            uint32_t mode   : 2;  // 0: normal, 1: ATLE
            uint32_t valid  : 1;  // Next_DD_Valid 
            uint32_t rsv    : 1;
            uint32_t iso    : 1;  // 1:isochronous
            uint32_t size   : 11; // Max_packet_size
            uint32_t length : 16; // DMA_buffer_length
        }bits;
        uint32_t val;
    }w1;
    uint32_t *buffer;             // W2: DMA_buffer_start_addr
    union{
        struct{
            uint32_t retired : 1; // DD_Retired
            uint32_t status  : 4; // DD_status
            uint32_t valid   : 1; // Packet_valid
            uint32_t ls      : 1; // LS_byte_extracted
            uint32_t ms      : 1; // MS_byte_extracted
            uint32_t length  : 6; // Message_length_position
            uint32_t rsv     : 2;
            uint32_t count   : 16;// Present_DMA_count
        }bits;
        uint32_t val;             // W3
    }w3;
    uint32_t *infbuf;             // W4: isochronous_packet_memory_address
};


/* USB Hardware Functions */
extern void USB_Init(void);
extern void USB_Connect(uint32_t con);
extern void USB_Reset(void);
extern void USB_Suspend(void);
extern void USB_Resume(void);
extern void USB_WakeUp(void);
extern void USB_WakeUpCfg(uint32_t cfg);
extern void USB_SetAddress(uint32_t adr);
extern void USB_Configure(uint32_t cfg);
extern void USB_ConfigEP(USB_ENDPOINT_DESCRIPTOR *pEPD);
extern void USB_DirCtrlEP(uint32_t dir);
extern void USB_EnableEP(uint32_t EPNum);
extern void USB_DisableEP(uint32_t EPNum);
extern void USB_ResetEP(uint32_t EPNum);
extern void USB_SetStallEP(uint32_t EPNum);
extern void USB_ClrStallEP(uint32_t EPNum);
extern void USB_ClearEPBuf(uint32_t EPNum);
extern uint32_t USB_ReadEP(uint32_t EPNum, uint8_t *pData);
extern uint32_t USB_WriteEP(uint32_t EPNum, uint8_t *pData, uint32_t cnt);
extern uint32_t USB_DMA_Setup(uint32_t EPNum, struct dma_descriptor *ddp);
extern void USB_DMA_Enable(uint32_t EPNum);
extern void USB_DMA_Disable(uint32_t EPNum);
extern uint32_t USB_DMA_Status(uint32_t EPNum);
extern uint32_t USB_DMA_BufAdr(uint32_t EPNum);
extern uint32_t USB_DMA_BufCnt(uint32_t EPNum);
extern uint32_t USB_GetFrame(void);
extern void USB_IRQHandler(void);

#endif /* __USBHW_H__ */
