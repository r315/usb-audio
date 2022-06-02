/*----------------------------------------------------------------------------
 *      U S B  -  K e r n e l
 *----------------------------------------------------------------------------
 * Name:    usbhw.c
 * Purpose: USB Hardware Layer Module for NXP's LPC17xx MCU
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
#include "LPC17xx.h" /* LPC17xx definitions */
#include "type.h"

#include "usb.h"
#include "usbcfg.h"
#include "usbreg.h"
#include "usbhw.h"
#include "usbcore.h"
#include "usbuser.h"

#include "logger.h"

#define EP_MSK_CTRL 0x0001 /* Control Endpoint Logical Address Mask */
#define EP_MSK_BULK 0xC924 /* Bulk Endpoint Logical Address Mask */
#define EP_MSK_INT  0x4492 /* Interrupt Endpoint Logical Address Mask */
#define EP_MSK_ISO  0x1248 /* Isochronous Endpoint Logical Address Mask */

#define P_EP(n) ((USB_EP_EVENT & (1 << (n))) ? USB_EndPoint##n : NULL)

#if USB_DMA

static uint32_t udca[USB_EP_NUM] __attribute__((section(".ep_ram"))) __attribute__ ((aligned (128)));       /* UDCA in USB RAM */
static struct dma_descriptor dd_iso_ram[DD_ISO_CNT] __attribute__((section(".ep_ram")));   /* Iso DMA Descr. */
//static uint8_t dd_niso_ram[DD_NISO_CNT] __attribute__((section(".ep_ram"))); /* Non-Iso DMA Descr. */
#endif

#define LOG_INFO(...)    logger("[USBHW]", LOG_INFO, __VA_ARGS__)

/* USB Endpoint Events Callback Pointers */
void (*const USB_P_EP[16]) (uint32_t event) = {
    P_EP (0),  P_EP (1),  P_EP (2),  P_EP (3),  
    P_EP (4),  P_EP (5),  P_EP (6),  P_EP (7),  
    P_EP (8),  P_EP (9),  P_EP (10), P_EP (11),
    P_EP (12), P_EP (13), P_EP (14), P_EP (15),
};

/*
 *  Get Endpoint Physical Address
 *    Parameters:      EPNum: Endpoint Number
 *                       EPNum.0..3: Address
 *                       EPNum.7:    Dir
 *    Return Value:    Endpoint Physical Address
 */

uint32_t EPAdr(uint32_t EPNum)
{
    uint32_t val;
    val = (EPNum & 0x0F) << 1;
    return (EPNum & 0x80) ? val + 1 : val;
}

/*
 *  Write Command
 *    Parameters:      cmd:   Command
 *    Return Value:    None
 */

void WrCmd(uint32_t cmd)
{
    LPC_USB->DevIntClr = CCEMTY_INT;
    LPC_USB->CmdCode = cmd;
    while ((LPC_USB->DevIntSt & CCEMTY_INT) == 0);
}

/*
 *  Write Command Data
 *    Parameters:      cmd:   Command
 *                     val:   Data
 *    Return Value:    None
 */

void WrCmdDat(uint32_t cmd, uint32_t val)
{
    LPC_USB->DevIntClr = CCEMTY_INT;
    LPC_USB->CmdCode = cmd;
    while ((LPC_USB->DevIntSt & CCEMTY_INT) == 0);
    LPC_USB->DevIntClr = CCEMTY_INT;
    LPC_USB->CmdCode = val;
    while ((LPC_USB->DevIntSt & CCEMTY_INT) == 0);
}

/*
 *  Write Command to Endpoint
 *    Parameters:      cmd:   Command
 *                     val:   Data
 *    Return Value:    None
 */

void WrCmdEP(uint32_t EPNum, uint32_t cmd)
{
    LPC_USB->DevIntClr = CCEMTY_INT;
    LPC_USB->CmdCode = CMD_SEL_EP(EPAdr(EPNum));
    while ((LPC_USB->DevIntSt & CCEMTY_INT) == 0);
    LPC_USB->DevIntClr = CCEMTY_INT;
    LPC_USB->CmdCode = cmd;
    while ((LPC_USB->DevIntSt & CCEMTY_INT) == 0);
}

/*
 *  Read Command Data
 *    Parameters:      cmd:   Command
 *    Return Value:    Data Value
 */

uint32_t RdCmdDat(uint32_t cmd)
{
    LPC_USB->DevIntClr = CCEMTY_INT | CDFULL_INT;
    LPC_USB->CmdCode = cmd;
    while ((LPC_USB->DevIntSt & CDFULL_INT) == 0);
    return (LPC_USB->CmdData);
}

/*
 *  USB Initialize Function
 *   Called by the User to initialize USB
 *    Return Value:    None
 */

void USB_Init(void)
{

    LPC_PINCON->PINSEL1 &= ~((3 << 26) | (3 << 28)); /* P0.29 D+, P0.30 D- */
    LPC_PINCON->PINSEL1 |= ((1 << 26) | (1 << 28));  /* PINSEL1 26.27, 28.29  = 01 */

    LPC_PINCON->PINSEL3 &= ~((3 << 4) | (3 << 28)); /* P1.18 GoodLink, P1.30 VBUS */
    LPC_PINCON->PINSEL3 |= ((1 << 4) | (2 << 28));  /* PINSEL3 4.5 = 01, 28.29 = 10 */

    LPC_PINCON->PINSEL4 &= ~((3 << 18)); /* P2.9 SoftConnect */
    LPC_PINCON->PINSEL4 |= ((1 << 18));  /* PINSEL4 18.19 = 01 */

    LPC_SC->PCONP |= (1UL << 31); /* USB PCLK -> enable USB Per.       */

    LPC_USB->USBClkCtrl = 0x12; /* Dev, AHB clock enable */
    while ((LPC_USB->USBClkSt & 0x12) != 0x12);

    NVIC_EnableIRQ(USB_IRQn); /* enable USB interrupt */

    USB_Reset();
    USB_SetAddress(0);
}

/*
 *  USB Connect Function
 *   Called by the User to Connect/Disconnect USB
 *    Parameters:      con:   Connect/Disconnect
 *    Return Value:    None
 */

void USB_Connect(uint32_t con)
{
    WrCmdDat(CMD_SET_DEV_STAT, DAT_WR_BYTE(con ? DEV_CON : 0));
}

/*
 *  USB Reset Function
 *   Called automatically on USB Reset
 *    Return Value:    None
 */

void USB_Reset(void)
{
    LPC_USB->EpInd = 0;                              // Setup packet size for Ep1:0
    LPC_USB->MaxPSize = USB_MAX_PACKET0;    
    LPC_USB->EpInd = 1;
    LPC_USB->MaxPSize = USB_MAX_PACKET0;
    while ((LPC_USB->DevIntSt & EP_RLZED_INT) == 0); // Wait for packet size update is completed

    LPC_USB->EpIntClr = 0xFFFFFFFF;
    LPC_USB->EpIntEn = 0xFFFFFFFF ^ USB_DMA_EP; // Slave mode for Ep6

    LPC_USB->DevIntClr = 0xFFFFFFFF;
    LPC_USB->DevIntEn = DEV_STAT_INT | EP_SLOW_INT |
                        (USB_SOF_EVENT ? FRAME_INT : 0) |
                        (USB_ERROR_EVENT ? ERR_INT : 0);

#if USB_DMA
    /* Clear USB Device Communication Area */
    for (uint8_t n = 0; n < USB_EP_NUM; n++){
        udca[n] = 0;
    }

    LPC_USB->UDCAH = (uint32_t)udca;    // Set USB Device Communication Area
    LPC_USB->DMARClr = 0xFFFFFFFF;      // Clear all endpoints DMA requests
    LPC_USB->EpDMADis = 0xFFFFFFFF;     // Disable DMA on all endpoints
    LPC_USB->EpDMAEn = USB_DMA_EP;      // Enable DMA for Ep6
    LPC_USB->EoTIntClr = 0xFFFFFFFF;    // Clear all end of transfer interrupts
    LPC_USB->NDDRIntClr = 0xFFFFFFFF;   // Clear all new DD requests interrupts
    LPC_USB->SysErrIntClr = 0xFFFFFFFF; // Clear all system errors
    LPC_USB->DMAIntEn = 0x00000007;     // Enable EOT, NDDR and ERR interrupts
#endif
}

/*
 *  USB Suspend Function
 *   Called automatically on USB Suspend
 *    Return Value:    None
 */

void USB_Suspend(void)
{
    /* Performed by Hardware */

#if USB_DMA
    /* Stop DMA */
    LPC_USB->EpDMAEn = 0;
    LPC_USB->DMARClr = 0xFFFFFFFF;
    LPC_USB->EoTIntClr = 0xFFFFFFFF;
    LPC_USB->NDDRIntClr = 0xFFFFFFFF;
    LPC_USB->SysErrIntClr = 0xFFFFFFFF;
    LPC_USB->EpIntEn = 0xFFFFFFFF;
    LPC_USB->DMARClr = 0xFFFFFFFF;
    LPC_USB->DMAIntEn = 0;
#endif
}

/*
 *  USB Resume Function
 *   Called automatically on USB Resume
 *    Return Value:    None
 */

void USB_Resume(void)
{
    /* Performed by Hardware */
}

/*
 *  USB Remote Wakeup Function
 *   Called automatically on USB Remote Wakeup
 *    Return Value:    None
 */

void USB_WakeUp(void)
{
    if (USB_DeviceStatus & USB_GETSTATUS_REMOTE_WAKEUP)
    {
        WrCmdDat(CMD_SET_DEV_STAT, DAT_WR_BYTE(DEV_CON));
    }
}

/*
 *  USB Remote Wakeup Configuration Function
 *    Parameters:      cfg:   Enable/Disable
 *    Return Value:    None
 */

void USB_WakeUpCfg(uint32_t cfg)
{
    /* Not needed */
}

/*
 *  USB Set Address Function
 *    Parameters:      adr:   USB Address
 *    Return Value:    None
 */

void USB_SetAddress(uint32_t adr)
{
    WrCmdDat(CMD_SET_ADDR, DAT_WR_BYTE(DEV_EN | adr)); /* Don't wait for next */
    WrCmdDat(CMD_SET_ADDR, DAT_WR_BYTE(DEV_EN | adr)); /*  Setup Status Phase */
}

/*
 *  USB Configure Function
 *    Parameters:      cfg:   Configure/Deconfigure
 *    Return Value:    None
 */

void USB_Configure(uint32_t cfg)
{
    WrCmdDat(CMD_CFG_DEV, DAT_WR_BYTE(cfg ? CONF_DVICE : 0));

    LPC_USB->ReEp = 0x00000003;
    while ((LPC_USB->DevIntSt & EP_RLZED_INT) == 0);
    LPC_USB->DevIntClr = EP_RLZED_INT;
}

/*
 *  Configure USB Endpoint according to Descriptor
 *    Parameters:      pEPD:  Pointer to Endpoint Descriptor
 *    Return Value:    None
 */

void USB_ConfigEP(USB_ENDPOINT_DESCRIPTOR *pEPD)
{
    uint32_t num;

    num = EPAdr(pEPD->bEndpointAddress);
    LPC_USB->ReEp |= (1 << num);
    LPC_USB->EpInd = num;
    LPC_USB->MaxPSize = pEPD->wMaxPacketSize;
    while ((LPC_USB->DevIntSt & EP_RLZED_INT) == 0);
    LPC_USB->DevIntClr = EP_RLZED_INT;
}

/*
 *  Set Direction for USB Control Endpoint
 *    Parameters:      dir:   Out (dir == 0), In (dir <> 0)
 *    Return Value:    None
 */

void USB_DirCtrlEP(uint32_t dir)
{
    /* Not needed */
}

/*
 *  Enable USB Endpoint
 *    Parameters:      EPNum: Endpoint Number
 *                       EPNum.0..3: Address
 *                       EPNum.7:    Dir
 *    Return Value:    None
 */

void USB_EnableEP(uint32_t EPNum)
{
    WrCmdDat(CMD_SET_EP_STAT(EPAdr(EPNum)), DAT_WR_BYTE(0));
}

/*
 *  Disable USB Endpoint
 *    Parameters:      EPNum: Endpoint Number
 *                       EPNum.0..3: Address
 *                       EPNum.7:    Dir
 *    Return Value:    None
 */

void USB_DisableEP(uint32_t EPNum)
{
    WrCmdDat(CMD_SET_EP_STAT(EPAdr(EPNum)), DAT_WR_BYTE(EP_STAT_DA));
}

/*
 *  Reset USB Endpoint
 *    Parameters:      EPNum: Endpoint Number
 *                       EPNum.0..3: Address
 *                       EPNum.7:    Dir
 *    Return Value:    None
 */

void USB_ResetEP(uint32_t EPNum)
{
    WrCmdDat(CMD_SET_EP_STAT(EPAdr(EPNum)), DAT_WR_BYTE(0));
}

/*
 *  Set Stall for USB Endpoint
 *    Parameters:      EPNum: Endpoint Number
 *                       EPNum.0..3: Address
 *                       EPNum.7:    Dir
 *    Return Value:    None
 */

void USB_SetStallEP(uint32_t EPNum)
{
    WrCmdDat(CMD_SET_EP_STAT(EPAdr(EPNum)), DAT_WR_BYTE(EP_STAT_ST));
}

/*
 *  Clear Stall for USB Endpoint
 *    Parameters:      EPNum: Endpoint Number
 *                       EPNum.0..3: Address
 *                       EPNum.7:    Dir
 *    Return Value:    None
 */

void USB_ClrStallEP(uint32_t EPNum)
{
    WrCmdDat(CMD_SET_EP_STAT(EPAdr(EPNum)), DAT_WR_BYTE(0));
}

/*
 *  Clear USB Endpoint Buffer
 *    Parameters:      EPNum: Endpoint Number
 *                       EPNum.0..3: Address
 *                       EPNum.7:    Dir
 *    Return Value:    None
 */

void USB_ClearEPBuf(uint32_t EPNum)
{
    WrCmdEP(EPNum, CMD_CLR_BUF);
}

/*
 *  Read USB Endpoint Data
 *    Parameters:      EPNum: Endpoint Number
 *                       EPNum.0..3: Address
 *                       EPNum.7:    Dir
 *                     pData: Pointer to Data Buffer
 *    Return Value:    Number of bytes read
 */

uint32_t USB_ReadEP(uint32_t EPNum, uint8_t *pData)
{
    uint32_t cnt, n;

    LPC_USB->Ctrl = ((EPNum & 0x0F) << 2) | CTRL_RD_EN;

    do{
        cnt = LPC_USB->RxPLen;
    } while ((cnt & PKT_RDY) == 0);

    cnt &= PKT_LNGTH_MASK;

    for (n = 0; n < (cnt + 3) / 4; n++)
    {
        *((uint32_t *)pData) = LPC_USB->RxData;
        pData += 4;
    }
    LPC_USB->Ctrl = 0;

    if (((EP_MSK_ISO >> EPNum) & 1) == 0)
    { /* Non-Isochronous Endpoint */
        WrCmdEP(EPNum, CMD_CLR_BUF);
    }
    return (cnt);
}

/*
 *  Write USB Endpoint Data
 *    Parameters:      EPNum: Endpoint Number
 *                       EPNum.0..3: Address
 *                       EPNum.7:    Dir
 *                     pData: Pointer to Data Buffer
 *                     cnt:   Number of bytes to write
 *    Return Value:    Number of bytes written
 */

uint32_t USB_WriteEP(uint32_t EPNum, uint8_t *pData, uint32_t cnt)
{
    uint32_t n;

    LPC_USB->Ctrl = ((EPNum & 0x0F) << 2) | CTRL_WR_EN;

    LPC_USB->TxPLen = cnt;

    for (n = 0; n < (cnt + 3) / 4; n++)
    {
        LPC_USB->TxData = *((uint32_t *)pData);
        pData += 4;
    }
    LPC_USB->Ctrl = 0;
    WrCmdEP(EPNum, CMD_VALID_BUF);
    return (cnt);
}

/*
 *  Get USB Last Frame Number
 *    Parameters:      None
 *    Return Value:    Frame Number
 */
uint32_t USB_GetFrame(void)
{
    uint32_t val;

    WrCmd(CMD_RD_FRAME);
    val = RdCmdDat(DAT_RD_FRAME);
    val = val | (RdCmdDat(DAT_RD_FRAME) << 8);

    return (val);
}

#if USB_DMA
/*
 *  Setup USB DMA Transfer for selected Endpoint
 *    Parameters:      EPNum: Endpoint Logical Number
 *                     ddc: Pointer to configuration DMA Descriptor
 *    Return Value:    TRUE - Success, FALSE - Error
 */
static uint8_t dd_idx = 0;
uint32_t USB_DMA_Setup(uint32_t EPNum, struct dma_descriptor *ddc)
{
    uint32_t ep_num, iso, n, dd_size;
    const uint32_t *dd_ram;
    struct dma_descriptor *nxt_dd;

    //iso = ddc->w1.bits.iso;      /* Iso or Non-Iso Descriptor */
    ep_num = EPAdr(EPNum);        /* Endpoint's Physical Address */
    //dd_size = DDSz[iso];
    //dd_ram = DDAdr[iso];

    //nxt_dd = (struct dma_descriptor*)udca_cp[ep_num]; /* Initial Descriptor */
    nxt_dd = &dd_iso_ram[(dd_idx++) & 7];

    nxt_dd->ddp = NULL;
    nxt_dd->buffer= ddc->buffer;
    nxt_dd->w1.val = ddc->w1.val;
    nxt_dd->w3.val = ddc->w3.val;
    nxt_dd->infbuf = ddc->infbuf;

    /* Go through Descriptor List */
    /* while (nxt_dd)
    {
        if (!pDD->Cfg.Type.Link)
        {                                       // Check for Linked Descriptors 
            n = ((uint32_t)nxt_dd - dd_ram) / dd_size; // Descriptor Index
            DDMemMap[iso] &= ~(1 << n);         // Unmark Memory Usage
        }
        nxt_dd = nxt_dd->ddp; // Next Descriptor
    } */

    /* Search for available Memory */
    /* for (n = 0; n < 32; n++){ 
        if ((DDMemMap[iso] & (1 << n)) == 0){
            break; // Memory found
        }
    } */
    
    //if (n == 32)
    //    return (FALSE); /* Memory not available */

    //DDMemMap[iso] |= 1 << n;          /* Mark Memory Usage */
    //nxt_dd = (struct dma_descriptor *)(dd_ram + n * dd_size); /* Next Descriptor */

    //if (ptr && pDD->Cfg.Type.Link){
    //    *((uint32_t *)(ptr + 0)) = nxt_dd;         /* Link in new Descriptor */
    //    *((uint32_t *)(ptr + 4)) |= 0x00000004; /* Next DD is Valid */
    //}else{
        //udca_cp[ep_num] = (uint32_t)nxt_dd; /* Save new Descriptor */
        udca[ep_num] = (uint32_t)nxt_dd; /* Update UDCA in USB */
    //}

    /* Fill in DMA Descriptor */
    //*((uint32_t *)nxt_dd++) = 0; /* Next DD Pointer */
    //*((uint32_t *)nxt_dd++) = pDD->Cfg.Type.ATLE |
    //                       (pDD->Cfg.Type.IsoEP << 4) |
    //                       (pDD->MaxSize << 5) |
    //                       (pDD->BufLen << 16);
    //*((uint32_t *)nxt_dd++) = pDD->BufAdr;
    //*((uint32_t *)nxt_dd++) = pDD->Cfg.Type.LenPos << 8;

    //if (iso){
    //    *((uint32_t *)nxt_dd) = pDD->InfoAdr;
    //}

    return TRUE; /* Success */
}

/*
 *  Enable USB DMA Endpoint
 *    Parameters:      EPNum: Endpoint Number
 *                       EPNum.0..3: Address
 *                       EPNum.7:    Dir
 *    Return Value:    None
 */

void USB_DMA_Enable(uint32_t EPNum)
{
    LPC_USB->EpDMAEn = 1 << EPAdr(EPNum);
}

/*
 *  Disable USB DMA Endpoint
 *    Parameters:      EPNum: Endpoint Number
 *                       EPNum.0..3: Address
 *                       EPNum.7:    Dir
 *    Return Value:    None
 */

void USB_DMA_Disable(uint32_t EPNum)
{
    LPC_USB->EpDMADis = 1 << EPAdr(EPNum);
}

/*
 *  Get USB DMA Endpoint Status
 *    Parameters:      EPNum: Endpoint Number
 *                       EPNum.0..3: Address
 *                       EPNum.7:    Dir
 *    Return Value:    DMA Status
 */

uint32_t USB_DMA_Status(uint32_t EPNum)
{
    uint32_t ptr, val;

    ptr = ((uint32_t*)LPC_USB->UDCAH)[EPAdr(EPNum)]; /* Current Descriptor */
    if (ptr == 0)
        return (USB_DMA_INVALID);

    val = *((uint32_t *)(ptr + 3 * 4)); /* Status Information */
    switch ((val >> 1) & 0x0F)
    {
    case 0x00: /* Not serviced */
        return (USB_DMA_IDLE);
    case 0x01: /* Being serviced */
        return (USB_DMA_BUSY);
    case 0x02: /* Normal Completition */
        return (USB_DMA_DONE);
    case 0x03: /* Data Under Run */
        return (USB_DMA_UNDER_RUN);
    case 0x08: /* Data Over Run */
        return (USB_DMA_OVER_RUN);
    case 0x09: /* System Error */
        return (USB_DMA_ERROR);
    }

    return (USB_DMA_UNKNOWN);
}

/*
 *  Get USB DMA Endpoint Current Buffer Address
 *    Parameters:      EPNum: Endpoint Number
 *                       EPNum.0..3: Address
 *                       EPNum.7:    Dir
 *    Return Value:    DMA Address (or -1 when DMA is Invalid)
 */

uint32_t USB_DMA_BufAdr(uint32_t EPNum)
{
    uint32_t ptr, val;

    ptr = ((uint32_t*)LPC_USB->UDCAH)[EPAdr(EPNum)]; /* Current Descriptor */
    if (ptr == 0)
    {
        return ((uint32_t)(-1)); /* DMA Invalid */
    }

    val = *((uint32_t *)(ptr + 2 * 4)); /* Buffer Address */
    return (val);                       /* Current Address */
}

/*
 *  Get USB DMA Endpoint Current Buffer Count
 *   Number of transfered Bytes or Iso Packets
 *    Parameters:      EPNum: Endpoint Number
 *                       EPNum.0..3: Address
 *                       EPNum.7:    Dir
 *    Return Value:    DMA Count (or -1 when DMA is Invalid)
 */

uint32_t USB_DMA_BufCnt(uint32_t EPNum)
{
    uint32_t ptr, val;

    ptr = ((uint32_t*)LPC_USB->UDCAH)[EPAdr(EPNum)]; /* Current Descriptor */
    if (ptr == 0)
    {
        return ((uint32_t)(-1)); /* DMA Invalid */
    }
    val = *((uint32_t *)(ptr + 3 * 4)); /* Status Information */
    return (val >> 16);                 /* Current Count */
}

#endif /* USB_DMA */

/*
 *  USB Interrupt Service Routine
 */

void USB_IRQHandler(void)
{
    uint32_t disr, val, n, m;
    uint32_t episr, episrCur;

    disr = LPC_USB->DevIntSt; /* Device Interrupt Status */

    /* Device Status Interrupt (Reset, Connect change, Suspend/Resume) */
    if (disr & DEV_STAT_INT)
    {
        LPC_USB->DevIntClr = DEV_STAT_INT;
        WrCmd(CMD_GET_DEV_STAT);
        val = RdCmdDat(DAT_GET_DEV_STAT); /* Device Status */
        if (val & DEV_RST)
        { /* Reset */
            USB_Reset();
#if USB_RESET_EVENT
            USB_Reset_Event();
#endif
            LOG_INFO("Reset\n");
        }
        if (val & DEV_CON_CH)
        { /* Connect change */
#if USB_POWER_EVENT
            USB_Power_Event(val & DEV_CON);
#endif
            LOG_INFO("Connection changed\n");
        }
        if (val & DEV_SUS_CH)
        { /* Suspend/Resume */
            if (val & DEV_SUS)
            { /* Suspend */
                USB_Suspend();
#if USB_SUSPEND_EVENT
                USB_Suspend_Event();
#endif
                LOG_INFO("Suspend\n");
            }
            else
            { /* Resume */
                USB_Resume();
#if USB_RESUME_EVENT
                USB_Resume_Event();
#endif
                LOG_INFO("Resume\n");
            }
        }
        return;
    }

#if USB_SOF_EVENT
    /* Start of Frame Interrupt, called every 1ms*/
    if (disr & FRAME_INT)
    {
        USB_SOF_Event();
        LPC_USB->DevIntClr = FRAME_INT;
    }
#endif

#if USB_ERROR_EVENT
    /* Error Interrupt */
    if (disr & ERR_INT)
    {
        WrCmd(CMD_RD_ERR_STAT);
        val = RdCmdDat(DAT_RD_ERR_STAT);
        USB_Error_Event(val);
    }
#endif

    /* Endpoint's Slow Interrupt */
    if (disr & EP_SLOW_INT)
    {
        episrCur = 0;
        episr = LPC_USB->EpIntSt;

        for (n = 0; n < USB_EP_NUM; n++)
        { /* Check All Endpoints */
            if (episr == episrCur)
                break; /* break if all EP interrupts handled */

            if (episr & (1 << n))
            {
                episrCur |= (1 << n);
                m = n >> 1;

                LPC_USB->EpIntClr = (1 << n);
                while ((LPC_USB->DevIntSt & CDFULL_INT) == 0);
                val = LPC_USB->CmdData;

                if ((n & 1) == 0)
                { /* OUT Endpoint */
                    if (n == 0)
                    { /* Control OUT Endpoint */
                        //DBG_PIN_LOW;
                        if (val & EP_SEL_STP)
                        { /* Setup Packet */
                            if (USB_P_EP[0])
                            {
                                LOG_INFO("Control Out: Setup Packet\n");
                                USB_P_EP[0](USB_EVT_SETUP);
                                continue;
                            }
                        }
                        //DBG_PIN_HIGH;
                    }

                    if (USB_P_EP[m])
                    {
                        LOG_INFO("Ep %d Out SLOW\n", m);
                        USB_P_EP[m](USB_EVT_OUT);
                    }
                }
                else
                { /* IN Endpoint */
                    if (USB_P_EP[m])
                    {
                        LOG_INFO("Ep %d In SLOW\n", m);
                        USB_P_EP[m](USB_EVT_IN);
                    }
                }
            }
        }
        LPC_USB->DevIntClr = EP_SLOW_INT;
    }

#if USB_DMA

    if (LPC_USB->DMAIntSt & EOT_INT)
    { /* End of Transfer Interrupt */
        val = LPC_USB->EoTIntSt;
        for (n = 2; n < USB_EP_NUM; n++)
        { /* Check All Endpoints */
            if (val & (1 << n))
            {
                m = n >> 1;
                if ((n & 1) == 0)
                { /* OUT Endpoint */
                    if (USB_P_EP[m])
                    {
                        LOG_INFO("Ep %d Out DMA EOT\n", m);
                        USB_P_EP[m](USB_EVT_OUT_DMA_EOT);
                    }
                }
                else
                { /* IN Endpoint */
                    if (USB_P_EP[m])
                    {
                        LOG_INFO("Ep %d In DMA EOT\n", m);
                        USB_P_EP[m](USB_EVT_IN_DMA_EOT);
                    }
                }
            }
        }
        LPC_USB->EoTIntClr = val;
    }

    if (LPC_USB->DMAIntSt & NDD_REQ_INT)
    { /* New DD Request Interrupt */
        val = LPC_USB->NDDRIntSt;
        for (n = 2; n < USB_EP_NUM; n++)
        { /* Check All Endpoints */
            if (val & (1 << n))
            {
                m = n >> 1;
                if ((n & 1) == 0)
                { /* OUT Endpoint */
                    if (USB_P_EP[m])
                    {
                        LOG_INFO("Ep %d Out DMA NDD\n", m);
                        USB_P_EP[m](USB_EVT_OUT_DMA_NDR);
                    }
                }
                else
                { /* IN Endpoint */
                    if (USB_P_EP[m])
                    {
                        LOG_INFO("Ep %d In DMA NDD\n", m);
                        USB_P_EP[m](USB_EVT_IN_DMA_NDR);
                    }
                }
            }
        }
        LPC_USB->NDDRIntClr = val;
    }

    if (LPC_USB->DMAIntSt & SYS_ERR_INT)
    { /* System Error Interrupt */
        val = LPC_USB->SysErrIntSt;
        for (n = 2; n < USB_EP_NUM; n++)
        { /* Check All Endpoints */
            if (val & (1 << n))
            {
                m = n >> 1;
                if ((n & 1) == 0)
                { /* OUT Endpoint */
                    if (USB_P_EP[m])
                    {
                        LOG_INFO("Ep %d Out DMA ERR\n", m);
                        USB_P_EP[m](USB_EVT_OUT_DMA_ERR);
                    }
                }
                else
                { /* IN Endpoint */
                    if (USB_P_EP[m])
                    {
                        LOG_INFO("Ep %d In DMA ERR\n", m);
                        USB_P_EP[m](USB_EVT_IN_DMA_ERR);
                    }
                }
            }
        }
        LPC_USB->SysErrIntClr = val;
    }

#endif /* USB_DMA */
}
