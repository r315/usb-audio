#include <stdint.h>
#include <stddef.h>
#include "lpc17xx_hal.h"
#include "i2s.h"
#include "audio.h"

#include "usbuser.h"

#define TEST_DATA_SIZE      485
#define VOLUME_CONST        128


static uint8_t Mute;    /* Mute State */
static uint16_t VolCur = 0x0100;       /* Volume Current Value */
static uint32_t Volume; /* Volume Level */

static short *DataBuf;
static uint16_t DataOut; /* Data Out Index */
static uint16_t DataIn;  /* Data In Index */
static uint8_t DataRun; /* Data Stream Run State */

static i2sbus_t i2s;
static uint32_t vum;    /* VU Meter */
static uint32_t tick;   /* Time Tick */
static uint16_t audio_buffer[B_S];
static uint16_t test_buffer[TEST_DATA_SIZE];

/**
 * @brief Callback to copy 16bit samples to 32bit fifo
 * 
 * @param dst   Fifo address, no increment is required 
 * @param len   number of transfer to fifo
 */
static void extdacCallBack(uint32_t *dst, uint32_t len){
    uint32_t Index, sample = 0;
    uint16_t *buf = (uint16_t*)i2s.txbuffer;
    Index = i2s.rdidx;

    #if EXT_DAC_MONO
    uint8_t msb = 0;
    #endif

    while(len){
        #if EXT_DAC_MONO
        if(msb & 1){
            sample |= (uint32_t)buf[Index++];
            *dst = sample;
            len--;
        }else{
            sample = (uint32_t)buf[Index++];
            sample <<= 16;
        }
        msb++;
        #else
            sample = (uint16_t)buf[Index++];
            *dst = (sample << 16) | sample;
            len--;
        #endif

        if (Index == i2s.buf_len){
            Index = 0;
        }
    }

    i2s.rdidx = Index;
}

/**
 * @brief 
 * 
 * 
 * @param dst 
 * @param len 
 */
static void dacCallBack(uint32_t *src, uint32_t len){
    int32_t cnt, sample;

    LPC_TIM0->IR = 1; /* Clear Interrupt Flag */

    if (DataRun){                                        /* Data Stream is running */
        sample = DataBuf[DataOut];               /* Get Audio Sample */
        cnt = (DataIn - DataOut) & (B_S - 1); /* Buffer Data Count */

        if (cnt == (B_S - P_C * P_S))
        {
            /* Too much Data in Buffer */
            DataOut++; /* Skip one Sample */
        }
      
        if (cnt > (P_C * P_S))
        {
            /* Still enough Data in Buffer */
            DataOut++; /* Update Data Out Index */
        }
      
        DataOut &= B_S - 1; /* Adjust Buffer Out Index */
      
        if (sample < 0)
            vum -= sample; /* Accumulate Neg Value */
        else
            vum += sample; /* Accumulate Pos Value */
      
        sample *= Volume; /* Apply Volume Level */
        sample >>= 16;    /* Adjust Value */
        sample += 0x8000; /* Add Bias */
        sample &= 0xFFFF; /* Mask Value */
    }else{
        sample = 0x8000; /* DAC Middle Point */
    }

    if (Mute){
        sample = 0x8000; /* DAC Middle Point */
    }

    LPC_DAC->CR = sample & 0xFFC0; /* Set Speaker Output */

    if ((tick++ & 0x03FF) == 0)
    {                 /* On every 1024th Tick */

        /** Check for Minimum Level
         * if minimum set volume to 0 else
         *  Chained Volume Level and multiply by pot value*/
        Volume = (VolCur == 0x8000) ? 0 : VolCur * 128;
        vum = 0;         /* Clear VUM */
   }
}
/**
 * @brief 
 * 
 */
static void AUDIO_InitDac(void){
    volatile uint32_t pclk;

    /**
     *  P0.25, A0.0, function 01, P0.26 AOUT, function 10.
     *  Setting DAC function on pin also enables DAC
     * */    
    LPC_PINCON->PINSEL1 &= ~((0x03 << 18) | (0x03 << 20));
    LPC_PINCON->PINSEL1 |= ((0x01 << 18) | (0x02 << 20));

    LPC_DAC->CR = 0x00008000; /* DAC Output set to Middle Point */

    /**
     *  By default, the PCLKSELx value is zero, thus, the PCLK for
     *  all the peripherals is 1/4 of the SystemCoreClock.
     *  Bit 2~3 is for TIMER0 */
    switch ((LPC_SC->PCLKSEL0 >> 2) & 0x03)
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
}

/**
 * @brief 
 * 
 */
static void AUDIO_InitExtDac(void){
    i2s.sample_rate = 32000;
    i2s.channels = 2;
    i2s.data_size = 16;
    i2s.mode = (I2S_TX_EN_MASTER | I2S_MCLK_OUT);
    i2s.bus = I2S_BUS2;
    //i2s.txbuffer = NULL;
    //i2s.buf_len = 0;
    i2s.rdidx = DataOut;
    i2s.txcp = extdacCallBack;

    for(int i = 0; i < TEST_DATA_SIZE; i++){
        test_buffer[i] = i;
    }

    I2S_Init(&i2s);

    i2s.txbuffer = (uint32_t*)test_buffer;
    i2s.buf_len = TEST_DATA_SIZE;
    I2S_Start(&i2s);
}

/**
 * @brief 
 * 
 */
void AUDIO_Init(void){
    
    DataOut = 0;
    DataIn  = 0;
    DataRun = 0;
    Mute = 0;
    DataBuf = (short *)audio_buffer;

    AUDIO_InitDac();
    AUDIO_InitExtDac();
}

/**
 * @brief 
 * 
 * @param buffer 
 * @param len 
 */
void AUDIO_Start(uint16_t *buffer, uint32_t len){
    if(len == 0 || buffer == NULL){
        return;
    }

    i2s.txbuffer = (uint32_t*)test_buffer;
    i2s.buf_len = TEST_DATA_SIZE;

    I2S_Start(&i2s);
}

uint8_t *AUDIO_GetBuffer(void){
    #if USB_DMA
    return (uint8_t*)DataBuf + 2 * DataIn;
    #else
    return (uint8_t*)&DataBuf[DataIn];
    #endif
}

void AUDIO_AdvanceBuffer(uint32_t cnt){
    DataIn = (DataIn + cnt) & (B_S - 1);     /* Update Data In Index */
    if (((DataIn - DataOut) & (B_S - 1)) == (B_S / 2)){
        DataRun = 1; /* Data Stream running */
    }
}

void AUDIO_FlushBuffer(void){
    DataRun = 0;      /* Data Stream not running */
    DataOut = DataIn; /* Initialize Data Indexes */
}

void AUDIO_SetVolume(uint16_t vol){
    VolCur = vol;
}

uint16_t AUDIO_GetVolume(void){
    return VolCur;
}

void AUDIO_SetMute(uint8_t mute){
    Mute = mute;
}

uint8_t AUDIO_GetMute(void){
    return Mute;
}

/**
 * @brief Interrupt handlers
 * 
 */
void I2S_IRQHandler(void){
    I2S_Handler(&i2s);
}

/*
 * Timer Counter 0 Interrupt Service Routine
 *   executed each 31.25us (32kHz frequency)
 */
void TIMER0_IRQHandler(void){
    
    LPC_TIM0->IR = 1; /* Clear Interrupt Flag */
    //DBG_PIN_TOGGLE;
   
    dacCallBack(NULL, 0);
}