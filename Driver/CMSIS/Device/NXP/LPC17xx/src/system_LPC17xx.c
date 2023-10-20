
#include <stdint.h>
#include "LPC17xx.h"

#define SC_SCS_OSCEN        (1 << 5)
#define SC_SCS_OSCSTAT      (1 << 6)

#define SC_CLKSRCSEL_IRC    0
#define SC_CLKSRCSEL_PLL0   1
#define SC_CLKSRCSEL_RTC    2

#define SC_FLASHCFG_FLASHTIM_1  (0 << 12) // CCLK < 20MHz
#define SC_FLASHCFG_FLASHTIM_2  (1 << 12) // CCLK < 40MHz
#define SC_FLASHCFG_FLASHTIM_3  (2 << 12) // CCLK < 60MHz
#define SC_FLASHCFG_FLASHTIM_4  (3 << 12) // CCLK < 80MHz
#define SC_FLASHCFG_FLASHTIM_5  (4 << 12) // CCLK < 100MHz
#define SC_FLASHCFG_FLASHTIM_6  (5 << 12) // Safe option

// PCONP
#define SC_PCONP_PCTIM0         (1 << 1)
#define SC_PCONP_PCTIM1         (1 << 2)
#define SC_PCONP_PCUART0        (1 << 3)
#define SC_PCONP_PCUART1        (1 << 4)
#define SC_PCONP_PCPWM1         (1 << 6)
#define SC_PCONP_PCI2C0         (1 << 7)
#define SC_PCONP_PCSPI          (1 << 8)
#define SC_PCONP_PCRTC          (1 << 9)
#define SC_PCONP_PCSSP1         (1 << 10)
#define SC_PCONP_PCADC          (1 << 12)
#define SC_PCONP_PCCAN1         (1 << 13)
#define SC_PCONP_PCCAN2         (1 << 14)
#define SC_PCONP_PCGPIO         (1 << 15)
#define SC_PCONP_PCRIT          (1 << 16)
#define SC_PCONP_PCMCPWM        (1 << 17)
#define SC_PCONP_PCQEI          (1 << 18)
#define SC_PCONP_PCI2C1         (1 << 19)
#define SC_PCONP_PCSSP0         (1 << 21)
#define SC_PCONP_PCTIM2         (1 << 22)
#define SC_PCONP_PCTIM3         (1 << 23)
#define SC_PCONP_PCUART2        (1 << 24)
#define SC_PCONP_PCUART3        (1 << 25)
#define SC_PCONP_PCI2C2         (1 << 26)
#define SC_PCONP_PCI2S          (1 << 27)
#define SC_PCONP_PCGPDMA        (1 << 29)
#define SC_PCONP_PCENET         (1 << 30)
#define SC_PCONP_PCUSB          (1 << 31)

#define XTAL        (12000000UL)        /* Oscillator frequency               */
#define OSC_CLK     (      XTAL)        /* Main oscillator frequency          */
#define RTC_CLK     (   32000UL)        /* RTC oscillator frequency           */
#define IRC_OSC     ( 4000000UL)        /* Internal RC oscillator frequency   */


#define PLLCLK  100

#if PLLCLK
#define CLOCK_SETUP 1
#define PLL0_SETUP  1
#define FLASH_SETUP 1
#endif

#if PLLCLK == 48
#define CCLKCFG_Val     (6 - 1)
#define PCLKSEL0_Val    0xAAAAAAAA  /* Peripheral Clock Selection CCLK/2  */
#define PCLKSEL1_Val    0xAAAAAAAA  /* 00=CCLK/4 01=CCLK 10=CCLK/2 11=CCLK/8 */
#define CLKSRCSEL_Val   SC_CLKSRCSEL_PLL0
#define USBCLKCFG_Val   0
#define CLKOUTCFG_Val   0
#define FLASHCFG_Val    SC_FLASHCFG_FLASHTIM_3
#define Nvalue          1
#define Mvalue          12
#elif PLLCLK == 72
#define CCLKCFG_Val     (4 - 1)
#define PCLKSEL0_Val    0xAAAAAAAA  /* Peripheral Clock Selection CCLK/2  */
#define PCLKSEL1_Val    0xAAAAAAAA  /* 00=CCLK/4 01=CCLK 10=CCLK/2 11=CCLK/8 */
#define CLKSRCSEL_Val   SC_CLKSRCSEL_PLL0
#define USBCLKCFG_Val   0
#define CLKOUTCFG_Val   0
#define FLASHCFG_Val    SC_FLASHCFG_FLASHTIM_4
#define Mvalue          24
#define Nvalue          2
#elif PLLCLK == 80
#define CCLKCFG_Val     (5 - 1)
#define PCLKSEL0_Val    0xAAAAAAAA  /* Peripheral Clock Selection CCLK/2  */
#define PCLKSEL1_Val    0xAAAAAAAA  /* 00=CCLK/4 01=CCLK 10=CCLK/2 11=CCLK/8 */
#define CLKSRCSEL_Val   SC_CLKSRCSEL_PLL0
#define USBCLKCFG_Val   0
#define CLKOUTCFG_Val   0
#define FLASHCFG_Val    SC_FLASHCFG_FLASHTIM_3
#define Mvalue          50
#define Nvalue          3
#elif PLLCLK == 100
#define CCLKCFG_Val     (3 - 1)
#define PCLKSEL0_Val    0xAAAAAAAA  /* Peripheral Clock Selection CCLK/2  */
#define PCLKSEL1_Val    0xAAAAAAAA  /* 00=CCLK/4 01=CCLK 10=CCLK/2 11=CCLK/8 */
#define CLKSRCSEL_Val   SC_CLKSRCSEL_PLL0
#define USBCLKCFG_Val   0
#define CLKOUTCFG_Val   0
#define FLASHCFG_Val    SC_FLASHCFG_FLASHTIM_3
#define Mvalue          25
#define Nvalue          2
#endif

uint32_t SystemCoreClock;

void SystemCoreClockUpdate (void)
{

	SystemCoreClock = IRC_OSC;

 /* Determine clock frequency according to clock register values             */
	if (((LPC_SC->PLL0STAT >> 24)&3)==3) 
	{
		switch (LPC_SC->CLKSRCSEL & 0x03) 
		{
			case 0:
      		case 3:
        	SystemCoreClock = (IRC_OSC * 
                          ((2 * ((LPC_SC->PLL0STAT & 0x7FFF) + 1)))  /
                          (((LPC_SC->PLL0STAT >> 16) & 0xFF) + 1)    /
                          ((LPC_SC->CCLKCFG & 0xFF)+ 1));
						  break;

      		case 1: /* Main oscillator => PLL0            */
        	SystemCoreClock = (OSC_CLK * 
                          ((2 * ((LPC_SC->PLL0STAT & 0x7FFF) + 1)))  /
                          (((LPC_SC->PLL0STAT >> 16) & 0xFF) + 1)    /
                          ((LPC_SC->CCLKCFG & 0xFF)+ 1));
						  break;

			case 2: /* RTC oscillator => PLL0             */
        	SystemCoreClock = (RTC_CLK * 
                          ((2 * ((LPC_SC->PLL0STAT & 0x7FFF) + 1)))  /
                          (((LPC_SC->PLL0STAT >> 16) & 0xFF) + 1)    /
                          ((LPC_SC->CCLKCFG & 0xFF)+ 1));
						  break;
    	}
  	} 
	else 
	{
    	switch (LPC_SC->CLKSRCSEL & 0x03) 
		{
      		case 0:
      		case 3:/* Reserved, default to Internal RC   */
        	SystemCoreClock = IRC_OSC / ((LPC_SC->CCLKCFG & 0xFF)+ 1);
			break;

			case 1:/* Main oscillator => PLL0            */
        	SystemCoreClock = OSC_CLK / ((LPC_SC->CCLKCFG & 0xFF)+ 1);
			break;

	 		case 2:/* RTC oscillator => PLL0             */
        	SystemCoreClock = RTC_CLK / ((LPC_SC->CCLKCFG & 0xFF)+ 1);
			break;
    	}
	}
}

void SystemInit (void)
{
#if (CLOCK_SETUP)
    LPC_SC->SCS = SC_SCS_OSCEN;  
    while ((LPC_SC->SCS & SC_SCS_OSCSTAT) == 0);    /* Wait for Oscillator to be ready    */
  
    LPC_SC->CCLKCFG   = CCLKCFG_Val;      /* Setup Clock Divider                */
    LPC_SC->PCLKSEL0  = PCLKSEL0_Val;     /* Peripheral Clock Selection         */
    LPC_SC->PCLKSEL1  = PCLKSEL1_Val;
    LPC_SC->CLKSRCSEL = CLKSRCSEL_Val;    /* Select Clock Source for PLL0       */

#if (PLL0_SETUP)
#define PLL0CFG_Val     (Nvalue - 1) << 16 | (Mvalue - 1)

    LPC_SC->PLL0CFG   = PLL0CFG_Val;      /* configure PLL0                     */
    LPC_SC->PLL0FEED  = 0xAA;
    LPC_SC->PLL0FEED  = 0x55;

    LPC_SC->PLL0CON   = 0x01;             /* PLL0 Enable                        */
    LPC_SC->PLL0FEED  = 0xAA;
    LPC_SC->PLL0FEED  = 0x55;
    while (!(LPC_SC->PLL0STAT & (1<<26)));/* Wait for PLOCK0                    */

    LPC_SC->PLL0CON   = 0x03;             /* PLL0 Enable & Connect              */
    LPC_SC->PLL0FEED  = 0xAA;
    LPC_SC->PLL0FEED  = 0x55;
    while (!(LPC_SC->PLL0STAT & ((1<<25) | (1<<24))));/* Wait for PLLC0_STAT & PLLE0_STAT */
#endif

#if (PLL1_SETUP)
    LPC_SC->PLL1CFG   = PLL1CFG_Val;
    LPC_SC->PLL1FEED  = 0xAA;
    LPC_SC->PLL1FEED  = 0x55;

    LPC_SC->PLL1CON   = 0x01;             /* PLL1 Enable                        */
    LPC_SC->PLL1FEED  = 0xAA;
    LPC_SC->PLL1FEED  = 0x55;
    while (!(LPC_SC->PLL1STAT & (1<<10)));/* Wait for PLOCK1                    */

    LPC_SC->PLL1CON   = 0x03;             /* PLL1 Enable & Connect              */
    LPC_SC->PLL1FEED  = 0xAA;
    LPC_SC->PLL1FEED  = 0x55;
    while (!(LPC_SC->PLL1STAT & ((1<< 9) | (1<< 8))));/* Wait for PLLC1_STAT & PLLE1_STAT */
#else
    LPC_SC->USBCLKCFG = USBCLKCFG_Val;    /* Setup USB Clock Divider            */
#endif

    LPC_SC->PCONP     = SC_PCONP_PCGPIO;

    LPC_SC->CLKOUTCFG = CLKOUTCFG_Val;    /* Clock Output Configuration         */
#endif

#if (FLASH_SETUP)                  /* Flash Accelerator Setup            */
    LPC_SC->FLASHCFG  = (LPC_SC->FLASHCFG & ~0x0000F000) | FLASHCFG_Val;
#endif
}
