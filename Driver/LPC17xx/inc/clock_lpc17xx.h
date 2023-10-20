#ifndef _clock_lpc17xx_h_
#define _clock_lpc17xx_h_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>


#define XTAL        (12000000UL)        /* Oscillator frequency               */
#define OSC_CLK     (      XTAL)        /* Main oscillator frequency          */
#define RTC_CLK     (   32000UL)        /* RTC oscillator frequency           */
#define IRC_OSC     ( 4000000UL)        /* Internal RC oscillator frequency   */


#define CLOCK_OUT {LPC_SC->CLKOUTCFG = (1<<8); /* CCLK/2, CLKOU_EN */     \
                  LPC_PINCON->PINSEL3 |= (1<<22);}    /* P1.27 CLKOUT */  \


//PCLKSEL0 Bits
#define PCLKSEL0_PCLK_TIMER0_pos      2
#define PCLKSEL0_PCLK_TIMER1_pos      4
#define PCLKSEL0_PCLK_UART0_pos       6
#define PCLKSEL0_PCLK_UART1_pos       8
#define PCLKSEL0_PCLK_PWM1_pos        12
#define PCLKSEL0_PCLK_I2C0_pos        14
#define PCLKSEL0_PCLK_SPI_pos         16
#define PCLKSEL0_PCLK_SSP1_pos        20
#define PCLKSEL0_PCLK_DAC_pos         22
#define PCLKSEL0_PCLK_ADC_pos         24
#define PCLKSEL0_PCLK_CAN1_pos        26
#define PCLKSEL0_PCLK_CAN2_pos        28
#define PCLKSEL0_PCLK_ACF_pos         30

#define PCLKSEL0_PCLK_TIMER0_MSK      ( 3 << PCLKSEL0_PCLK_TIMER0_pos)
#define PCLKSEL0_PCLK_TIMER1_MSK      ( 3 << PCLKSEL0_PCLK_TIMER1_pos)


//PCLKSEL1 Bits
#define PCLKSEL1_PCLK_RIT_pos         26
#define PCLKSEL1_PCLK_TIMER2_pos      12
#define PCLKSEL1_PCLK_TIMER3_pos      14
#define PCLKSEL1_PCLK_I2S_pos	      22

#define PCLKSEL1_PCLK_TIMER2_MSK      ( 3 << PCLKSEL1_PCLK_TIMER2_pos)
#define PCLKSEL1_PCLK_TIMER3_MSK      ( 3 << PCLKSEL1_PCLK_TIMER3_pos)

// PCLK Peripherals index
typedef enum pclknum {
    PCLK_WDT       = 0,
    PCLK_TIMER0,
    PCLK_TIMER1,
    PCLK_UART0,
    PCLK_UART1,
    PCLK_PWM1      = 6,
    PCLK_I2C0,
    PCLK_SPI,
    PCLK_SSP1      = 10,
    PCLK_DAC,
    PCLK_ADC,
    PCLK_CAN1,
    PCLK_CAN2,
    PCLK_ACF,
    PCLK_QEI,
    PCLK_GPIOINT,
    PCLK_PCB,
    PCLK_I2C1,
    PCLK_SSP0      = 21,
    PCLK_TIMER2,
    PCLK_TIMER3,
    PCLK_UART2,
    PCLK_UART3,
    PCLK_I2C2,
    PCLK_I2S,
    PCLK_RIT       = 29,
    PCLK_SYSCON,
    PCLK_MC
}pclknum_e;


// CCLK Dividers
#define PCLK_1 		1
#define PCLK_2 		2
#define PCLK_4 		0
#define PCLK_8 		3

//
#define CCLK_DIV1   1
#define CCLK_DIV2   2
#define CCLK_DIV4   0
#define CCLK_DIV8   3 

// Core clock
#define CCLK_100	100
#define CCLK_80		80
#define CCLK_72		72
#define CCLK_48		48

#define __USE_SYSTICK

 /**
 * @brief get current system clock
 **/
 uint32_t CLOCK_GetCCLK(void);

/**
 * @brief set system clock, for values < 48Mhz internal osccilator is activated
 *        on invalid clock settings defaults to 48Mhz
 **/
void CLOCK_Init(uint32_t cclk);

/**
 * @brief set PLL1 to generat 48Mhz clock for usb 
 **/
void CLOCK_InitUSBCLK(void);

/**
 * @brief delay ms function
 *         this function uses systick
 **/
void CLOCK_DelayMs(uint32_t ms);
 
 /**
 * @brief returns ms passed from the last powerup/reset
 **/

uint32_t CLOCK_GetTicks(void);

/**
 * @brief returns ticks passed from the parameter ticks
 **/
uint32_t CLOCK_ElapsedTicks(uint32_t ticks);

/**
 *  Get the current clock (Hz) for the supplied peripheral
 **/
uint32_t CLOCK_GetPCLK(pclknum_e peripheral);

/**
 *  Set cclk divider for the supplied peripheral
 **/
void CLOCK_SetPCLK(pclknum_e peripheral, uint8_t div);


#ifdef __cplusplus
}
#endif

#endif /* _clock_h_ */
