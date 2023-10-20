#ifndef _board_h_
#define _board_h_


#ifdef __cplusplus
extern "C" {
#endif

#include <stddef.h>
#include <stdint.h>
#include "at32f415.h"
//#include "at32f415_gpio.h"
#include "at32f415_crm.h"

#define SET_BIT(REG, BIT)       ((REG) |= (BIT))
#define CLEAR_BIT(REG, BIT)     ((REG) &= ~(BIT))

#ifndef BOARD_415DK
#define BOARD_415DK
#endif


#define LCD_IO_SET(port, pinmask) port->BSRE = pinmask
#define LCD_IO_RESET(port, pinmask) port->BRE = pinmask

#define LED1_PIN_INIT \
        CRM->apb2en_bit.gpioaen = 1; \
        GPIOA->cfghr |= (6 << 0)
        

#define LED1_OFF        GPIOA->SCR = (1 << 8)
#define LED1_ON         GPIOA->CLR = (1 << 8)
#define LED1_TOGGLE     GPIOA->odt = GPIOA->idt ^ (1 << 8)

#define DBG_PIN_INIT    //LED1_PIN_INIT
#define DBG_PIN_TOGGLE  //LED1_TOGGLE

//enum {false = 0, true, OFF = false, ON = true};


void board_init(void);
void delay_ms(uint32_t ms);
uint32_t ElapsedTicks(uint32_t start_ticks);
uint32_t GetTick(void);
void SW_Reset(void);
void __debugbreak(void);

void BOARD_LCD_Init(void);
#ifdef __cplusplus
}
#endif

#endif