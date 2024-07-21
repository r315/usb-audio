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

#define DBG_PIN_INIT    LED1_PIN_INIT
#define DBG_PIN_TOGGLE  LED1_TOGGLE

#define USER_BUTTON 1

#define I2C_TIMEOUT                      0xFFFFFFFF
#define I2Cx_SPEED                       100000
#define I2Cx_ADDRESS                     0x00
#define I2Cx_PORT                        I2C1
#define I2Cx_CLK                         CRM_I2C1_PERIPH_CLOCK
#define I2Cx_SCL_GPIO_PIN                GPIO_PINS_8
#define I2Cx_SCL_GPIO_PORT               GPIOB
#define I2Cx_SDA_GPIO_PIN                GPIO_PINS_9
#define I2Cx_SDA_GPIO_PORT               GPIOB


void board_init(void);
void delay_ms(uint32_t ms);
uint32_t ElapsedTicks(uint32_t start_ticks);
uint32_t GetTick(void);
void SW_Reset(void);
void __debugbreak(void);

void BOARD_LCD_Init(void);

uint32_t I2C_Master_Write(uint8_t, const uint8_t*, uint32_t);
uint32_t I2C_Master_Read(uint8_t, uint8_t*, uint32_t);

void serial_init(void);
uint32_t serial_available(void);
uint32_t serial_read(uint8_t *data, uint32_t len);
uint32_t serial_write(const uint8_t *buf, uint32_t len);

#ifdef __cplusplus
}
#endif

#endif