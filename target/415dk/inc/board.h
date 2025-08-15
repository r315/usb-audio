#ifndef _board_h_
#define _board_h_


#ifdef __cplusplus
extern "C" {
#endif

#include <stddef.h>
#include <stdint.h>
#include "at32f415.h"
#include "at32f415_crm.h"

#define SET_BIT(REG, BIT)       ((REG) |= (BIT))
#define CLEAR_BIT(REG, BIT)     ((REG) &= ~(BIT))

#ifndef BOARD_415DK
#define BOARD_415DK
#endif

#define LED1_PIN_INIT \
        CRM->apb2en_bit.gpioaen = 1; \
        GPIOA->cfghr_bit.iomc8 = 2; \
        GPIOA->cfghr_bit.iofc8 = 0;

#define LED1_OFF        GPIOA->scr = (1 << 8)
#define LED1_ON         GPIOA->clr = (1 << 8)
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

#define I2S1_WS_PIN                      GPIO_PINS_4
#define I2S1_CK_PIN                      GPIO_PINS_5
#define I2S1_SD_PIN                      GPIO_PINS_7
#define I2S1_GPIO                        GPIOA
#define I2S1_GPIO_CRM_CLK                CRM_GPIOA_PERIPH_CLOCK
#define I2S1_DT_ADDRESS                  (&(SPI1->dt))
#define I2S1_MCK_PIN                     GPIO_PINS_0
#define I2S1_MCK_GPIO                    GPIOB

#define I2S2_WS_PIN                      GPIO_PINS_12
#define I2S2_CK_PIN                      GPIO_PINS_13
#define I2S2_SD_PIN                      GPIO_PINS_15
#define I2S2_GPIO                        GPIOB
#define I2S2_GPIO_CRM_CLK                CRM_GPIOB_PERIPH_CLOCK
#define I2S2_DT_ADDRESS                  (&(SPI2->dt))

#define HICK_TRIM                        50 // This is very critical

typedef struct{
    uint32_t freq;
    uint8_t bitw;
    uint8_t mode;
    uint16_t dma_buf_tx_size;
    uint16_t *dma_buf_tx;
    uint16_t dma_buf_rx_size;
    uint16_t *dma_buf_rx;
}i2s_config_t;

void board_init(void);
void delay_ms(uint32_t ms);
uint32_t ElapsedTicks(uint32_t start_ticks);
uint32_t GetTick(void);
void SW_Reset(void);
void __debugbreak(void);

void BOARD_LCD_Init(void);

uint32_t I2C_Master_Write(uint8_t, const uint8_t*, uint32_t);
uint32_t I2C_Master_Read(uint8_t, uint8_t*, uint32_t);
uint32_t I2C_Master_Scan(uint8_t device);

void serial_init(void);
int serial_available(void);
int serial_read(char *data, int len);
int serial_write(const char *buf, int len);

void disconnect_usb(void);
void connect_usb(void);

void bus_i2s_reset(void);
int bus_i2s_init(i2s_config_t *cfg);
void bus_i2s_mclk(uint32_t freq, uint8_t type, uint32_t enable);

#ifdef __cplusplus
}
#endif

#endif