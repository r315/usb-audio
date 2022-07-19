/**
* @file		i2s.h
* @brief	Contains the i2s API header.
*     		
* @version	1.0
* @date		12 Dec. 2021
* @author	Hugo Reis
**********************************************************************/

#ifndef _I2S_H_
#define _I2S_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define I2S_TX_EN           (1 << 0)
#define I2S_RX_EN           (1 << 1)
#define I2S_TX_MASTER       (1 << 2)
#define I2S_RX_MASTER       (1 << 3)
#define I2S_TX_EN_MASTER    (I2S_TX_EN | I2S_TX_MASTER)
#define I2S_RX_EN_MASTER    (I2S_RX_EN | I2S_RX_MASTER)
#define I2S_MCLK_OUT        (1 << 4)

typedef enum {
    I2S_BUS0 = 0,
    I2S_BUS1,
    I2S_BUS2,
    I2S_BUS3
}i2sbus_e;

typedef void (*i2sCallback)(uint32_t *, uint32_t);

typedef struct {
    void *regs;
    void *dma;
    uint32_t sample_rate;
    uint8_t data_size;
    uint8_t channels;
    uint8_t mode;
    uint8_t mute;
    i2sbus_e bus;
    volatile uint32_t *txbuffer;
    volatile uint32_t *rxbuffer;
    volatile uint32_t wridx;
    volatile uint32_t rdidx;
    uint32_t tx_buf_len;
    uint32_t rx_buf_len;
    i2sCallback txcp;
    i2sCallback rxcp;
}i2sbus_t;


void I2S_Init(i2sbus_t *i2s);
void I2S_Config(i2sbus_t *i2s);
void I2S_Stop(i2sbus_t *i2s);
void I2S_Start(i2sbus_t *i2s);

#ifdef __cplusplus
}
#endif

#endif
