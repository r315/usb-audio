#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include "at32f415.h"
#include "at32f415_crm.h"

#define UART_BUFFER_SIZE  128

static uint8_t tx_buf[UART_BUFFER_SIZE];
static uint8_t rx_buf[UART_BUFFER_SIZE];
static volatile uint16_t tx_rd, tx_wr, rx_rd, rx_wr;

/**
 * API
 * */
void serial_init(void){
    crm_clocks_freq_type clocks;
    crm_clocks_freq_get(&clocks);

    CRM->apb2en_bit.usart1en = 1;
    CRM->apb2rst_bit.usart1rst = 1;
    CRM->apb2rst_bit.usart1rst = 0;

    USART1->ctrl1 = 0x20AC;
    USART1->baudr = clocks.apb2_freq / 115200;

    CRM->apb2en_bit.gpioaen = 1;
    GPIOA->cfghr = (GPIOA->cfghr & 0xFFFFF00F) | (4 << 8) | (10 << 4);       // PA9 TX, PA10 RX

    rx_rd = rx_wr = tx_rd = tx_wr = 0;

    NVIC_SetPriority(USART1_IRQn, 5);
    NVIC_EnableIRQ(USART1_IRQn);

    setvbuf(stdout, NULL, _IONBF, 0);
}

uint32_t serial_available(void){
	return (rx_wr > rx_rd) ? rx_wr - rx_rd : rx_rd - rx_wr;
}

uint32_t serial_write(const uint8_t *buf, uint32_t len){
	usart_type *uart = USART1;
    const uint8_t *end = buf + len;

	while(buf < end){
        uint16_t size = (tx_wr > tx_rd) ? tx_wr - tx_rd : tx_rd - tx_wr;
		if(UART_BUFFER_SIZE - size > 0){
			tx_buf[tx_wr++] = *buf++;
            if(tx_wr == UART_BUFFER_SIZE){
                tx_wr = 0;
            }
		}else{
			uart->ctrl1_bit.tdbeien = 1;
			while(tx_wr == tx_rd);
		}
	}	
	
	uart->ctrl1_bit.tdbeien = 1;
    return len;
}

uint32_t serial_read(uint8_t *data, uint32_t len){
    uint32_t count = len;

	while(count--){
        while(serial_available() == 0);
        *data++ = rx_buf[rx_rd++];
        if(rx_rd == UART_BUFFER_SIZE){
            rx_rd = 0;
        }
    }

    return len;
}

void USART1_IRQHandler(void){
    uint32_t isrflags = USART1->sts;
    uint32_t errorflags = isrflags & 0x000F;
    uint32_t ctrl = USART1->ctrl1;

    if (errorflags){
        USART1->sts = errorflags & 0x3F0;
        return;
    }

    if ((isrflags & (1 << 5)) && (ctrl & (1 << 5))){
        if(serial_available() < UART_BUFFER_SIZE){
            rx_buf[rx_wr++] = USART1->dt;
            if(rx_wr == UART_BUFFER_SIZE){
                rx_wr = 0;
            }
        }else{
            errorflags = USART1->dt;
        }
    }

    if ((isrflags & (1 << 7)) && (ctrl & (1 << 7))){
        /* TX empty, send more data or finish transmission */
        if(tx_wr == tx_rd){
            USART1->sts_bit.tdc = 0;            // Clear Transmit Data Complete bit since no write to DT ocurred
            USART1->ctrl1_bit.tdbeien = 0;      // Disable Transmit Buffer Empty interrupt
        }else{
            USART1->dt = tx_buf[tx_rd++];
            if(tx_rd == UART_BUFFER_SIZE){
                tx_rd = 0; 
            }
        }
    }
}

