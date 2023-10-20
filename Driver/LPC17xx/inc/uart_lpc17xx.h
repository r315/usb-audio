#ifndef _uart_lpc17xx_h_
#define _uart_lpc17xxh_


#ifdef __cplusplus
extern "C" {
#endif
 
#include <stdint.h>

/* UART0/2/3 */

// LSR
#define UART_LSR_RDR        (1 << 0)
#define UART_LSR_OE         (1 << 1)
#define UART_LSR_PE         (1 << 2)
#define UART_LSR_FE         (1 << 3)
#define UART_LSR_BI         (1 << 4)
#define UART_LSR_THRE       (1 << 5)
#define UART_LSR_TEMT       (1 << 6)
#define UART_LSR_RXFE       (1 << 7)

/* UART1 */

// IER
#define UART_IER_RBR        (1 << 0)    // Receive data available
#define UART_IER_THRE       (1 << 1)    // Transmitter Holding Register Empty
#define UART_IER_RX         (1 << 2)    // RX Line
#define UART_IER_MS         (1 << 3)    // Modem Status
#define UART_IER_CTS        (1 << 7)    //
#define UART_IER_ABEO       (1 << 8)    // Auto-baud 
#define UART_IER_ABTO       (1 << 9)    // Auto-baud timeout

// IIR
#define UART_IIR_STATUS     (1 << 0)
#define UART_IIR_THRE       1
#define UART_IIR_RLS        3
#define UART_IIR_RDA        2
#define UART_IIR_CTI        6

// FCR
#define UART_FCR_EN         (1 << 0)    // 
#define UART_FCR_RX_RST     (1 << 1)    // 
#define UART_FCR_TX_RST     (1 << 2)    // 
#define UART_FCR_DMA        (1 << 3)    //
#define UART_FCR_RX_LVL0    (0 << 6)    // 
#define UART_FCR_RX_LVL1    (1 << 6)    // 
#define UART_FCR_RX_LVL2    (2 << 6)    // 
#define UART_FCR_RX_LVL3    (3 << 6)    // 

// LCR
#define UART_LCR_WL5        (0 << 0)
#define UART_LCR_WL6        (1 << 0)
#define UART_LCR_WL7        (2 << 0)
#define UART_LCR_WL8        (3 << 0)    // Word length 8bit 
#define UART_LCR_SB         (1 << 2)    // Stop bit
#define UART_LCR_PE         (1 << 3)    // Parity enable
#define UART_LCR_PS_ODD     (0 << 4)    // Odd parity
#define UART_LCR_PS_EVEN    (1 << 4)    // Even parity
#define UART_LCR_PS_SET     (2 << 4)    // Forced "1"
#define UART_LCR_PS_CLR     (3 << 4)    // Forced "0"
#define UART_LCR_BRK        (1 << 6)    // Break Control
#define UART_LCR_DLAB       (1 << 7)    // Diviso latched enable

// TER
#define UART_TER_TXEN       (1 << 7)

#ifdef __cplusplus
}
#endif

#endif
