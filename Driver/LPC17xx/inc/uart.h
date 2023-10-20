#ifndef _uart_h_
#define _uart_h_

#include <stdint.h>

typedef enum uartx{
    UART_BUS0 = 0,
    UART_BUS1,
    UART_BUS2,
    UART_BUS3
}uart_e;

typedef struct serialbus {
    void *ctrl;             // CMSIS compliant controller
    uint8_t  bus;   		// bus number 0,1...
    uint32_t speed;
    union{
        struct {
            uint8_t parity;
            uint8_t stopbit;
            uint8_t datalength;
        };
        uint32_t cfg;
    };
    void (*cb)(void);     // TODO: implement callback for Read
}serialbus_t;

void UART_Init(serialbus_t *huart);
void UART_PutChar(serialbus_t *huart, char c);
void UART_Puts(serialbus_t *huart, const char *str);
char UART_GetChar(serialbus_t *huart);
uint8_t UART_GetCharNonBlocking(serialbus_t *huart, char *c);
uint8_t UART_Kbhit(serialbus_t *huart);

uint16_t UART_Write(serialbus_t *huart, uint8_t *data, uint16_t len);
uint16_t UART_Read(serialbus_t *huart, uint8_t *data, uint16_t len);
void UART_Attach(serialbus_t *huart, void (*fptr)(void));
void UART_IRQHandler(void *ptr);
#endif /* _usart_h_ */
