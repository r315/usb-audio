#include <stdint.h>
#include <stdio.h>
#include "uart.h"
#include "uart_lpc17xx.h"
#include "LPC17xx.h"

struct FdrPair{
	unsigned short fr;
	unsigned char FdrVal; //(MulVal<<4) | MulVal	
};

//Fractional Divider setting look-up table
static const struct FdrPair _frdivTab[]={
//FR, DivAddVal|MulVal<<4
{1000,0|(1<<4)},
{1067,1|(15<<4)},
{1071,1|(14<<4)},
{1077,1|(13<<4)},
{1083,1|(12<<4)},
{1091,1|(11<<4)},
{1100,1|(10<<4)},
{1111,1|(9<<4)},
{1125,1|(8<<4)},
{1133,2|(15<<4)},
{1143,1|(7<<4)},
{1154,2|(13<<4)},
{1167,1|(6<<4)},
{1182,2|(11<<4)},
{1200,1|(5<<4)},
{1214,3|(14<<4)},
{1222,2|(9<<4)},
{1231,3|(13<<4)},
{1250,1|(4<<4)},
{1267,4|(15<<4)},
{1273,3|(11<<4)},
{1286,2|(7<<4)},
{1300,3|(10<<4)},
{1308,4|(13<<4)},
{1333,1|(3<<4)},
{1357,5|(14<<4)},
{1364,4|(11<<4)},
{1375,3|(8<<4)},
{1385,5|(13<<4)},
{1400,2|(5<<4)},
{1417,5|(12<<4)},
{1429,3|(7<<4)},
{1444,4|(9<<4)},
{1455,5|(11<<4)},
{1462,6|(13<<4)},
{1467,7|(15<<4)},
{1500,1|(2<<4)},
{1533,8|(15<<4)},
{1538,7|(13<<4)},
{1545,6|(11<<4)},
{1556,5|(9<<4)},
{1571,4|(7<<4)},
{1583,7|(12<<4)},
{1600,3|(5<<4)},
{1615,8|(13<<4)},
{1625,5|(8<<4)},
{1636,7|(11<<4)},
{1643,9|(14<<4)},
{1667,2|(3<<4)},
{1692,9|(13<<4)},
{1700,7|(10<<4)},
{1714,5|(7<<4)},
{1727,8|(11<<4)},
{1733,11|(15<<4)},
{1750,3|(4<<4)},
{1769,10|(13<<4)},
{1778,7|(9<<4)},
{1786,11|(14<<4)},
{1800,4|(5<<4)},
{1818,9|(11<<4)},
{1833,5|(6<<4)},
{1846,11|(13<<4)},
{1857,6|(7<<4)},
{1867,13|(15<<4)},
{1875,7|(8<<4)},
{1889,8|(9<<4)},
{1900,9|(10<<4)},
{1909,10|(11<<4)},
{1917,11|(12<<4)},
{1923,12|(13<<4)},
{1929,13|(14<<4)},
{1933,14|(15<<4)}
};

//TODO: Fix bug, add end condition for pointer on cycle do while
unsigned char frdivLookup(struct FdrPair *frdiv, unsigned int baudrate, uint64_t pclk){
unsigned int DLest,FRest;
struct FdrPair *pfrdvtab = (struct FdrPair *)_frdivTab;

	do{		
		DLest = (pclk * 1000)/ (16 * baudrate * (pfrdvtab->fr));
		FRest = (pclk * 1000)/ (16 * baudrate * DLest);
		pfrdvtab += 1;
	}while(FRest < 1000 || FRest > 1900);
	
	frdiv->fr = pfrdvtab->fr;
	frdiv->FdrVal = pfrdvtab->FdrVal;
return DLest & 255;
}

void UART_Init(serialbus_t *serialbus){
	LPC_UART_TypeDef *puart = NULL;
	//IRQn_Type irq;
	struct FdrPair frdiv;	
	unsigned char DLest;

    switch(serialbus->bus){
        case UART_BUS0: 
            puart = (LPC_UART_TypeDef *)LPC_UART0;
            // Turn on power to UART0	        
			LPC_SC->PCONP |= (1 << 3);
            // Turn on UART0 peripheral clock
			LPC_SC->PCLKSEL0 &= ~(3 << 6);  // CCLK/4			
			//irq = UART0_IRQn;

	        // P0.2 = TXD0, P0.3 = RXD0, Alternative function impose direction
	        LPC_PINCON->PINSEL0 &= ~0xf0;
	        LPC_PINCON->PINSEL0 |= (0x05 << 4);
            break;

        case UART_BUS1: 
			puart = (LPC_UART_TypeDef *)LPC_UART1;
			LPC_SC->PCONP |= (1 << 4);
			LPC_SC->PCLKSEL0 &= ~(3 << 8);  // CCLK/4
			//irq = UART1_IRQn;

			// P2.0 = TXD1, P2.1 = RXD1
	        LPC_PINCON->PINSEL4 &= ~(0x0f << 0);
	        LPC_PINCON->PINSEL4 |= (0x0A << 0);
			break;

        case UART_BUS2:
		#if 0  /* PINS are not available on BB */
			puart = (LPC_UART_TypeDef *)LPC_UART2;
			PCONP_UART2_ENABLE();
			CLOCK_SetPCLK(PCLK_UART2, PCLK_4);
			irq = UART2_IRQn;
			//P0.10 = TXD2, P0.11 = RXD2
	        LPC_PINCON->PINSEL0 &= ~(0x0f << 20);
	        LPC_PINCON->PINSEL0 |= (0x05 << 20);
			break;
		#else
			return;
		#endif

        case UART_BUS3: 
			puart = (LPC_UART_TypeDef *)LPC_UART3;
			LPC_SC->PCONP |= (1 << 25);
			LPC_SC->PCLKSEL1 &= ~(3 << 18);  // CCLK/4
			//irq = UART3_IRQn;

			// P0.0 = TXD3, P0.1 = RXD3
	        //LPC_PINCON->PINSEL0 &= ~(0x0f << 20);
	        //LPC_PINCON->PINSEL0 |= (0x0A << 20);

			// P0.25 = TXD3, P0.26 = RXD3
	        //LPC_PINCON->PINSEL1 &= ~(0x0f << 18);
	        LPC_PINCON->PINSEL1 |= (0x0F << 18);
			break;

        default: return;
    }

    puart->LCR = UART_LCR_DLAB | UART_LCR_WL8;
    DLest = frdivLookup(&frdiv, serialbus->speed, (SystemCoreClock >> 2));
    puart->DLM = DLest >> 8;
    puart->DLL = DLest & 0xFF;
	puart->FDR = frdiv.FdrVal;

    puart->LCR = UART_LCR_WL8;		// 8 bits, no Parity, 1 Stop bit
    puart->FCR = 0; 				// Disable FIFO

	//NVIC_EnableIRQ(irq);
    serialbus->ctrl = puart;

	//puart->IER |= (UART_IER_RBR | UART_IER_THRE);
}

char UART_GetChar(serialbus_t *huart){
	return 0;
}

void UART_PutChar(serialbus_t *huart, char c){
	LPC_UART_TypeDef *uart = (LPC_UART_TypeDef*)huart->ctrl;
	*((uint8_t*)&uart->THR) = (uint8_t)c;
	while(!(uart->LSR & UART_LSR_THRE));
}

void UART_Puts(serialbus_t *huart, const char *str){

}

uint8_t UART_GetCharNonBlocking(serialbus_t *huart, char *c){
	return 0;
}

uint8_t UART_Kbhit(serialbus_t *huart){
	return 0;
}

void UART_Attach(serialbus_t *huart, void(*fptr)(void)){
	
}

uint16_t UART_Write(serialbus_t *huart, uint8_t *data, uint16_t len){
/*	LPC_UART_TypeDef *uart = (LPC_UART_TypeDef*)huart->ctrl;
	uint16_t count = len;
	while(--count){
		if(fifo_put(&huart->txfifo, (uint8_t)*data++) == 0){
			break;
		}
	}
	fifo_get(&huart->txfifo, (uint8_t*)&uart->THR);
	return len - count; */
	return 0;
}

uint16_t UART_Read(serialbus_t *huart, uint8_t *data, uint16_t len){
	return 0;
}

void UART_IRQHandler(void *ptr){
	serialbus_t *serialbus;
	LPC_UART_TypeDef *uart;
	uint32_t iir, lsr, dummy __attribute__ ((unused));

	if(ptr == NULL){
		return;
	}

	serialbus = (serialbus_t*)ptr;
	uart = (LPC_UART_TypeDef*)serialbus->ctrl;
	iir = uart->IIR;	// Read clears interrupt identification 

	if((iir & UART_IIR_STATUS) == 0){
		// Check source
		switch((iir >> 1) & 7){
	
			case UART_IIR_THRE:
				//fifo_get(&serialbus->txfifo, (uint8_t*)&uart->THR);
				break;

			case UART_IIR_RDA:
				//fifo_put(&serialbus->rxfifo, uart->RBR);
				break;

			case UART_IIR_RLS:
				lsr = uart->LSR;
				if(lsr & (UART_LSR_OE|UART_LSR_PE|UART_LSR_FE|UART_LSR_RXFE|UART_LSR_BI) ){
					// errors have occurred
					dummy = uart->RBR;
				}else if(lsr & UART_LSR_RDR){
					// Data received
					//fifo_put(&serialbus->rxfifo, uart->RBR);
				}
				break;

			case UART_IIR_CTI:
				break;

			default:
				break;
		}
	}
}


