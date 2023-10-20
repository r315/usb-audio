/*
    Basic startup code
*/	
#define WEAK __attribute__ ((weak))
#define VTOR *((unsigned int*) 0xE000ED08)
extern int main(void);

//*****************************************************************************
// Forward declaration of the default fault handlers.
//*****************************************************************************
/* System exception vector handler */
void WEAK 		NMI_Handler(void);               /* NMI Handler */
//void WEAK 		HardFault_Handler(void);         /* Hard Fault Handler */
void WEAK 		MemManage_Handler(void);         /* MPU Fault Handler */
void WEAK 		BusFault_Handler(void);          /* Bus Fault Handler */
void WEAK 		UsageFault_Handler(void);        /* Usage Fault Handler */
void WEAK 		SVC_Handler(void);               /* SVCall Handler */
void WEAK 		DebugMon_Handler(void);          /* Debug Monitor Handler */
void WEAK 		PendSV_Handler(void);            /* PendSV Handler */
void WEAK 		SysTick_Handler(void);           /* SysTick Handler */

/* External interrupt vector handler */
void WEAK      	WDT_IRQHandler(void);            /* Watchdog Timer */
void WEAK      	TIMER0_IRQHandler(void);         /* Timer0 */
void WEAK      	TIMER1_IRQHandler(void);         /* Timer1 */
void WEAK      	TIMER2_IRQHandler(void);         /* Timer2 */
void WEAK      	TIMER3_IRQHandler(void);         /* Timer3 */
void WEAK      	UART0_IRQHandler(void);          /* UART0 */
void WEAK      	UART1_IRQHandler(void);          /* UART1 */
void WEAK      	UART2_IRQHandler(void);          /* UART2 */
void WEAK      	UART3_IRQHandler(void);          /* UART3 */
void WEAK      	PWM1_IRQHandler(void);           /* PWM1 */
void WEAK      	I2C0_IRQHandler(void);           /* I2C0 */
void WEAK      	I2C1_IRQHandler(void);           /* I2C1 */
void WEAK      	I2C2_IRQHandler(void);           /* I2C2 */
void WEAK      	SPI_IRQHandler(void);            /* SPI */
void WEAK      	SSP0_IRQHandler(void);           /* SSP0 */
void WEAK      	SSP1_IRQHandler(void);           /* SSP1 */
void WEAK      	PLL0_IRQHandler(void);           /* PLL0 (Main PLL) */
void WEAK      	RTC_IRQHandler(void);            /* Real Time Clock */
void WEAK      	EINT0_IRQHandler(void);          /* External Interrupt 0 */
void WEAK      	EINT1_IRQHandler(void);          /* External Interrupt 1 */
void WEAK      	EINT2_IRQHandler(void);          /* External Interrupt 2 */
void WEAK      	EINT3_IRQHandler(void);          /* External Interrupt 3 */
void WEAK      	ADC_IRQHandler(void);            /* A/D Converter */
void WEAK      	BOD_IRQHandler(void);            /* Brown Out Detect */
void WEAK      	USB_IRQHandler(void);            /* USB */
void WEAK      	CAN_IRQHandler(void);            /* CAN */
void WEAK      	DMA_IRQHandler(void);            /* GP DMA */
void WEAK      	I2S_IRQHandler(void);            /* I2S */
void WEAK      	ENET_IRQHandler(void);           /* Ethernet */
void WEAK      	RIT_IRQHandler(void);            /* Repetitive Interrupt Timer */
void WEAK      	MCPWM_IRQHandler(void);          /* Motor Control PWM */
void WEAK      	QEI_IRQHandler(void);            /* Quadrature Encoder Interface */
void WEAK      	PLL1_IRQHandler(void);           /* PLL1 (USB PLL) */

//*****************************************************************************
// Provide weak aliases for each Exception handler to the Default_Handler.
// As they are weak aliases, any function with the same name will override
// this definition.
//*****************************************************************************
//#pragma weak HardFault_Handler = Default_Handler          /* MPU Fault Handler */
#pragma weak MemManage_Handler = Default_Handler          /* MPU Fault Handler */
#pragma weak BusFault_Handler = Default_Handler           /* Bus Fault Handler */
#pragma weak UsageFault_Handler = Default_Handler         /* Usage Fault Handler */
#pragma weak SVC_Handler = Default_Handler                /* SVCall Handler */
#pragma weak DebugMon_Handler = Default_Handler           /* Debug Monitor Handler */
#pragma weak PendSV_Handler = Default_Handler             /* PendSV Handler */
#pragma weak SysTick_Handler = Default_Handler            /* SysTick Handler */

/* External interrupt vector handler */
#pragma weak WDT_IRQHandler = Default_Handler            /* Watchdog Timer */
#pragma weak TIMER0_IRQHandler = Default_Handler         /* Timer0 */
#pragma weak TIMER1_IRQHandler = Default_Handler         /* Timer1 */
#pragma weak TIMER2_IRQHandler = Default_Handler         /* Timer2 */
#pragma weak TIMER3_IRQHandler = Default_Handler         /* Timer3 */
#pragma weak UART0_IRQHandler = Default_Handler          /* UART0 */
#pragma weak UART1_IRQHandler = Default_Handler          /* UART1 */
#pragma weak UART2_IRQHandler = Default_Handler          /* UART2 */
#pragma weak UART3_IRQHandler = Default_Handler          /* UART3 */
#pragma weak PWM1_IRQHandler = Default_Handler           /* PWM1 */
#pragma weak I2C0_IRQHandler = Default_Handler           /* I2C0 */
#pragma weak I2C1_IRQHandler = Default_Handler           /* I2C1 */
#pragma weak I2C2_IRQHandler = Default_Handler           /* I2C2 */
#pragma weak SPI_IRQHandler = Default_Handler            /* SPI */
#pragma weak SSP0_IRQHandler = Default_Handler           /* SSP0 */
#pragma weak SSP1_IRQHandler = Default_Handler           /* SSP1 */
#pragma weak PLL0_IRQHandler = Default_Handler           /* PLL0 (Main PLL) */
#pragma weak RTC_IRQHandler = Default_Handler            /* Real Time Clock */
#pragma weak EINT0_IRQHandler = Default_Handler          /* External Interrupt 0 */
#pragma weak EINT1_IRQHandler = Default_Handler          /* External Interrupt 1 */
#pragma weak EINT2_IRQHandler = Default_Handler          /* External Interrupt 2 */
#pragma weak EINT3_IRQHandler = Default_Handler          /* External Interrupt 3 */
#pragma weak ADC_IRQHandler = Default_Handler            /* A/D Converter */
#pragma weak BOD_IRQHandler = Default_Handler            /* Brown Out Detect */
#pragma weak USB_IRQHandler = Default_Handler            /* USB */
#pragma weak CAN_IRQHandler = Default_Handler            /* CAN */
#pragma weak DMA_IRQHandler = Default_Handler            /* GP DMA */
#pragma weak I2S_IRQHandler = Default_Handler            /* I2S */
#pragma weak ENET_IRQHandler = Default_Handler           /* Ethernet */
#pragma weak RIT_IRQHandler = Default_Handler            /* Repetitive Interrupt Timer */
#pragma weak MCPWM_IRQHandler = Default_Handler          /* Motor Control PWM */
#pragma weak QEI_IRQHandler = Default_Handler            /* Quadrature Encoder Interface */
#pragma weak PLL1_IRQHandler = Default_Handler           /* PLL1 (USB PLL) */


/* ----------------------- Linker script Exported constants ---------------------------------- */
extern unsigned int _etext;		/* end address for the .text section.*/
extern unsigned int _sidata;	/* start address of .data section on flash*/
extern unsigned int _sdata;     /* start address for the .data section on ram */
extern unsigned int _edata;	    /* end address for the .data section. */
extern unsigned int _sbss;		/* start address for the .bss section. */
extern unsigned int _ebss;	    /* end address for the .bss section. */
extern unsigned int _stack;		/* init value for the stack pointer. */
extern unsigned int _siramcode;
extern unsigned int _sramcode;
extern unsigned int _eramcode;
extern unsigned int _estartup;

/* ---------------------------- System clock ------------------------------------------ */
typedef unsigned int uint32_t;
//*****************************************************************************
// Reset_Handler is the first code executed after reset state and is responsible
// for initialize data, clear uninitialized data and call main function
//*****************************************************************************
void Reset_Handler(void)
{ 
    unsigned int *pulDest;
    unsigned int *pulSrc; 

    /* clear uninitialized data (bss segment) */
    for(pulDest = &_sbss; pulDest < &_ebss; pulDest++){
        *pulDest = 0;
    }

    // initialize data (data segment)
    if (&_sidata != &_sdata) {	// only if needed
        pulSrc = &_sidata;
    
        for(pulDest = &_sdata; pulDest < &_edata; ) {
            *(pulDest++) = *(pulSrc++);
        }
    }

    // copy ram functions with __attribute__ ((section(".ram_code"))
    if (&_siramcode != &_sramcode) {	// only if needed
        pulSrc = &_siramcode;
        for(pulDest = &_sramcode; pulDest < &_eramcode; ) {
            *(pulDest++) = *(pulSrc++);
        }
    }
    
    VTOR = (unsigned int)&_estartup; // if running on ram remap vector table

    main();
    
    while(1){ }
}
//*****************************************************************************
//
// This is the code that gets called when the processor receives an unexpected
// interrupt.  This simply enters an infinite loop, preserving the system state
// for examination by a debugger.
//
//*****************************************************************************
void Default_Handler(void) {	
    while (1) {
    }
}

/*
__attribute__ ((naked)) 
void StackTrace(unsigned int *stack){
    unsigned int r0 = stack[0];
    unsigned int r1 = stack[1];
    unsigned int r2 = stack[2];
    unsigned int r3 = stack[3];
    unsigned int r12 = stack[4];
    unsigned int lr = stack[5];
    unsigned int pc = stack[6];
    unsigned int psr = stack[7];
}
*/

WEAK void HardFault_Handler(void){   
    asm volatile ("TST LR, #4");
    asm volatile ("ITE EQ");
    asm volatile ("MRSEQ R0, MSP");
    asm volatile ("MRSNE R0, PSP");
    //asm volatile ("B StackTrace");
    asm volatile ("B .");
}
//*****************************************************************************
// vectors table
//*****************************************************************************
__attribute__ ((section(".startup"))) void *vecTable[] = {
    &_stack,                   /* The initial stack pointer */		
    Reset_Handler,             /* Reset Handler */   
    NMI_Handler,               /* NMI Handler */
    HardFault_Handler,         /* Hard Fault Handler */
    MemManage_Handler,         /* MPU Fault Handler */
    BusFault_Handler,          /* Bus Fault Handler */
    UsageFault_Handler,        /* Usage Fault Handler */
    0,                         /* Checksum word */
    0,                         /* Reserved */
    0,                         /* Reserved */
    0,                         /* Reserved */
    SVC_Handler,               /* SVCall Handler */
    DebugMon_Handler,          /* Debug Monitor Handler */
    0,                         /* Reserved */
    PendSV_Handler,            /* PendSV Handler */
    SysTick_Handler,           /* SysTick Handler */

    // External Interrupts
    WDT_IRQHandler,            /* Watchdog Timer */
    TIMER0_IRQHandler,         /* Timer0 */
    TIMER1_IRQHandler,         /* Timer1 */
    TIMER2_IRQHandler,         /* Timer2 */
    TIMER3_IRQHandler,         /* Timer3 */
    UART0_IRQHandler,          /* UART0 */
    UART1_IRQHandler,          /* UART1 */
    UART2_IRQHandler,          /* UART2 */
    UART3_IRQHandler,          /* UART3 */
    PWM1_IRQHandler,           /* PWM1 */
    I2C0_IRQHandler,           /* I2C0 */
    I2C1_IRQHandler,           /* I2C1 */
    I2C2_IRQHandler,           /* I2C2 */
    SPI_IRQHandler,            /* SPI */
    SSP0_IRQHandler,           /* SSP0 */
    SSP1_IRQHandler,           /* SSP1 */
    PLL0_IRQHandler,           /* PLL0 (Main PLL) */
    RTC_IRQHandler,            /* Real Time Clock */
    EINT0_IRQHandler,          /* External Interrupt 0 */
    EINT1_IRQHandler,          /* External Interrupt 1 */
    EINT2_IRQHandler,          /* External Interrupt 2 */
    EINT3_IRQHandler,          /* External Interrupt 3 */
    ADC_IRQHandler,            /* A/D Converter */
    BOD_IRQHandler,            /* Brown Out Detect */
    USB_IRQHandler,            /* USB */
    CAN_IRQHandler,            /* CAN */
    DMA_IRQHandler,            /* GP DMA */
    I2S_IRQHandler,            /* I2S */
    ENET_IRQHandler,           /* Ethernet */
    RIT_IRQHandler,            /* Repetitive Interrupt Timer */
    MCPWM_IRQHandler,          /* Motor Control PWM */
    QEI_IRQHandler,            /* Quadrature Encoder Interface */
    PLL1_IRQHandler,           /* PLL1 (USB PLL) */    
};