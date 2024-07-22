#include <sys/times.h>
#include "board.h"
#include "cli_simple.h"

static stdout_ops_t stdout_ops_serial = {
    .init = serial_init,
    .available = serial_available,
    .read = serial_read,
    .write = serial_write
};

#if (USE_TIMER_SYSTICK == 1)
#else
static volatile uint32_t ticms;
void SysTick_Handler(void){
    ticms++;
}

void delay_ms(uint32_t ms){
    volatile uint32_t end = ticms + ms;
    while (ticms < end){ }
}

uint32_t ElapsedTicks(uint32_t start_ticks){
	int32_t delta = GetTick() - start_ticks;
    return (delta < 0) ? -delta : delta;
}

inline uint32_t GetTick(void)
{
    return ticms;
}
#endif

clock_t clock(void){
    return (clock_t)GetTick();
}

void board_init(void)
{	
	SystemInit();
	system_core_clock_update(); 

	SysTick_Config((SystemCoreClock / 1000) - 1); // config 1000us

    redirect_stdout(&stdout_ops_serial);
   	
    LED1_PIN_INIT;
}

void SW_Reset(void){
    NVIC_SystemReset();
}

void __debugbreak(void){
	 asm volatile
    (
        "bkpt #01 \n"
    );
}
