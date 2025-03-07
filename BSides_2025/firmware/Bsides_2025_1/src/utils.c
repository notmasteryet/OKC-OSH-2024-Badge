#include "utils.h"

#include "ch32fun.h"

#define SYSTICK_ONE_MILLISECOND ((uint32_t)FUNCONF_SYSTEM_CORE_CLOCK / 1000)

volatile uint32_t systick_millis;

uint32_t millis() { return systick_millis; }

// Copy from https://github.com/cnlohr/ch32fun/examples/systick_irq/systick_irq.c

void systickInit(void)
{
	// Reset any pre-existing configuration
	SysTick->CTLR = 0x0000;
	
	// Set the compare register to trigger once per millisecond
	SysTick->CMP = SYSTICK_ONE_MILLISECOND - 1;

	// Reset the Count Register, and the global millis counter to 0
	SysTick->CNT = 0x00000000;
	systick_millis = 0x00000000;
	
	// Set the SysTick Configuration
	// NOTE: By not setting SYSTICK_CTLR_STRE, we maintain compatibility with
	// busywait delay funtions used by ch32v003_fun.
	SysTick->CTLR |= SYSTICK_CTLR_STE   |  // Enable Counter
	                 SYSTICK_CTLR_STIE  |  // Enable Interrupts
	                 SYSTICK_CTLR_STCLK ;  // Set Clock Source to HCLK/1
	
	// Enable the SysTick IRQ
	NVIC_EnableIRQ(SysTicK_IRQn);
}

void SysTick_Handler(void) __attribute__((interrupt()));
void SysTick_Handler(void)
{
	SysTick->CMP += SYSTICK_ONE_MILLISECOND;
	SysTick->SR = 0x00000000;
	systick_millis++;
}
