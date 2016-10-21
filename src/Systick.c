/*
 * systick.c
 *
 *  Created on: Mar 10, 2015
 *      Author: dongfang
 */

// ----------------------------------------------------------------------------
#include "Types.h"
#include "Globals.h"
#include "stm32l1xx.h"
#include <diag/trace.h>

volatile uint32_t timer_delayCount;
volatile uint32_t timerMark;
volatile uint32_t systemTimeMillis;

// ----------------------------------------------------------------------------
#include "systick.h"
//#include "cortexm/ExceptionHandlers.h"

extern uint32_t SystemCoreClock;

void systick_start() {
	// Use SysTick as reference for the delay loops.
	SysTick_Config(SystemCoreClock / TIMER_FREQUENCY_HZ);
}

void systick_end() {
	SysTick->CTRL = 0;
}

boolean timer_sleep(uint32_t ticks) {
	timer_delayCount = ticks;

	// Prevent normal IRQ handling from eating the interrupt that is supposed to wake us up (hm? Not correct?)
	// NVIC_DisableIRQ(SysTick_IRQn);
	// __DSB();
	// __ISB();
	// NVIC_ClearPendingIRQ(SysTick_IRQn);
	// Busy wait until the SysTick decrements the counter to zero.
	// HMMM! Should we not clear pending interrupts or something like that??
	NVIC_EnableIRQ(SysTick_IRQn);

	while (timer_delayCount != 0u) {
		PWR_EnterSleepMode(PWR_Regulator_ON, PWR_SLEEPEntry_WFI);
#ifdef TRACE
		// trace_printf("w");
#endif
	}

	return true;
}

void timer_mark() {
	timerMark = systemTimeMillis;
}

boolean timer_elapsed(int32_t millis) {
	// TODO: Does this work when overflowing?
	return systemTimeMillis >= timerMark + millis;
}

uint32_t timer_timeSinceMark() {
	// TODO: Does this work when overflowing?
	return systemTimeMillis - timerMark;
}


// ----- SysTick_Handler() ----------------------------------------------------

void SysTick_Handler(void) {
	systemTimeMillis++;
	//if (systemTimeMillis % 1000 == 0)
	//	trace_printf("t");
	// Decrement to zero the counter used by the delay routine.
	if (timer_delayCount != 0u) {
		--timer_delayCount;
	}
}
