/*
 * systick.c
 *
 *  Created on: Mar 10, 2015
 *      Author: dongfang
 */

// ----------------------------------------------------------------------------
#include "Types.h"

volatile uint32_t timer_delayCount;
volatile uint32_t timerMark;
volatile uint32_t systemTimeMillis;

// ----------------------------------------------------------------------------
#include "systick.h"
//#include "cortexm/ExceptionHandlers.h"

extern uint32_t SystemCoreClock;

void timer_start(void) {
	// Use SysTick as reference for the delay loops.
	SysTick_Config(SystemCoreClock / TIMER_FREQUENCY_HZ);
}

void timer_sleep(uint32_t ticks) {
	timer_delayCount = ticks;

	// Busy wait until the SysTick decrements the counter to zero.
	while (timer_delayCount != 0u)
		;
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
	// Decrement to zero the counter used by the delay routine.
	if (timer_delayCount != 0u) {
		--timer_delayCount;
	}
}
