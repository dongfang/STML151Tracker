#ifndef SYSTICK_H_
#define SYSTICK_H_

// ----------------------------------------------------------------------------

#define TIMER_FREQUENCY_HZ (1000u)

#ifdef __cplusplus
extern "C" {
#endif

#include "Types.h"

extern volatile uint32_t timer_delayCount;
extern volatile uint32_t systemTimeMillis;

void systick_start();
void systick_end();

boolean timer_sleep (uint32_t ticks);

void timer_mark();
boolean timer_elapsed(int32_t millis);

uint32_t timer_timeSinceMark();

#ifdef __cplusplus
}
#endif
// ----------------------------------------------------------------------------

#endif // SYSTICK_H_
