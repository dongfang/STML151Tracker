#ifndef SYSTICK_H_
#define SYSTICK_H_

// ----------------------------------------------------------------------------

#define TIMER_FREQUENCY_HZ (1000u)

extern volatile uint32_t timer_delayCount;
extern volatile uint32_t systemTimeMillis;

extern void timer_start (void);
extern void timer_sleep (uint32_t ticks);


extern void timer_mark();
extern uint8_t timer_elapsed(int32_t millis);

// ----------------------------------------------------------------------------

#endif // SYSTICK_H_
