/*
 * RTC.c
 *
 *  Created on: May 29, 2015
 *      Author: dongfang
 */
#include "stm32l1xx.h"
#include "RTC.h"
#include "Globals.h"
#include "Systick.h"
#include <diag/trace.h>

int secondsOfDay(Time_t* time) {
	int result = time->seconds;
	result += time->minutes * 60;
	result += time->hours * 3600;
	return result;
}

int timeDiffSeconds(Time_t* from, Time_t* to) {
	return secondsOfDay(to) - secondsOfDay(from);
}

// From must be before or equal to to.
int timeAfter_seconds(Time_t* from, Time_t* to) {
	int diff = timeDiffSeconds(from, to);
	if (diff < 0)
		diff += 24 * 60 * 60;
	return diff;
}

// The modulo-24h difference
int timeDiffModulo24(Time_t* from, Time_t* to) {
	int diff = timeDiffSeconds(from, to);
	if (diff <= -12* 60 * 60)
		diff += 24 * 60 * 60;
	else if (diff > 12* 60 * 60)
		diff -= 24 * 60 * 60;
	return diff;
}

boolean RTC_init() {
	int timeout = 1000000;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);
	PWR_RTCAccessCmd(ENABLE);

	/* Reset RTC Backup Domain */
	// RCC_RTCResetCmd(ENABLE);
	// RCC_RTCResetCmd(DISABLE);
	//Enable LSE oscillator (32.768 Hz)
	RCC_LSEConfig(RCC_LSE_ON);

	/* Wait until LSE is ready */
	/* seems we HAVE to do that this early. Just starting use the clock and letting it get
	 * ready when-ever is not good enough.
	 */
	while (timeout-- > 0 && RCC_GetFlagStatus(RCC_FLAG_LSERDY) == RESET)
		;

	if (RCC_GetFlagStatus(RCC_FLAG_LSERDY) == RESET) {
		trace_printf("ERROR : LSE clock not ready\n");
		if (!(RCC->CSR & RCC_CSR_LSEON)) {
			trace_printf("  (doesn't even seem enabled)\n");
		}
		return false; // fail.
	}

	RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);
	RCC_RTCCLKCmd(ENABLE);

	RTC_InitTypeDef R;

	/* Waits until the RTC Time and Date registers are synchronized with RTC APB clock.*/
	RTC_WaitForSynchro();

	/* Set hour format to 24hrs and the prescaler for 32768Hz xtal (default) */
	RTC_StructInit(&R);

	/* initialize the RTC */
	if (RTC_Init(&R) == ERROR) {
		trace_printf("RTC init failed \r\n");
		return false; // fail.
	}

	PWR_RTCAccessCmd(DISABLE);
	return true; // okay
}

// Get the RTC time in our own data format.
void RTC_getTime(Time_t* time) {
	RTC_TimeTypeDef rtcTime;
	RTC_GetTime(RTC_Format_BIN, &rtcTime);
	time->hours = rtcTime.RTC_Hours;
	time->minutes = rtcTime.RTC_Minutes;
	time->seconds = rtcTime.RTC_Seconds;
}

void RTC_debugRTCTime() {
	RTC_DateTypeDef rtcDate;
	RTC_TimeTypeDef rtcTime;
	RTC_GetDate(RTC_Format_BIN, &rtcDate);
	RTC_GetTime(RTC_Format_BIN, &rtcTime);
	trace_printf("RTC is now %02u:%02u:%02u @%02u \n", rtcTime.RTC_Hours,
			rtcTime.RTC_Minutes, rtcTime.RTC_Seconds, rtcDate.RTC_Date);
}

int RTC_timeDiff_s(RTC_TimeTypeDef* t1, RTC_TimeTypeDef* t2) {
	int result = t1->RTC_Seconds - t2->RTC_Seconds;
	result += (t1->RTC_Minutes - t2->RTC_Minutes) * 60;
	result += (t1->RTC_Hours - t2->RTC_Hours) * 3600;
	return result;
}

int RTC_waitTillModuloMinutes(uint8_t modulo, uint8_t seconds) {
	uint8_t _minutes;
	uint8_t _seconds;
	RTC_WaitForSynchro();
	RTC_TimeTypeDef rtcFirstTime;
	RTC_GetTime(RTC_Format_BIN, &rtcFirstTime);
	RTC_TimeTypeDef rtcTime;

	do {
		RTC_GetTime(RTC_Format_BIN, &rtcTime);
		_minutes = rtcTime.RTC_Minutes;
		_seconds = rtcTime.RTC_Seconds;
	} while ((seconds != _seconds || (_minutes % modulo) != 0) && timer_sleep(250));

	return RTC_timeDiff_s(&rtcTime, &rtcFirstTime);
}

boolean RTC_setRTC(Date_t* date, Time_t* time) {
	RTC_DateTypeDef rtcDate;
	PWR_RTCAccessCmd(ENABLE);

	rtcDate.RTC_WeekDay = 1; // we don't give a damn.
	rtcDate.RTC_Year = date->year100;
	rtcDate.RTC_Month = date->month;
	rtcDate.RTC_Date = date->date;

	RTC_TimeTypeDef rtcTime;

	rtcTime.RTC_Seconds = time->seconds;
	rtcTime.RTC_Minutes = time->minutes;
	rtcTime.RTC_Hours = time->hours;

	uint8_t result;
	// RTC_WaitForSynchro();
	result = RTC_SetDate(RTC_Format_BIN, &rtcDate);
	result = RTC_SetTime(RTC_Format_BIN, &rtcTime);
	RTC_WaitForSynchro();

	RTC_debugRTCTime();
	PWR_RTCAccessCmd(DISABLE);

	return result != ERROR;
}

void RTC_getDHM(uint8_t* date, uint8_t* hours24, uint8_t* minutes) {
	PWR_RTCAccessCmd(ENABLE);
	RTC_WaitForSynchro();

	RTC_DateTypeDef rtcDate;
	RTC_GetDate(RTC_Format_BIN, &rtcDate);
	*date = rtcDate.RTC_Date;

	RTC_TimeTypeDef rtcTime;
	RTC_GetTime(RTC_Format_BIN, &rtcTime);
	*hours24 = rtcTime.RTC_Hours;
	*minutes = rtcTime.RTC_Minutes;
}

void RTC_init_RTC_ALARMA_EXTI() {
	/* Enable the RTC Interrupt
	 * Nah we dont need to, this is EXTI events, not NVIC interrupts.
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = RTC_Alarm_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	*/

	EXTI_InitTypeDef exti; /// [12] According to manual RTC Alarm interrupt works
	EXTI_ClearITPendingBit(EXTI_Line17); ///      in pair with external interrupt controller line 17.
	exti.EXTI_Line = EXTI_Line17;
	exti.EXTI_Mode = EXTI_Mode_Event; ///     Mode is interrupt, not event.
	exti.EXTI_Trigger = EXTI_Trigger_Rising; ///     Interrupt sensitive to rising edge.
	exti.EXTI_LineCmd = ENABLE;                 ///     Make line available.
	EXTI_Init(&exti);
}

void RTC_init_RTC_WKUP_EXTI() {
	/* Enable the RTC Interrupt
	 * Nah we dont need to, this is EXTI events, not NVIC interrupts.
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = RTC_WKUP_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
    */
	EXTI_InitTypeDef exti; /// [12] According to manual RTC Alarm interrupt works
	EXTI_ClearITPendingBit(EXTI_Line20); ///      in pair with external interrupt controller line 17.
	exti.EXTI_Line = EXTI_Line20;
	exti.EXTI_Mode = EXTI_Mode_Event; 			///
	exti.EXTI_Trigger = EXTI_Trigger_Rising; 	///     Interrupt sensitive to rising edge.
	exti.EXTI_LineCmd = ENABLE;                 ///     Make line available.
	EXTI_Init(&exti);
}

// Schedule an alarm event in the nearest possible future where the minutes mod slotMinutes = 0
void RTC_scheduleDailyEvent() {
	PWR_RTCAccessCmd(ENABLE);
	RTC_init_RTC_ALARMA_EXTI();

	RTC_AlarmTypeDef rtcAlarm;

	rtcAlarm.RTC_AlarmTime.RTC_Hours = 8;
	rtcAlarm.RTC_AlarmTime.RTC_Minutes = 0;
	rtcAlarm.RTC_AlarmTime.RTC_Seconds = 0;

	rtcAlarm.RTC_AlarmMask = RTC_AlarmMask_DateWeekDay; //|RTC_AlarmMask_Minutes;
	rtcAlarm.RTC_AlarmDateWeekDay = 1; // just some junk date that will be ignored.
	rtcAlarm.RTC_AlarmDateWeekDaySel = RTC_AlarmDateWeekDaySel_Date;

	RTC_ClearITPendingBit(RTC_IT_ALRA);
	EXTI_ClearITPendingBit(EXTI_Line17);
	RTC_ITConfig(RTC_IT_ALRA, ENABLE);

	uint8_t error = RTC_AlarmCmd(RTC_Alarm_A, DISABLE);
	RTC_SetAlarm(RTC_Format_BIN, RTC_Alarm_A, &rtcAlarm);
	error &= RTC_AlarmCmd(RTC_Alarm_A, ENABLE);

	PWR_RTCAccessCmd(DISABLE);

	trace_printf("Scheduled an alarm at %02u:%02u:%02u with error %d\n",
			rtcAlarm.RTC_AlarmTime.RTC_Hours,
			rtcAlarm.RTC_AlarmTime.RTC_Minutes,
			rtcAlarm.RTC_AlarmTime.RTC_Seconds,
			error);
}

/*
 * To enable the RTC Wakeup interrupt, the following sequence is required:
 1. Configure and enable the EXTI Line 20 in interrupt mode and select the rising edge sensitivity.
 2. Configure and enable the RTC_WKUP IRQ channel in the NVIC.
 3. Configure the RTC to generate the RTC wakeup timer event.
 */
void RTC_setWakeup(uint32_t periodSeconds) {
	PWR_RTCAccessCmd(ENABLE);
	RTC_init_RTC_WKUP_EXTI();

	RTC_ITConfig(RTC_IT_WUT, DISABLE);

	RTC_WakeUpCmd(DISABLE); // This amounts to unlocking RTC_WPR and clearing WUTE
	RTC_WakeUpClockConfig(RTC_WakeUpClock_CK_SPRE_16bits);
	RTC_SetWakeUpCounter(periodSeconds - 1);
	RTC_WakeUpCmd(ENABLE);

	RTC_ClearITPendingBit(RTC_IT_WUT);
	EXTI_ClearITPendingBit(EXTI_Line20);
	RTC_ITConfig(RTC_IT_WUT, ENABLE);

	PWR_RTCAccessCmd(DISABLE);
}

#define INITMODE_TIMEOUT         ((uint32_t) 0x00002000)
void RTC_setWakeupNoNonsense(uint32_t periodSeconds) {
	RTC->WPR = 0xCA;
	RTC->WPR = 0x53;

	/* Disable the Wakeup Timer */
	RTC->CR &= ~RTC_CR_WUTE;
	uint32_t wutwfstatus;
	uint32_t wutcounter = 0;
	/* Wait till RTC WUTWF flag is set and if Time out is reached exit */
	do {
		wutwfstatus = RTC->ISR & RTC_ISR_WUTWF;
		wutcounter++;
	} while ((wutcounter != INITMODE_TIMEOUT) && (wutwfstatus == 0x00));

	RTC->WUTR = periodSeconds;

	/* Clear the Wakeup Timer clock source bits in CR register */
	RTC->CR &= (uint32_t) ~RTC_CR_WUCKSEL;

	/* Configure the clock source */
	RTC->CR |= 4;

	// Enable WUT again
	RTC->CR |= RTC_CR_WUTE;

	// Write protect again.
	RTC->WPR = 0xFF;
}

/*
 * Coarse-calibrate RTC.
 */
// rfactor is <1 when we are too slow and >1 when too fast.
// = refcounts(1 GPS second) / refcounts(1 RTC second)
void RTC_setCalibration(int16_t correction_PP10M) {
	uint16_t ivalue;
	uint32_t sign;
	if (correction_PP10M >= 0) {
		/* We are too slow. Positive: 4ppm stepsize
		 *                This value should be between 0 and 126 when using positive sign
		 *                with a 4-ppm step.
		 */
		ivalue = correction_PP10M / 40;
		if (ivalue > 126)
			ivalue = 126;
		sign = RTC_CalibSign_Positive;
	} else if (correction_PP10M < 0) {
		/* We are too fast. Negative: 2ppm stepsize
		 * This value should be between 0 and 63 when using negative sign
		 * with a 2-ppm step.
		 */
		ivalue = -correction_PP10M / 20;
		if (ivalue > 63)
			ivalue = 63;
		sign = RTC_CalibSign_Negative;
	}
	RTC_CoarseCalibConfig(sign, ivalue);
}

