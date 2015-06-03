/*
 * RTC.c
 *
 *  Created on: May 29, 2015
 *      Author: dongfang
 */
#include "stm32l1xx_conf.h"
#include "RTC.h"
#include <diag/trace.h>

uint8_t RTC_init() {
	int timeout = 1000000;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);
	PWR_RTCAccessCmd(ENABLE);

	/* Reset RTC Backup Domain */
	RCC_RTCResetCmd(ENABLE);
	RCC_RTCResetCmd(DISABLE);

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
		return 0; // fail.
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
		return 0; // fail.
	}

	trace_printf("RTC OK. \r\n");

	return 1; // okay
}

void setRTC(Date_t* date, Time_t* time) {
	RTC_DateTypeDef rtcDate;

	rtcDate.RTC_WeekDay = 1; // we don't give a damn.
	rtcDate.RTC_Year = date->year100;
	rtcDate.RTC_Month = date->month;
	rtcDate.RTC_Date = date->date;

	RTC_TimeTypeDef rtcTime;

	rtcTime.RTC_Seconds = time->seconds;
	rtcTime.RTC_Minutes = time->minutes;
	rtcTime.RTC_Hours = time->hours;

	RTC_SetDate(RTC_HourFormat_24, &rtcDate);
	RTC_SetTime(RTC_HourFormat_24, &rtcTime);

	trace_printf("Set RTC to %02u:%02u:%02u\n", rtcTime.RTC_Hours,
			rtcTime.RTC_Minutes, rtcTime.RTC_Seconds);

	// how to shift the clock
	// RTC_SynchroShiftConfig(RTC_ShiftAdd1S_Reset, 0);
}

void init_RTC_NVIC() {
	EXTI_InitTypeDef exti; /// [12] According to manual RTC Alarm interrupt works
	EXTI_ClearITPendingBit(EXTI_Line17); ///      in pair with external interrupt controller line 17.
	exti.EXTI_Line = EXTI_Line17;
	exti.EXTI_Mode = EXTI_Mode_Interrupt; ///     Mode is interrupt, not event.
	exti.EXTI_Trigger = EXTI_Trigger_Rising; ///     Interrupt sensetive to rising edge.
	exti.EXTI_LineCmd = ENABLE;                 ///     Make line available.
	EXTI_Init(&exti);

	NVIC_InitTypeDef NVIC_InitStructure;

	/* Configure one bit for preemption priority */
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);

	/* Enable the RTC Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = RTC_Alarm_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

/**
 * @brief  This function handles RTC global interrupt request.
 * @param  None
 * @retval None
 */
void RTC_IRQHandler(void) {
	if (RTC_GetITStatus(RTC_IT_ALRA) != RESET) {
		/* Clear the RTC Second interrupt */
		RTC_ClearITPendingBit(RTC_IT_ALRA);

		/* Wait until last write operation on RTC registers has finished */
//    RTC_WaitForLastTask();
		/* Reset RTC Counter when Time is 23:59:59 */
		//  if (RTC_GetCounter() == 0x00015180)
		{
			//  RTC_SetCounter(0x0);
			/* Wait until last write operation on RTC registers has finished */
			//  RTC_WaitForLastTask();
		}
		trace_printf("RTC interrupt A\n");
	}
}

void EXTI_IRQHandler(void) {
	if (EXTI_GetITStatus(EXTI_Line17) != RESET) {
		EXTI_ClearITPendingBit(EXTI_Line17);
		trace_printf("EXT interrupt\n");
	}
}

// Schedule an alarm event in the nearest possible future where the minutes mod slotMinutes = 0
void scheduleASAPAlarmInSlot(int slotMinutes) {
	RTC_TimeTypeDef rtcTime;
	RTC_AlarmTypeDef rtcAlarm;
	RTC_GetTime(RTC_HourFormat_24, &rtcTime);
	rtcTime.RTC_Minutes += slotMinutes + 1;
	rtcTime.RTC_Minutes -= rtcTime.RTC_Minutes % slotMinutes;
	if (rtcTime.RTC_Minutes >= 60) {
		rtcTime.RTC_Minutes -= 60;
		rtcTime.RTC_Hours++;
		if (rtcTime.RTC_Hours >= 24)
			rtcTime.RTC_Hours = 0;
	}
	rtcTime.RTC_Seconds = 0;
	rtcAlarm.RTC_AlarmTime = rtcTime;
	rtcAlarm.RTC_AlarmMask =
	RTC_AlarmMask_DateWeekDay | RTC_AlarmMask_Hours; // fire on minutes count??

	RTC_AlarmCmd(RTC_Alarm_A, DISABLE);
	RTC_SetAlarm(RTC_HourFormat_24, RTC_Alarm_A, &rtcAlarm);
	RTC_AlarmCmd(RTC_Alarm_A, ENABLE);
	RTC_ClearFlag(RTC_FLAG_ALRAF);
	RTC_ITConfig(RTC_IT_ALRA, ENABLE);
	init_RTC_NVIC();
	trace_printf("Scheduled an alarm at %02u:%02u:%02u\n", rtcTime.RTC_Hours,
			rtcTime.RTC_Minutes, rtcTime.RTC_Seconds);
}
