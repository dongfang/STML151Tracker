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

void debugRTCTime() {
	RTC_TimeTypeDef rtcTime;
	RTC_GetTime(RTC_Format_BIN, &rtcTime);
	trace_printf("RTC is now %02u:%02u:%02u\n", rtcTime.RTC_Hours,
			rtcTime.RTC_Minutes, rtcTime.RTC_Seconds);
}

void RTC_waitTillModuloMinutes(uint8_t modulo, uint8_t seconds) {
	uint8_t minutes;
	uint8_t _seconds;
	do {
		RTC_TimeTypeDef rtcTime;
		RTC_GetTime(RTC_Format_BIN, &rtcTime);
		minutes = rtcTime.RTC_Minutes;
		_seconds = rtcTime.RTC_Seconds;
	} while (seconds != _seconds || (minutes % modulo) != 0);
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

	debugRTCTime();

	// how to shift the clock. Well might not be supported on this Category 1 bucket.
	// RTC_SynchroShiftConfig(RTC_ShiftAdd1S_Reset, 0);
}

void init_RTC_ALARMA_NVIC() {
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

void init_RTC_WKUP_NVIC() {
	EXTI_InitTypeDef exti; /// [12] According to manual RTC Alarm interrupt works
	EXTI_ClearITPendingBit(EXTI_Line20); ///      in pair with external interrupt controller line 17.
	exti.EXTI_Line = EXTI_Line20;
	exti.EXTI_Mode = EXTI_Mode_Interrupt; ///     Mode is interrupt, not event.
	exti.EXTI_Trigger = EXTI_Trigger_Rising; ///     Interrupt sensetive to rising edge.
	exti.EXTI_LineCmd = ENABLE;                 ///     Make line available.
	EXTI_Init(&exti);

	NVIC_InitTypeDef NVIC_InitStructure;

	/* Configure one bit for preemption priority */
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);

	/* Enable the RTC Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = RTC_WKUP_IRQn;
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
void RTC_Alarm_IRQHandler(void) {
	trace_printf("Well there is a rtc.\n");
	EXTI_ClearFlag(EXTI_Line17);

	if (RTC_GetITStatus(RTC_IT_ALRA) != RESET) {
		trace_printf("It was alarm A\n");
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
	}
	if (RTC_GetITStatus(RTC_IT_WUT) != RESET) {
		trace_printf("It was WUT\n");
		/* Clear the RTC Second interrupt */
		RTC_ClearITPendingBit(RTC_IT_WUT);
	}
	if (RTC_GetITStatus(RTC_IT_ALRB) != RESET) {
		trace_printf("It was Alarm B\n");
		/* Clear the RTC Second interrupt */
		RTC_ClearITPendingBit(RTC_IT_ALRB);
	}
	if (RTC_GetITStatus(RTC_IT_TAMP) != RESET) {
		trace_printf("It was tamper\n");
		/* Clear the RTC Second interrupt */
		RTC_ClearITPendingBit(RTC_IT_TAMP);
	}
	if (RTC_GetITStatus(RTC_IT_TAMP1) != RESET) {
		trace_printf("It was tamper1\n");
		/* Clear the RTC Second interrupt */
		RTC_ClearITPendingBit(RTC_IT_TAMP1);
	}
	if (RTC_GetITStatus(RTC_IT_TS) != RESET) {
		trace_printf("It was ts\n");
		/* Clear the RTC Second interrupt */
		RTC_ClearITPendingBit(RTC_IT_TS);
	}
}

void RTC_WKUP_IRQHandler(void) {
	EXTI_ClearFlag(EXTI_Line20);
	if (RTC_GetITStatus(RTC_IT_WUT) != RESET) {
		RTC_ClearITPendingBit(RTC_IT_WUT);
		trace_printf("WKUP IRQ II @ ");
		debugRTCTime();
	}
}

void EXTI_IRQHandler(void) {
	trace_printf("Well there is a stray exti\n");
	if (EXTI_GetITStatus(EXTI_Line17) != RESET) {
		EXTI_ClearITPendingBit(EXTI_Line17);
		trace_printf("EXT interrupt\n");
	}
}

// Schedule an alarm event in the nearest possible future where the minutes mod slotMinutes = 0
void scheduleASAPAlarmInSlot(uint16_t slotMinutes) {
	init_RTC_ALARMA_NVIC();

	RTC_TimeTypeDef alarmTime;
	RTC_AlarmTypeDef rtcAlarm;
	RTC_GetTime(RTC_HourFormat_24, &alarmTime);
//	alarmTime.RTC_H12
	alarmTime.RTC_Minutes += slotMinutes + 1;
	alarmTime.RTC_Minutes -= alarmTime.RTC_Minutes % slotMinutes;
	if (alarmTime.RTC_Minutes >= 60) {
		alarmTime.RTC_Minutes -= 60;
		alarmTime.RTC_Hours++;
		if (alarmTime.RTC_Hours >= 24)
			alarmTime.RTC_Hours -= 24;
	}

	alarmTime.RTC_Seconds = 0;
	rtcAlarm.RTC_AlarmTime = alarmTime;
	rtcAlarm.RTC_AlarmMask = RTC_AlarmMask_DateWeekDay; //|RTC_AlarmMask_Minutes;
	rtcAlarm.RTC_AlarmDateWeekDay = 1; // just some junk date that will be ignored.
	rtcAlarm.RTC_AlarmDateWeekDaySel = RTC_AlarmDateWeekDaySel_Date;

	//RTC_ClearITPendingBit(RTC_IT_ALRA);
	//RTC_ClearFlag(RTC_FLAG_ALRAF);
	RTC_ITConfig(RTC_IT_ALRA, ENABLE);

	// RTC_WriteProtectionCmd(DISABLE);
	uint8_t error = RTC_AlarmCmd(RTC_Alarm_A, DISABLE);
	RTC_SetAlarm(RTC_Format_BIN, RTC_Alarm_A, &rtcAlarm);
	error &= RTC_AlarmCmd(RTC_Alarm_A, ENABLE);

	trace_printf("Scheduled an alarm at %02u:%02u:%02u with error %d\n",
			alarmTime.RTC_Hours, alarmTime.RTC_Minutes, alarmTime.RTC_Seconds,
			error);
}

/*
 * To enable the RTC Wakeup interrupt, the following sequence is required:
 1. Configure and enable the EXTI Line 20 in interrupt mode and select the rising edge sensitivity.
 2. Configure and enable the RTC_WKUP IRQ channel in the NVIC.
 3. Configure the RTC to generate the RTC wakeup timer event.
 */
void setWakeup(uint16_t periodSeconds) {
	init_RTC_WKUP_NVIC();

	RTC_WakeUpCmd(DISABLE);
	RTC_WakeUpClockConfig(RTC_WakeUpClock_CK_SPRE_16bits);
	RTC_SetWakeUpCounter(periodSeconds - 1);
	RTC_WakeUpCmd(ENABLE);

	RTC_ITConfig(RTC_IT_WUT, ENABLE);

	trace_printf("Survived the WUT setup\n");
}

/*
 void setWakeupNoNonsens(int periodSeconds) {
 RTC->WPR = 0xCA;
 RTC->WPR = 0x53;

 / * Disable the Wakeup Timer * /
 RTC->CR &= (uint32_t) ~RTC_CR_WUTE;
 / * Wait till RTC WUTWF flag is set and if Time out is reached exit * /
 do {
 wutwfstatus = RTC->ISR & RTC_ISR_WUTWF;
 wutcounter++;
 } while ((wutcounter != INITMODE_TIMEOUT) && (wutwfstatus == 0x00));

 / * Clear the Wakeup Timer clock source bits in CR register * /
 RTC->CR &= (uint32_t) ~RTC_CR_WUCKSEL;

 / * Configure the clock source * /
 RTC->CR |= 4;

 RTC->WUTR = periodSeconds;

 RTC->WPR = 0xFF;
 }
 */

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
	ErrorStatus result = RTC_CoarseCalibConfig(sign, ivalue);
	/*
	trace_printf("Using sign %s and ivalue %d. Success: %s\n",
			sign == RTC_CalibSign_Positive ? "pos" : "neg", ivalue,
			result == SUCCESS ? "ok" : "fail");
	*/
}

