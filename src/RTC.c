/*
 * RTC.c
 *
 *  Created on: May 29, 2015
 *      Author: dongfang
 */
#include "stm32l1xx_conf.h"
#include "RTC.h"
#include "Globals.h"
#include "Systick.h"
#include <diag/trace.h>

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

	// trace_printf("RTC OK. \r\n");

	return true; // okay
}

void RTC_debugRTCTime() {
	RTC_TimeTypeDef rtcTime;
	RTC_GetTime(RTC_Format_BIN, &rtcTime);
	trace_printf("RTC is now %02u:%02u:%02u\n", rtcTime.RTC_Hours,
			rtcTime.RTC_Minutes, rtcTime.RTC_Seconds);
}

int RTC_timeDiff_s(RTC_TimeTypeDef* t1, RTC_TimeTypeDef* t2) {
	int result = t1->RTC_Seconds - t2->RTC_Seconds;
	result += (t1->RTC_Minutes - t2->RTC_Minutes) * 60;
	result += (t1->RTC_Hours - t2->RTC_Hours) * 3600;
	return result;
}

int RTC_waitTillModuloMinutes(uint8_t modulo, uint8_t seconds) {
	uint8_t minutes;
	uint8_t _seconds;
	RTC_WaitForSynchro();
	RTC_TimeTypeDef rtcFirstTime;
	RTC_GetTime(RTC_Format_BIN, &rtcFirstTime);
	RTC_TimeTypeDef rtcTime;

	do {
		RTC_GetTime(RTC_Format_BIN, &rtcTime);
		minutes = rtcTime.RTC_Minutes;
		_seconds = rtcTime.RTC_Seconds;
	} while ((seconds != _seconds || (minutes % modulo) != 0)
			&& timer_sleep(100));

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
	RTC_WaitForSynchro();
	result = RTC_SetDate(RTC_HourFormat_24, &rtcDate);
	result = RTC_SetTime(RTC_HourFormat_24, &rtcTime);
	RTC_WaitForSynchro();

	RTC_debugRTCTime();
	//PWR_BackupAccessCmd(DISABLE);

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

void RTC_init_RTC_ALARMA_NVIC() {
	/* Configure one bit for preemption priority */
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);

	NVIC_InitTypeDef NVIC_InitStructure;

	/* Enable the RTC Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = RTC_Alarm_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	EXTI_InitTypeDef exti; /// [12] According to manual RTC Alarm interrupt works
	EXTI_ClearITPendingBit(EXTI_Line17); ///      in pair with external interrupt controller line 17.
	exti.EXTI_Line = EXTI_Line17;
	exti.EXTI_Mode = EXTI_Mode_Interrupt; ///     Mode is interrupt, not event.
	exti.EXTI_Trigger = EXTI_Trigger_Rising; ///     Interrupt sensitive to rising edge.
	exti.EXTI_LineCmd = ENABLE;                 ///     Make line available.
	EXTI_Init(&exti);
}

void RTC_init_RTC_WKUP_NVIC() {
	NVIC_InitTypeDef NVIC_InitStructure;

	/* Configure one bit for preemption priority */
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);

	/* Enable the RTC Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = RTC_WKUP_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	EXTI_InitTypeDef exti; /// [12] According to manual RTC Alarm interrupt works
	EXTI_ClearITPendingBit(EXTI_Line20); ///      in pair with external interrupt controller line 17.
	exti.EXTI_Line = EXTI_Line20;
	exti.EXTI_Mode = EXTI_Mode_Interrupt; ///     Mode is interrupt, not event.
	exti.EXTI_Trigger = EXTI_Trigger_Rising; ///     Interrupt sensitive to rising edge.
	exti.EXTI_LineCmd = ENABLE;                 ///     Make line available.
	EXTI_Init(&exti);
}

/*
 void RTCAlarm_IRQHandler(void) {
 if (RTC_GetITStatus(RTC_IT_ALRA) != RESET) {
 / * Clear EXTI line17 pending bit * /
 EXTI_ClearITPendingBit(EXTI_Line17);

 / * Check if the Wake-Up flag is set * /
 if (PWR_GetFlagStatus(PWR_FLAG_WU) != RESET) {
 / * Clear Wake Up flag * /
 PWR_ClearFlag(PWR_FLAG_WU);
 }

 / * Wait until last write operation on RTC registers has finished * /
 RTC_WaitForSynchro();

 / * Clear RTC Alarm interrupt pending bit * /
 RTC_ClearITPendingBit(RTC_IT_ALRA);
 }
 }
 */

/**
 * @brief  This function handles RTC global interrupt request.
 * @param  None
 * @retval None
 */
void RTC_Alarm_IRQHandler(void) {
	if (RTC_GetITStatus(RTC_IT_ALRA) != RESET) {

		EXTI_ClearFlag(EXTI_Line17);

		trace_printf("*******************\n");
		trace_printf("*** RTC Alarm A ***\n");
		trace_printf("*******************\n");
		/* Clear the RTC Second interrupt */
		RTC_ClearITPendingBit(RTC_IT_ALRA);

		PWR_ClearFlag(PWR_FLAG_WU);

		if (interruptAlarm) {
			trace_printf("RTCAl");
		}
	}
	if (RTC_GetITStatus(RTC_IT_WUT) != RESET) {

		EXTI_ClearFlag(EXTI_Line20);

		trace_printf("*******************\n");
		trace_printf("*** RTC Wakeup  ***\n");
		trace_printf("*******************\n");
		RTC_ClearITPendingBit(RTC_IT_WUT);

		PWR_ClearFlag(PWR_FLAG_WU);
		if (interruptAlarm) {
			trace_printf("WUT1\n");
		}
	}
}

void RTC_WKUP_IRQHandler(void) {
	/*
	 if (RTC_GetITStatus(RTC_IT_WUT) != RESET) {
	 EXTI_ClearFlag(EXTI_Line20);
	 RTC_ClearITPendingBit(RTC_IT_WUT);
	 trace_printf("WKUP IRQ II @ ");
	 RTC_debugRTCTime();
	 if (interruptAlarm) {
	 trace_printf("WUT2\n");
	 }
	 }*/
	if (RTC_GetITStatus(RTC_IT_WUT) != RESET) {
		EXTI_ClearITPendingBit(EXTI_Line20); //OK
		PWR_RTCAccessCmd(ENABLE);

		// These 2 seem to do the same!!
		RTC_ClearITPendingBit(RTC_IT_WUT);
		// RTC_ClearFlag(RTC_FLAG_WUTF);
	}
}

void EXTI_IRQHandler(void) {
	trace_printf("Well there is a stray exti\n");
	if (EXTI_GetITStatus(EXTI_Line17) != RESET) {
		EXTI_ClearITPendingBit(EXTI_Line17);
		if (interruptAlarm) {
			trace_printf("EXTI\n");
		}
	}
}

// Schedule an alarm event in the nearest possible future where the minutes mod slotMinutes = 0
void RTC_scheduleASAPAlarmInSlot(uint16_t slotMinutes) {
	PWR_RTCAccessCmd(ENABLE);
	RTC_WaitForSynchro();
	RTC_init_RTC_ALARMA_NVIC();

	RTC_TimeTypeDef alarmTime;
	RTC_AlarmTypeDef rtcAlarm;
	RTC_GetTime(RTC_HourFormat_24, &alarmTime);
//	alarmTime.RTC_H12
	alarmTime.RTC_Minutes += slotMinutes + 0;
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
void RTC_setWakeup(uint32_t periodSeconds) {
	//trace_printf("Setting the WUT setup\n");
	RTC_init_RTC_WKUP_NVIC();

	RTC_WakeUpCmd(DISABLE);
	RTC_WakeUpClockConfig(RTC_WakeUpClock_CK_SPRE_16bits);
	RTC_SetWakeUpCounter(periodSeconds - 1);
	RTC_WakeUpCmd(ENABLE);

	RTC_ITConfig(RTC_IT_WUT, ENABLE);

	//trace_printf("Survived the WUT setup\n");
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
	RTC_CoarseCalibConfig(sign, ivalue);
	/*
	 trace_printf("Using sign %s and ivalue %d. Success: %s\n",
	 sign == RTC_CalibSign_Positive ? "pos" : "neg", ivalue,
	 result == SUCCESS ? "ok" : "fail");
	 */
}

