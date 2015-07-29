/*
 * RTC.h
 *
 *  Created on: May 30, 2015
 *      Author: dongfang
 */

#ifndef RTC_H_
#define RTC_H_

#include "Types.h"

boolean RTC_init() ;

boolean RTC_setRTC(Date_t* date, Time_t* time);

void RTC_getDHM(uint8_t* date, uint8_t* hours24, uint8_t* minutes);

void RTC_scheduleASAPAlarmInSlot(uint16_t minutes);

void RTC_setWakeup(uint32_t periodSeconds);

void RTC_setCalibration(int16_t correction_PP10M);

int RTC_timeDiff_s(RTC_TimeTypeDef* t1, RTC_TimeTypeDef* t2);

void RTC_debugRTCTime();

int RTC_waitTillModuloMinutes(uint8_t modulo, uint8_t seconds);

#endif /* RTC_H_ */
