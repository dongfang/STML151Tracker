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

// Get the RTC time in our own data format.
void RTC_getTime(Time_t* time);

void RTC_scheduleDailyEvent();
void RTC_setWakeup(uint32_t periodSeconds);
void RTC_setWakeupNoNonsense(uint32_t periodSeconds) ;

void RTC_setCalibration(int16_t correction_PP10M);

void RTC_debugRTCTime();

// Returns how many seconds were waited.
int RTC_waitTillModuloMinutes(uint8_t modulo, uint8_t seconds);

int secondsOfDay(Time_t* time);

int timeDiffSeconds(Time_t* from, Time_t* to);

// As above but considers one past-midnight event
int timeAfter_seconds(Time_t* from, Time_t* to);
int timeDiffModulo24(Time_t* from, Time_t* to);

#endif /* RTC_H_ */
