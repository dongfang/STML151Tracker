/*
 * RTC.h
 *
 *  Created on: May 30, 2015
 *      Author: dongfang
 */

#ifndef RTC_H_
#define RTC_H_

#include "Types.h"

uint8_t RTC_init() ;

void setRTC(Date_t* date, Time_t* time);

void scheduleASAPAlarmInSlot(uint16_t minutes);

void setWakeup(uint16_t periodSeconds);

void RTC_setCalibration(double rfactor);

void debugRTCTime();

#endif /* RTC_H_ */
