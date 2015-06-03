/*
 * RTC.h
 *
 *  Created on: May 30, 2015
 *      Author: dongfang
 */

#ifndef RTC_H_
#define RTC_H_
#include "GPS.h"
uint8_t RTC_init() ;
void setRTC(Date_t* date, Time_t* time);
void scheduleASAPAlarmInSlot(int minutes);
#endif /* RTC_H_ */
