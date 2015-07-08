/*
 * Setup.h
 *
 *  Created on: Jun 27, 2015
 *      Author: dongfang
 */

#ifndef SRC_SETUP_H_
#define SRC_SETUP_H_

#include "Types.h"

#define GPS_TIMEPULSE_WASTED_PULSES 0
#define RTC_WUT_WASTED_PULSES 0

// Development only! Whether we want to defeat the Power.c checks so that we can reset...
#define DEFEAT_VOLTAGE_CHECKS false

#define REUSE_CALIBRATION true

#define POSITION_MAX_TIME_S 360

#define REQUIRE_HIGH_PRECISION_POSITIONS true
#define REQUIRE_HIGH_PRECISION_NUM_SATS 4
#define REQUIRE_HIGH_PRECISION_FIXLEVEL 1
#define REQUIRE_HIGH_PRECISION_ALTITUDE true
#define REQUIRE_HIGH_PRECISION_MAX_TIME_S 360

#define LOWALT_THRESHOLD 7500

// VHF each 2 min, WSPR each 10, HF APRS each 30
#define LOWALT_SCHEDULE_TIME (2*60),1,5,15

// VHF each 5 min, WSPR each 10, HF APRS each 30
#define DAY_SCHEDULE_TIME (5*60),1,2,12

// Wakeup each 10, VHF each 20, WSPR each 40, HF APRS each 1h
#define NIGHT_SCHEDULE_TIME (10*60),2,2,3

// Wakeup each 20, VHF each 60, WSPR each 120, HF APRS almost never
#define LOWBATT_SCHEDULE_TIME (20*60),3,2,255

// Wakeup each 20, VHF each 120, WSPR each 120, HF APRS almost never
#define CRISIS_SCHEDULE_TIME (20*60),6,2,255

#define HF_30m_HARDWARE_OUTPUT 3
#define DIRECT_2m_HARDWARE_OUTPUT 2

// When battery and temperature are both at least this good,
// ignore old records of CPU fail while running equipment.
#define BATTERY_FAILSAFE_ALWAYS_SAFE_VOLTAGE 3.8
#define BATTERY_FAILSAFE_ALWAYS_SAFE_TEMPERATURE -3

// When CPU failed runing some equipment, don't try again until
// battery voltage is this much better than at the crashing run.
#define BATTERY_FAILSAFE_VOLTAGE_THRESHOLD 0.1

// When getting colder than this, schedule slower. It will happen
// in the evening, and it will happen rapidly.
#define NORMAL_SCHEDULE_MIN_TEMPERATURE -15

#endif /* SRC_SETUP_H_ */
