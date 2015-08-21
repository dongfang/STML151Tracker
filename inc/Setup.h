/*
 * Setup.h
 *
 *  Created on: Jun 27, 2015
 *      Author: dongfang
 */

#ifndef SRC_SETUP_H_
#define SRC_SETUP_H_

#include "Types.h"
#include "IndividualBoard.h"

// An AX.25 address.
typedef struct {
	const char* callsign;
	uint8_t ssid;
} AX25_Address_t;

//#define MYCALL "KM4FSW"
#define MYCALL "HB9FDK"
#define MY_APRS_SSID 11

// AX.25 addresses; MY_ADDRESS.callsign is used for all modes.
extern const AX25_Address_t MY_ADDRESS;
extern const AX25_Address_t APRS_APSTM1_DEST;
extern const AX25_Address_t APRS_DEST;
extern const AX25_Address_t APRS_DIGI1;
extern const AX25_Address_t APRS_DIGI2;

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

// 10000 ft
#define LOWALT_THRESHOLD 3048

// VHF each 2 min, WSPR each 10, HF APRS each 30
#define LOWALT_SCHEDULE_TIME 'a',(2*60),1,5,15

// VHF each 5 min, WSPR each 15, HF APRS each 30
#define DAY_SCHEDULE_TIME 'd',(5*60),1,3,12

// Wakeup each 10, VHF each 20, WSPR each 40, HF APRS each 1h
#define NIGHT_SCHEDULE_TIME 'n',(10*60),2,2,3

// Wakeup each 20, VHF each 60, WSPR each 120, HF APRS almost never
#define LOWBATT_SCHEDULE_TIME 'b',(20*60),3,2,255

// Wakeup each 20, VHF each 240, WSPR each 480, HF APRS almost never
#define CRISIS_SCHEDULE_TIME 'c',(20*60),12,2,255

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
// in the evening, and it will happen quickly.
#define NORMAL_SCHEDULE_MIN_TEMPERATURE -12

#define STORAGE_INTERVAL_S 3600

#define MAX_HSE_CALIBRATION_TIME 30000
#define MAX_GPS_TIMELOCK_TIME 300000


#endif /* SRC_SETUP_H_ */
