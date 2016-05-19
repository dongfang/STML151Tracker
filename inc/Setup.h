/*
 * Setup.h
 *
 *  Created on: Jun 27, 2015
 *      Author: dongfang
 */

#ifndef SRC_SETUP_H_
#define SRC_SETUP_H_

#include <stdint.h>

// An AX.25 address.
typedef struct {
	const char* callsign;
	uint8_t ssid;
} AX25_Address_t;

#define MYCALL "HB9FDK"
#define MY_APRS_SSID 15

#define SI4463_DAYTIME_TX_POWER 100
#define SI4463_NIGHTTIME_TX_POWER 10

#define FLIGHT 1
#define GROUNDTEST 2
#define MODE FLIGHT

// If defined, VHF will remain keyed unmodulated for some seconds after tx to allow
// checking whether the frequency is right.
// #define TEST_VHF_FREQUENCY

// AX.25 addresses; MY_ADDRESS.callsign is used for all modes.
extern const AX25_Address_t MY_ADDRESS;
extern const AX25_Address_t APRS_APSTM1_DEST;
extern const AX25_Address_t APRS_DEST;
extern const AX25_Address_t APRS_DIGI1;
extern const AX25_Address_t APRS_DIGI2;

// we are without a regulator right now
#define ABSOLUTE_MIN_VBATT 3.0
#define LOW_VBATT 3.5
// When battery and temperature are both at least this good,
// ignore old records of CPU fail while running equipment.
#define BATTERY_FAILSAFE_ALWAYS_SAFE_VOLTAGE 3.8
#define BATTERY_FAILSAFE_ALWAYS_SAFE_TEMPERATURE -20
#define BATTERY_FAILSAFE_ALWAYS_SAFE_SOLAR_VOLTAGE 0.20

// Development only! Whether we want to defeat the Power.c checks so that we can reset...
#define DEFEAT_VOLTAGE_CHECKS false

#define RTC_WAKEUP_PERIOD_S 120

// Experimental: If a reset occurs (even initial power-up reset, d'uh!), wait for
// SIMPLE_BROWNOUT_WAIT_MINUTES before doing any GPS or radio stuff.
#define SIMPLE_BROWNOUT_MODE

// Since HF packet is just a joke, don't spend much energy on that.
#define HF_PACKET_DIVIDER 7

// Max time to wait for GPS acquisition.
#define POSITION_MAX_TIME_S 200

#define REQUIRE_HIGH_PRECISION_POSITIONS true
#define REQUIRE_HIGH_PRECISION_NUM_SATS 4
#define REQUIRE_HIGH_PRECISION_FIXLEVEL 1
#define REQUIRE_HIGH_PRECISION_ALTITUDE true
#define REQUIRE_HIGH_PRECISION_MAX_TIME_S 300

#if MODE==GROUNDTEST
// WSPR once every n radio cycles.
#define WSPR_DIVIDER 1
#define __LOWALTSCHEDULE_SECONDS 30
#define __DAYSCHEDULE_SECONDS 30
#define __NIGHTSCHEDULE_SECONDS 30
#define __LOWBATTSCHEDULE_SECONDS 30
#define __CRISISSCHEDULE_SECONDS 1440
#define __STORAGE_INTERVAL_SECONDS 360
#define SIMPLE_BROWNOUT_WAIT_MINUTES 1

#else
// WSPR once every n radio cycles.
#define WSPR_DIVIDER 3
#define __LOWALTSCHEDULE_SECONDS 120
#define __DAYSCHEDULE_SECONDS 180
#define __NIGHTSCHEDULE_SECONDS 900
#define __LOWBATTSCHEDULE_SECONDS 1800
// 4h
#define __CRISISSCHEDULE_SECONDS 14400
// 1h
#define __STORAGE_INTERVAL_SECONDS 3600
// if brownout, wait 1hr until doing anything requiring power
#define SIMPLE_BROWNOUT_WAIT_MINUTES 60
#endif

#define SIMPLE_BROWNOUT_PERIODS (SIMPLE_BROWNOUT_WAIT_MINUTES*60/RTC_WAKEUP_PERIOD_S)

// 15000 ft
#define LOWALT_THRESHOLD 5000

// VHF each x min
#define LOWALT_SCHEDULE_TIME 'a', (__LOWALTSCHEDULE_SECONDS/RTC_WAKEUP_PERIOD_S)

// VHF each x min
#define DAY_SCHEDULE_TIME 'd', (__DAYSCHEDULE_SECONDS/RTC_WAKEUP_PERIOD_S)

// VHF each x
#define NIGHT_SCHEDULE_TIME 'n', (__NIGHTSCHEDULE_SECONDS/RTC_WAKEUP_PERIOD_S)

// VHF each x
#define LOWBATT_SCHEDULE_TIME 'b', (__LOWBATTSCHEDULE_SECONDS/RTC_WAKEUP_PERIOD_S)

// VHF each x
#define CRISIS_SCHEDULE_TIME 'c', (__CRISISSCHEDULE_SECONDS/RTC_WAKEUP_PERIOD_S)

#define DIRECT_2m_HARDWARE_OUTPUT 3
#define HF_30m_HARDWARE_OUTPUT 2

// When CPU failed runing some equipment, don't try again until
// battery voltage is this much better than at the crashing run.
#define BATTERY_FAILSAFE_VOLTAGE_DELTA 0.15
#define BATTERY_FAILSAFE_TEMPERATURE_DELTA 10

#define STORAGE_INTERVAL_S __STORAGE_INTERVAL_SECONDS
#define MAX_HSE_CALIBRATION_TIME 15000
#define MAX_GPS_TIMELOCK_TIME 300

#define REUSE_CALIBRATION true

#endif /* SRC_SETUP_H_ */
