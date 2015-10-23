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

// Development only! Whether we want to defeat the Power.c checks so that we can reset...
#define DEFEAT_VOLTAGE_CHECKS false

#define RTC_WAKEUP_PERIOD_S 180

// Experimental: If a reset occurs (even initial power-up reset, d'uh!), wait for
// SIMPLE_BROWNOUT_WAIT_MINUTES before doing any GPS or radio stuff.
// #define SIMPLE_BROWNOUT_MODE

// Since HF packet is just a joke, don't spend much time on that.
#define HF_PACKET_DIVIDER 13

// WSPR once every 3 radio cycles. That would be every 12 minutes in daytime + is own time.
#define WSPR_DIVIDER 4
#define POSITION_MAX_TIME_S 240

#define REQUIRE_HIGH_PRECISION_POSITIONS true
#define REQUIRE_HIGH_PRECISION_NUM_SATS 4
#define REQUIRE_HIGH_PRECISION_FIXLEVEL 1
#define REQUIRE_HIGH_PRECISION_ALTITUDE true
#define REQUIRE_HIGH_PRECISION_MAX_TIME_S 300

#if MODE==GROUNDTEST
#define __LOWALTSCHEDULE_SECONDS 180
#define __DAYSCHEDULE_SECONDS 360
#define __NIGHTSCHEDULE_SECONDS 360
#define __LOWBATTSCHEDULE_SECONDS 720
#define __CRISISSCHEDULE_SECONDS 1440
#define __STORAGE_INTERVAL_SECONDS 360
#define SIMPLE_BROWNOUT_WAIT_MINUTES 3

#else
#define __LOWALTSCHEDULE_SECONDS 180
#define __DAYSCHEDULE_SECONDS 360
#define __NIGHTSCHEDULE_SECONDS 1800
#define __LOWBATTSCHEDULE_SECONDS 7200
#define __CRISISSCHEDULE_SECONDS 12000
#define __STORAGE_INTERVAL_SECONDS 3600
#define SIMPLE_BROWNOUT_WAIT_MINUTES 60
#endif

#define SIMPLE_BROWNOUT_PERIODS (SIMPLE_BROWNOUT_WAIT_MINUTES*60/RTC_WAKEUP_PERIOD_S)

// 20000 ft
#define LOWALT_THRESHOLD 5000

// VHF each 2 min
#define LOWALT_SCHEDULE_TIME 'a', (__LOWALTSCHEDULE_SECONDS/RTC_WAKEUP_PERIOD_S)

// VHF each 5 min
#define DAY_SCHEDULE_TIME 'd', (__DAYSCHEDULE_SECONDS/RTC_WAKEUP_PERIOD_S)

// VHF each 20
#define NIGHT_SCHEDULE_TIME 'n', (__NIGHTSCHEDULE_SECONDS/RTC_WAKEUP_PERIOD_S)

// VHF each 60
#define LOWBATT_SCHEDULE_TIME 'b', (__LOWBATTSCHEDULE_SECONDS/RTC_WAKEUP_PERIOD_S)

// VHF each 240
#define CRISIS_SCHEDULE_TIME 'c', (__CRISISSCHEDULE_SECONDS/RTC_WAKEUP_PERIOD_S)

#define DIRECT_2m_HARDWARE_OUTPUT 2
#define HF_30m_HARDWARE_OUTPUT 3

// When battery and temperature are both at least this good,
// ignore old records of CPU fail while running equipment.
#define BATTERY_FAILSAFE_ALWAYS_SAFE_VOLTAGE 3.8
#define BATTERY_FAILSAFE_ALWAYS_SAFE_TEMPERATURE -12

// When CPU failed runing some equipment, don't try again until
// battery voltage is this much better than at the crashing run.
#define BATTERY_FAILSAFE_VOLTAGE_DELTA 0.15
#define BATTERY_FAILSAFE_TEMPERATURE_DELTA 10

// When getting colder than this, schedule slower. It will happen
// in the evening, and it will happen quickly.
#define NORMAL_SCHEDULE_MIN_TEMPERATURE -12

#define STORAGE_INTERVAL_S __STORAGE_INTERVAL_SECONDS
#define MAX_HSE_CALIBRATION_TIME 30000
#define MAX_GPS_TIMELOCK_TIME 300

#define REUSE_CALIBRATION true

#endif /* SRC_SETUP_H_ */
