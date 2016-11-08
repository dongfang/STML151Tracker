/*
 * Setup.h
 *
 *  Created on: Jun 27, 2015
 *      Author: dongfang
 */

#ifndef SRC_SETUP_H_
#define SRC_SETUP_H_

#include <stdint.h>
#include "CDCE913.h"

#define WSPR_CALL "OZ1ZOR"
#define APRS_CALL "HB9FDK"
#define MY_APRS_SSID 15

#define FLIGHT 1
#define GROUNDTEST 2
#define MODE FLIGHT

// If defined, VHF will remain keyed unmodulated for some seconds after tx to allow
// checking whether the frequency is right.
// #define TEST_VHF_FREQUENCY

extern const char WSPR_CALLSIGN[];

// we are without a regulator right now
#define ABSOLUTE_MIN_VBATT 2.75
// When battery and temperature are both at least this good,
// ignore old records of CPU fail while running equipment.
// #define DAY_MODE_VOLTAGE 3.7
// #define DAY_MODE_SOLAR_VOLTAGE 0.3

// Development only! Whether we want to defeat the Power.c checks so that we can reset...
#define DEFEAT_VOLTAGE_CHECKS false


// Since HF packet is just a joke, don't spend much energy on that.
#define HF_PACKET_DIVIDER 7

// Max time to wait for GPS acquisition.
#define MAX_GPS_TIMELOCK_TIME 200
#define POSITION_MAX_TIME_S 150
#define REQUIRE_HIGH_PRECISION_MAX_TIME_S 150

#define REQUIRE_HIGH_PRECISION_POSITIONS true
#define REQUIRE_HIGH_PRECISION_NUM_SATS 4
#define REQUIRE_HIGH_PRECISION_FIXLEVEL 1
#define REQUIRE_HIGH_PRECISION_ALTITUDE true

#if MODE == GROUNDTEST
// WSPR once every n radio cycles.
#define SIMPLE_BROWNOUT_WAIT_CYCLES 10

#else
// 1h
#define __STORAGE_INTERVAL_SECONDS 3600
// if brownout and reset, 1/2 hour penalty!
#define SIMPLE_BROWNOUT_SECONDS 1800
#endif

// 15000 ft
#define LOWALT_THRESHOLD 5000

#define DIRECT_2m_HARDWARE_OUTPUT CDCE913_OutputMode_OUTPUT_3
#define HF_30m_HARDWARE_OUTPUT CDCE913_OutputMode_OUTPUT_2

// When CPU failed running some equipment, don't try again until
// battery voltage is this much better than at the crashing run.
#define BATTERY_FAILSAFE_VOLTAGE_DELTA 0.15
#define BATTERY_FAILSAFE_TEMPERATURE_DELTA 10

#define STORAGE_INTERVAL_S __STORAGE_INTERVAL_SECONDS
#define MAX_HSE_CALIBRATION_TIME 15000

#endif /* SRC_SETUP_H_ */
