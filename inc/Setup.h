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
#define DEFEAT_VOLTAGE_CHECKS true

#define REUSE_CALIBRATION true

#define REQUIRE_HIGH_PRECISION_POSITIONS true
#define REQUIRE_HIGH_PRECISION_NUM_SATS 4
#define REQUIRE_HIGH_PRECISION_FIXLEVEL 1
#define REQUIRE_HIGH_PRECISION_ALTITUDE true
#define REQUIRE_HIGH_PRECISION_MAX_TIME_S 240

#endif /* SRC_SETUP_H_ */
