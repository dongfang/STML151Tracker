/*
 * SelfCalibration.h
 *
 *  Created on: May 27, 2015
 *      Author: dongfang
 */

#ifndef SELFCALIBRATION_H_
#define SELFCALIBRATION_H_

#include "Types.h"

#define GPS_TIMEPULSE_WASTED_PULSES 0
#define RTC_WUT_WASTED_PULSES 0

typedef struct {
	// Time before giving up (some hardware or config fault - no signal)
	const uint32_t maxTimeMillis;

	// Peak-Peak modulation in DAC steps to use
	const uint16_t modulation_PP;

	// How many clock cycles to time per round
	const uint16_t numCyclesPerRound;

	// How many rounds (of each one period with low modulation and one with high)
	const uint16_t numRounds;

	// DAC to output test modulation to
	const DACChannel_t DACChannel;
} SelfCalibrationConfig_t;

extern const SelfCalibrationConfig_t WSPR_SELF_CALIBRATION;
extern const SelfCalibrationConfig_t HF_PACKET_SELF_CALIBRATION;
extern const SelfCalibrationConfig_t TRIM_SELF_CALIBRATION;

/*
 * Measure frequency (fsys is reference clock) and deviation, given settings and a trim.
 */
boolean selfCalibrateModulation(
		uint32_t fsys,
		const SelfCalibrationConfig_t* settings,
		uint8_t trim_pF,
		double* deviationMeasured,
		uint32_t* oscillatorFrequencyMeasured);

/*
 * Self-calibrate to find ratio of frequency at given trim and frequency at default trim.
 */
boolean selfCalibrateTrimming(
		double* ppmPerpFMeasured,
		uint8_t trim);

/*
 * Find HSE frequency (using GPS 1PPM signal)
 */
boolean HSECalibration(uint32_t maxTime, uint32_t* result);

/*
 * Find RTC period (count number of HSE cycles in a RTC second)
 */
boolean RTCCalibration(uint32_t maxTime, uint32_t* fSysCyclesPerRTCSecondResult);

typedef struct {
	// Checksum for establishing validity of stored records
	uint32_t checksum;

	// Measured HSE frequency (just for the fun of it)
	uint32_t HSEFrequency;

	// Tranmitter osc frequency measured at default trim
	uint32_t transmitterOscillatorFrequencyAtDefaultTrim;

	// RTC error measured against HSE (which is again against GPS)
	uint16_t RTCNeededCorrectionPP10M;

	// Temperature measured at
	uint8_t temperature;
} CalibrationRecord_t;

#define DEFAULT_CALIBRATION { \
	.HSEFrequency = 16E6,	\
	.transmitterOscillatorFrequencyAtDefaultTrim = 26E6,\
	.RTCErrorPP10M = 1000, /* This xtal is VERY bad */ \
	.temperature = 0, \
	.checksum = 0 \
}

boolean selfCalibrate(CalibrationRecord_t* target);

#endif /* SELFCALIBRATION_H_ */
