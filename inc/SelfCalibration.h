/*
 * SelfCalibration.h
 *
 *  Created on: May 27, 2015
 *      Author: dongfang
 */

#ifndef SELFCALIBRATION_H_
#define SELFCALIBRATION_H_

#include "DataTypes.h"
#include "DAC.h"

// Change carrier frequency this number of cycles
// #define SELF_CALIBRATION_NUMCYCLES 5
#define GPS_TIMEPULSE_WASTED_PULSES 1

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
} SelfCalibrationSettings_t;

extern const SelfCalibrationSettings_t WSPR_SELF_CALIBRATION;
extern const SelfCalibrationSettings_t HF_PACKET_SELF_CALIBRATION;

// For each change of carrier frequency, capture this number of times.
// For 26MHz divided by 1024 in PLL and then again by 8 in ARM, each cycle takes
// about 0.315 ms and counts about 5041 16MHz cpu cycles.
// #define SELF_CALIBRATION_NUM_CAPTURE_CYCLES 5000

// How much in DAC value (square wqve p-p) to change modulation for each cycle.
// #define SELF_CALIBRATION_MODULATION_AMPL_PP 1000

boolean selfCalibrateModulation(
		uint32_t fsys,
		const SelfCalibrationSettings_t* settings,
		double* deviationMeasured,
		double* oscillatorFrequencyMeasured);

boolean getOscillatorCalibration(uint32_t maxTime, uint32_t* fsys);

#endif /* SELFCALIBRATION_H_ */
