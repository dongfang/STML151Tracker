/*
 * SelfCalibration.h
 *
 *  Created on: May 27, 2015
 *      Author: dongfang
 */

#ifndef SELFCALIBRATION_H_
#define SELFCALIBRATION_H_

// Change carrier frequency this number of cycles
#define SELF_CALIBRATION_NUMCYCLES 5

#define GPS_TIMEPULSE_WASTED_PULSES 1

// For each change of carrier frequency, capture this number of times.
// For 26MHz divided by 1024 in PLL and then again by 8 in ARM, each cycle takes
// about 0.315 ms and counts about 5041 16MHz cpu cycles.
#define SELF_CALIBRATION_NUM_CAPTURE_CYCLES 5000

// How much in DAC value (square wqve p-p) to change modulation for each cycle.
#define SELF_CALIBRATION_MODULATION_AMPL_PP 1000

uint8_t selfCalibrateModulation(
		uint32_t maxTime,
		uint32_t fsys,
		double* deviationMeasured,
		double* osciallatorFrequencyMeasured);
uint8_t getOscillatorCalibration(uint32_t maxTime, uint32_t* fsys);

#endif /* SELFCALIBRATION_H_ */
