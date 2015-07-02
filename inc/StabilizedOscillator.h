/*
 * StabilizedOscillator.h
 *
 *  Created on: Jun 25, 2015
 *      Author: dongfang
 */

#ifndef INC_STABILIZEDOSCILLATOR_H_
#define INC_STABILIZEDOSCILLATOR_H_

#include "SelfCalibration.h"
#include "PLL.h"

/*
 * Whereas SelfCalibration.h was about finding clock frequencies of various oscillators,
 * this is about using those frequencies for estimating good oscillator settings.
 */

/*
 * For calibration to temperatures, we have a choice between:
 * 1) Storing measured data. That would be real PLL fosc and RTC real frequency
 * 2) Storing correction data. That would be PLL trim in pF and RTC trim. Drop that.
 */

#define NUM_TEMPERATURE_RANGES 6
// -60 to -45
// -45 to -30
// -30 to -15
// -15 to 0
//  0 to 15
// 15 or more
#define TEMPERATURE_RANGES {-60,-45,-30,-15,0,15};
extern const int8_t temperatureRanges[NUM_TEMPERATURE_RANGES];
extern CalibrationRecord_t calibrationByTemperatureRanges[NUM_TEMPERATURE_RANGES];

/*
 * This should be usable for all modes.
 * The info about which output is used is not here. This is only about frequency generation.
 */
typedef struct {
	const void* PLLSetting;
	uint8_t transmitterOscillatorTrim;
	// # DAC steps each symbol.
	// float symbolModulation;
	// Estimate of the frequency ultimately arrived at.
	uint32_t estimatedFrequency;
} TransmitterTuning_t;

extern const CalibrationRecord_t defaultCalibration;

const CalibrationRecord_t* getCalibration(int8_t temperature, boolean doAttemptCalibrate);

void bestStoredPLLSetting(const PLL_Setting_t* pllSettings, uint8_t numSettings, double desiredMultiplication,
		uint8_t* bestIndex, uint8_t* bestTrim);

//void WSPRSynthesisExperiment(uint32_t oscillatorFrequencyMeasured);
//void PLLSettingExperiment(const PLL_Setting_t* pllSettings, uint8_t numSettings, double desiredMultiplication) ;

#endif /* INC_STABILIZEDOSCILLATOR_H_ */
