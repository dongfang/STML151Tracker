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

#define NUM_TEMPERATURE_RANGES 11
// #define TEMPERATURE_RANGES {-54,-42,-30,-18,-6,6,18,30};
#define TEMPERATURE_RANGES {-60,-50,-40,-30,-20,-10,0,10,20,30,40}
extern const int8_t temperatureRanges[NUM_TEMPERATURE_RANGES];
extern CalibrationRecord_t calibrationByTemperatureRanges[NUM_TEMPERATURE_RANGES];

extern const CalibrationRecord_t defaultCalibration;

const CalibrationRecord_t* getCalibration(int8_t temperature, boolean doAttemptCalibrate);

/* No longer used. We just calculate locally.
void bestStoredPLLSetting(
		const PLL_Setting_t* pllSettings,
		uint8_t numSettings,
		double desiredMultiplication,
		uint8_t* bestIndex,
		uint8_t* bestTrim);
*/

#endif /* INC_STABILIZEDOSCILLATOR_H_ */
