/*
 * StabilizedOscillator.c
 *
 *  Created on: Jun 25, 2015
 *      Author: dongfang
 */

#include "StabilizedOscillator.h"

const int8_t temperatureRanges[] = TEMPERATURE_RANGES;
CalibrationRecord_t calibrationByTemperatureRanges[NUM_TEMPERATURE_RANGES];

// If all else fails, use this. Checksum does not match!
const CalibrationRecord_t defaultCalibration = DEFAULT_CALIBRATION;


static uint32_t checksum(const CalibrationRecord_t* record) {
	uint32_t checksum = 0x1F55F1AA;
	checksum += record->HSEFrequency;
	checksum += record->transmitterOscillatorFrequencyAtDefaultTrim * 7;
	checksum += record->RTCNeededCorrectionPP10M * 1319;
	checksum += record->temperature * 12345;
	return checksum;
}

const CalibrationRecord_t* getCalibration(uint8_t temperature, boolean doAttemptCalibrate) {
	// 0:	-60 to -45
	// 1:	-45 to -30
	// 2:	-30 to -15
	// 3:	-15 to 0
	// 4:	 0 to 15
	// 5:	15 or more
	uint8_t index = 0;

	while(index < NUM_TEMPERATURE_RANGES-1 && temperatureRanges[index+1] < temperature)
		index++;

	uint32_t check = checksum(calibrationByTemperatureRanges+index);
	if (check != calibrationByTemperatureRanges[index].checksum) {
		if (!doAttemptCalibrate) {
			return &defaultCalibration;	// damn, we had nothing!
		}

		if (!selfCalibrate(calibrationByTemperatureRanges + index)) {
			return  &defaultCalibration;	// damn, we failed!
		}

		calibrationByTemperatureRanges[index].checksum = checksum(calibrationByTemperatureRanges + index);
	}

	return calibrationByTemperatureRanges + index;
}


int8_t getBestTrimIndex(int16_t desiredPP10M) {
	int16_t bestError = 30000;
	int8_t bestIndex = -1;
	for (uint8_t i = PLL_MIN_TRIM_INDEX_VALUE; i <= PLL_MAX_TRIM_INDEX_VALUE; i++) {

		int16_t test = desiredPP10M - PLL_XTAL_TRIM_PP10M[i];
		if (test < 0)
			test = -test;
		if (test < bestError) {
			bestError = test;
			bestIndex = i;
		}
	}
	return bestIndex;
}

void PLLSettingExperiment(const PLL_Setting_t* pllSettings, uint8_t numSettings, double desiredMultiplication) {
	uint8_t i;
	uint8_t nearestDefaultTrim = 100;
	int8_t bestPLLOptionIndex = -1;
	for (i = 0; i < numSettings; i++) {
		double tooHighFactor = pllSettings[i].mul
				/ desiredMultiplication;
		double asPP10M = (1 - tooHighFactor) * 1E7;
		int8_t settingTrim = getBestTrimIndex(asPP10M);
		int16_t remainingErrorPP10M = asPP10M
				- PLL_XTAL_TRIM_PP10M[settingTrim];
		int8_t distanceDefault = settingTrim - PLL_PREFERRED_TRIM_VALUE;
		if (distanceDefault < 0)
			distanceDefault = -distanceDefault;
		if (distanceDefault < nearestDefaultTrim) {
			nearestDefaultTrim = distanceDefault;
			bestPLLOptionIndex = i;
		}
		trace_printf(
				"%s Mul %u needs correction of %d PP10M. Suggest trim index %d, remaining error %d PP10M\n",
				bestPLLOptionIndex == i ? "*" : " ",
				(int) (pllSettings[i].mul * 1E7), (int) asPP10M,
				settingTrim, remainingErrorPP10M);
	}
}
