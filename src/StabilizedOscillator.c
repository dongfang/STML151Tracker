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

const TransmissionOption_t WSPR_TRANSMISSION_OPTIONS[2] = {
		{
			1,
			sizeof(cdce913_PLL_WSPR_30m)/sizeof(PLL_Setting_t),
			cdce913_PLL_WSPR_30m,
			WSPR_FREQUENCIES[THIRTY_M]
		},{
			2,
			sizeof(cdce913_PLL_WSPR_10m)/sizeof(PLL_Setting_t),
			cdce913_PLL_WSPR_10m,
			WSPR_FREQUENCIES[THIRTY_M]
		}
};

static uint32_t checksum(const CalibrationRecord_t* record) {
	uint32_t checksum = 0x1F55F1AA;
	checksum += record->HSEFrequency;
	checksum += record->transmitterOscillatorFrequencyAtDefaultTrim * 7;
	checksum += record->RTCErrorPP10M * 1319;
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
	for (uint8_t i = TX_OSC_MIN_TRIM_INDEX; i <= TX_OSC_MAX_TRIM_INDEX; i++) {

		int16_t test = desiredPP10M - CDCEL913_XTAL_TRIM_PP10M[i];
		if (test < 0)
			test = -test;
		if (test < bestError) {
			bestError = test;
			bestIndex = i;
		}
	}
	return bestIndex;
}

void PLLSettingExperiment(const WSPR_BandSetting_t* bandSettings,
		double desiredMultiplication) {
	uint8_t i;
	uint8_t nearestDefaultTrim = 100;
	int8_t bestPLLOptionIndex = -1;
	for (i = 0; i < bandSettings->numPLLOptions; i++) {
		double tooHighFactor = bandSettings->PLLOptions[i].mul
				/ desiredMultiplication;
		double asPP10M = (1 - tooHighFactor) * 1E7;
		int8_t settingTrim = getBestTrimIndex(asPP10M);
		int16_t remainingErrorPP10M = asPP10M
				- CDCEL913_XTAL_TRIM_PP10M[settingTrim];
		int8_t distanceDefault = settingTrim - CDCE913_PREFERRED_TRIM;
		if (distanceDefault < 0)
			distanceDefault = -distanceDefault;
		if (distanceDefault < nearestDefaultTrim) {
			nearestDefaultTrim = distanceDefault;
			bestPLLOptionIndex = i;
		}
		trace_printf(
				"%s Mul %u needs correction of %d PP10M. Suggest trim index %d, remaining error %d PP10M\n",
				bestPLLOptionIndex == i ? "*" : " ",
				(int) (bandSettings->PLLOptions[i].mul * 1E7), (int) asPP10M,
				settingTrim, remainingErrorPP10M);
	}
}

void WSPRSynthesisExperiment(int32_t oscillatorFrequencyMeasured) {
	for (WSPRBand_t band = THIRTY_M; band <= TEN_M; band++) {
		trace_printf("Trying band %d\n", band);
		double targetFrequency = WSPR_BAND_SETTINGS[band].frequency;

		double desiredMultiplication = (double) targetFrequency
				/ oscillatorFrequencyMeasured;

		PLLSettingExperiment(WSPR_BAND_SETTINGS + band, desiredMultiplication);
	}
}
