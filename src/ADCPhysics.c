/*
 * ADCPhysics.c

 *
 *  Created on: Aug 26, 2015
 *      Author: dongfang
 */

#include <stdint.h>
#include <Setup.h>

// For ground test, see the DummyPhysics source.
#if MODE==FLIGHT

#include "Physics.h"
#include "ADC.h"
#include <diag/trace.h>
#include <math.h>
#include "LED.h"

static float _adcFactor;
static float _VDDa;

void ADC_ensureVDDMeasured() {
	if (_VDDa != 0)
		return;

	ADC_measurement_blocking(ADC_MEASUREMENT_VDD);

	float sampleSum = 0;

	// First sample is thrown away
	for (uint8_t i = 1; i < NUM_ADC_VDD_VALUES; i++) {
		sampleSum += ADC_VDDValues[i];
	}

	sampleSum /= (NUM_ADC_VDD_VALUES-1);

	uint16_t* refintCal = (uint16_t*) 0x1FF80078;
	float VDDa = 3.0 * (*refintCal) / sampleSum;

	trace_printf("VDDa %d mV\n", (int) (VDDa * 1000));

	_VDDa = VDDa;
	_adcFactor = VDDa / 4096;
}

/*
 float PHY_externalTemperature() {
 float vTemp_mV = ADCTemperatureValues[2] * adcFactor * 1000.0f;
 float rootexp = 5.506 * 5.506 + 4 * 0.00176 * (870.6 - vTemp_mV);//30.43
 float temp = (5.506 - sqrt(rootexp)) / (2 * -0.00176) + 30.0;
 return temp;
 }
 */

float PHY_internalTemperature() {

	// TODO maybe this needs not be done soooooo often.
	volatile int i;
	ADC_measurement_blocking(ADC_MEASUREMENT_TEMPERATURE);

	uint16_t* cal30Ptr = (uint16_t*) 0x1FF8007A;
	uint16_t* cal110Ptr = (uint16_t*) 0x1FF8007E;
	float sampleSum = 0;

	// First sample is thrown away.
	for (int i = 1; i < NUM_ADC_TEMPERATURE_VALUES; i++) {
		sampleSum += ADCTemperatureValues[i];
	}

	float compensatedTempADCValue = _VDDa * sampleSum
			/ ((NUM_ADC_TEMPERATURE_VALUES-1) * 3.0);

	float temperature = 30
			+ 80.0 / (*cal110Ptr - *cal30Ptr)
					* (compensatedTempADCValue - *cal30Ptr);
	return temperature;
}

float PHY_batteryBeforeLoadVoltage() {
	return ADCBeforeLoadPowerValues[0] * BATT_ADC_FACTOR * _adcFactor;
}

float PHY_batteryAfterGPSVoltage() {
	return ADCAfterGPSPowerValues[0] * BATT_ADC_FACTOR * _adcFactor;
}

float PHY_batteryAfterHFVoltage() {
	return ADCAfterHFPowerValues[0] * BATT_ADC_FACTOR * _adcFactor;
}

float PHY_solarVoltage() {
	return ADCBeforeLoadPowerValues[1] * _adcFactor;
}

#endif
