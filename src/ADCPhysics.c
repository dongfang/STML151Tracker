/*
 * ADCPhysics.c

 *
 *  Created on: Aug 26, 2015
 *      Author: dongfang
 */

#include "Setup.h"

// For ground test, see the DummyPhysics source.
#if MODE==FLIGHT

#include "Physics.h"
#include "ADC.h"
#include <diag/trace.h>
#include <math.h>

float PHY_temperature() {
	float vTemp_mV = ADCUnloadedValues[2] * ADC_FACTOR * 1000.0f;
	float rootexp = 5.506 * 5.506 + 4 * 0.00176 * (870.6 - vTemp_mV);//30.43
	float temp = (5.506 - sqrt(rootexp)) / (2 * -0.00176) + 30.0;
	return temp;
}

int8_t PHY_simpleTemperature(float t_f) {
	if (t_f>30) t_f = 30; else if (t_f<-65) t_f = -65;
	return (int8_t)t_f;
}

float PHY_batteryUnloadedVoltage() {
	return ADCUnloadedValues[0] * BATT_ADC_FACTOR;
}

float PHY_batteryLoadedVoltage() {
	return ADCLoadedValues[0] * BATT_ADC_FACTOR;
}

float PHY_solarVoltage() {
	return ADCUnloadedValues[1] * SOLAR_ADC_FACTOR;
}

/*
float PHY_internalTemperature() {
	uint16_t* cal30Ptr = (uint16_t*)0x1FF8007A;
	uint16_t* cal110Ptr = (uint16_t*)0x1FF8007E;
	trace_printf("Cal val for 30 is %d and for 110 is %d\n", *cal30Ptr, *cal110Ptr);
	float temperature = 30 + 80.0 / (*cal110Ptr - *cal30Ptr) * (ADCUnloadedValues[3] - *cal30Ptr);
	return temperature;
}
*/
#endif
