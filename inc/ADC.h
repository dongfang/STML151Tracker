/*
 * ADC_DMA.h
 *
 *  Created on: May 14, 2015
 *      Author: dongfang
 */

#ifndef ADC_H_
#define ADC_H_

#include "Types.h"
#include "Setup.h"
#include "IndividualBoard.h"
#include <stdint.h>

//#define ADC_FACTOR (VCC / 4096)

typedef enum {
	ADC_MEASUREMENT_POWER_UNLOADED,
	ADC_MEASUREMENT_POWER_LOADED,
	ADC_MEASUREMENT_VDD,
	ADC_MEASUREMENT_TEMPERATURE
} ADC_MEASUREMENT_t;

// VBatt, VSolar
#define NUM_ADC_POWER_VALUES 2
extern volatile uint16_t ADCUnloadedPowerValues[NUM_ADC_POWER_VALUES];
extern volatile uint16_t ADCLoadedPowerValues[NUM_ADC_POWER_VALUES];

// What-ever is needed for temperature measurement.
// Depends if we use external temp. sensor (1 or 2 values) or internal (2 values).
#define NUM_ADC_TEMPERATURE_VALUES 5
extern volatile uint16_t ADCTemperatureValues[NUM_ADC_TEMPERATURE_VALUES];

#define NUM_ADC_VDD_VALUES 5
extern volatile uint16_t ADC_VDDValues[NUM_ADC_VDD_VALUES];

extern volatile boolean ADC_DMA_Complete;

float ADC_internalTemperature();
void ADC_ensureVDDMeasured();
void ADC_DMA_init(ADC_MEASUREMENT_t measurement);
void ADC_measurement_blocking(ADC_MEASUREMENT_t measurement);
void ADC_DMA_shutdown();

#endif /* ADC_H_ */
