/*
 * ADC_DMA.h
 *
 *  Created on: May 14, 2015
 *      Author: dongfang
 */

#ifndef ADC_H_
#define ADC_H_

#include "stm32l1xx_conf.h"
#include "Types.h"

#define BATT_MEASURED_VOLTAGE 3.453
#define BATT_READ_ADC12 2991.0

#define SOLAR_MEASURED_VOLTAGE 3.453
#define SOLAR_READ_ADC12 2991.0

#define VCC 2.191
#define ADC_FACTOR (VCC / 4096)

// #define READ_COARSE_BATT_ADC (BATT_READ_ADC12 / 16)

#define BATT_ADC_FACTOR (BATT_MEASURED_VOLTAGE / BATT_READ_ADC12)
#define REV_BATT_ADC_FACTOR (BATT_READ_ADC12 / BATT_MEASURED_VOLTAGE)

#define SOLAR_ADC_FACTOR (SOLAR_MEASURED_VOLTAGE / SOLAR_READ_ADC12)

#define BATT_ADC_2V7 (REV_BATT_ADC_FACTOR * 2.7)
#define BATT_ADC_3V0 (REV_BATT_ADC_FACTOR * 3.0)
#define BATT_ADC_3V3 (REV_BATT_ADC_FACTOR * 3.3)
#define BATT_ADC_3V7 (REV_BATT_ADC_FACTOR * 3.7)
#define BATT_ADC_4V2 (REV_BATT_ADC_FACTOR * 4.2)

#define COARSE(x) ((x)*16)
#define REV_COARSE(x) ((x)/16)

#define NUM_ADC_VALUES 4

extern volatile uint16_t ADCUnloadedValues[NUM_ADC_VALUES];
extern volatile uint16_t ADCLoadedValues[NUM_ADC_VALUES];

extern volatile boolean ADC_DMA_Complete;

// uint16_t ADC_cheaplyMeasureBatteryVoltage() ;

void ADC_DMA_init(volatile uint16_t* conversionTargetArray);
// void ADC_DMA_start(volatile uint16_t* conversionTargetArray);
void ADC_DMA_shutdown();

float ADC_batteryUnloadedVoltage();
float ADC_batteryLoadedVoltage();
float ADC_solarVoltage();
float ADC_temperature();
float ADC_internalTemperature();
int8_t ADC_simpleTemperature(float temp);

#endif /* ADC_H_ */
