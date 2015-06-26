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

#define BATT_ADC_VDIV_FACTOR (BATT_MEASURED_VOLTAGE/BATT_READ_ADC12)
#define SOLAR_ADC_VDIV_FACTOR (SOLAR_MEASURED_VOLTAGE/SOLAR_READ_ADC12)

#define READ_COARSE_BATT_ADC (BATT_READ_ADC12 / 16)

#define COARSE_BATT_ADC_FACTOR (BATT_MEASURED_VOLTAGE / READ_COARSE_BATT_ADC)
#define REV_COARSE_BATT_ADC_FACTOR (READ_COARSE_BATT_ADC / MEASURED_VOLTAGE)

#define COARSE_BATT_ADC_3V0 (REV_ADC_FACTOR * 3.0)
#define COARSE_BATT_ADC_3V3 (REV_ADC_FACTOR * 3.3)
#define COARSE_BATT_ADC_3V7 (REV_ADC_FACTOR * 3.7)
#define COARSE_BATT_ADC_4V2 (REV_ADC_FACTOR * 4.2)

#define NUM_ADC_VALUES 3

extern volatile uint16_t ADCUnloadedValues[NUM_ADC_VALUES];
extern volatile uint16_t ADCLoadedValues[NUM_ADC_VALUES];

extern volatile boolean ADC_DMA_Complete;

uint16_t ADC_cheaplyMeasureBatteryVoltage() ;

void ADC_DMA_init(volatile uint16_t* conversionTargetArray);
// void ADC_DMA_start(volatile uint16_t* conversionTargetArray);
void ADC_DMA_shutdown();

float batteryVoltage(uint16_t ADCvalue12);
float solarVoltage(uint16_t ADCvalue12);
float temperature(uint16_t ADCvalue12);

#endif /* ADC_H_ */