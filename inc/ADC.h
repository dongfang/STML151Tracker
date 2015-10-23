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

#define ADC_FACTOR (VCC / 4096)

#define NUM_ADC_VALUES 3

extern volatile uint16_t ADCUnloadedValues[NUM_ADC_VALUES];
extern volatile uint16_t ADCLoadedValues[NUM_ADC_VALUES];

extern volatile boolean ADC_DMA_Complete;

void ADC_DMA_init(volatile uint16_t* conversionTargetArray);
void ADC_DMA_shutdown();


#endif /* ADC_H_ */
