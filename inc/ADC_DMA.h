/*
 * ADC_DMA.h
 *
 *  Created on: May 14, 2015
 *      Author: dongfang
 */

#ifndef ADC_DMA_H_
#define ADC_DMA_H_

#include "stm32l1xx_conf.h"

extern __IO uint16_t ADC_ConvertedValue;

void ADC_DMA_Config(void);

#endif /* ADC_DMA_H_ */
