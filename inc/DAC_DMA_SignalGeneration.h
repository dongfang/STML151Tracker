/*
 * DAC_DMA_SignalGeneration.h
 *
 *  Created on: Mar 10, 2015
 *      Author: dongfang
 */

#ifndef DAC_DMA_SIGNALGENERATION_H_
#define DAC_DMA_SIGNALGENERATION_H_
#include <stdint.h>

void setupSimpleDAC(void);
void setupDMASignalGeneration(void) ;

extern volatile int numDMACycles;
extern volatile int numTim2Cycles;
void setupDMASignalGeneration_alternative(void);

#endif /* DAC_DMA_SIGNALGENERATION_H_ */
