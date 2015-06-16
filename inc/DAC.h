/*
 * DAC.h
 *
 *  Created on: Jun 3, 2015
 *      Author: dongfang
 */

#ifndef DAC_H_
#define DAC_H_

#include "DataTypes.h"

// WSPR stuff - no DMA.
// Setup DAC ch 2 but leave Ch1 floating
void WSPR_DAC_Init();
void setDAC(DACChannel_t channel, uint16_t value);

// AFSK stuff - uses DMA
void AFSK_init(float modulationAmplitude);
void AFSK_shutdown();
void GFSK_init(float modurationAmplitude);
void GFSK_shutdown();

void AFSK_startTransmission();

#endif /* DAC_H_ */
