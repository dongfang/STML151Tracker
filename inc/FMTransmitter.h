/*
 * FMTransmitter.h
 *
 *  Created on: Jun 11, 2015
 *      Author: dongfang
 */

#ifndef INC_FMTRANSMITTER_H_
#define INC_FMTRANSMITTER_H_

#include <stdint.h>

void AFSK_startTransmission();
uint8_t AFSK_ended();
void AFSK_endTransmission();

#endif /* INC_FMTRANSMITTER_H_ */
