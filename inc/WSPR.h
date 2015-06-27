/*
 * WSPR.h
 *
 *  Created on: Mar 12, 2015
 *      Author: dongfang
 */

#ifndef WSPR_H_
#define WSPR_H_

#include <stdint.h>
#include "Types.h"

extern const uint32_t WSPR_FREQUENCIES[];

// The fake subsubgrid message types for WSPR
enum WSPR_FAKE_EXTENDED_LOCATION {
  REAL_EXTENDED_LOCATION,
  SUPERFINE_EXTENDED_LOCATION,
  ALTITUDE,
  TELEMETRY
};

typedef enum {
	THIRTY_M,
	TEN_M
} WSPRBand_t;

//uint8_t WSPRDidUpdate();
//uint8_t WSPREnded();
//void WSPR_shutdownHW();

void prepareWSPRMessage(uint8_t type, enum WSPR_FAKE_EXTENDED_LOCATION extendedFake, float txVoltage);
void WSPR_transmit(WSPRBand_t band, uint32_t oscillatorFrequencyMeasured, float stepModulation);

#endif /* WSPR_H_ */
