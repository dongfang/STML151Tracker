/*
 * WSPR.h
 *
 *  Created on: Mar 12, 2015
 *      Author: dongfang
 */

#ifndef WSPR_H_
#define WSPR_H_

#include "Types.h"
#include "PLL.h"

extern const uint32_t WSPR_FREQUENCIES[];

// The fake subsubgrid message types for WSPR
enum WSPR_MESSAGE_TYPE {
	TYPE1,
	REAL_EXTENDED_LOCATION,
	// SUPERFINE_EXTENDED_LOCATION,		// not necessary
	ALTITUDE,
	// TELEMETRY						// not necessary
};

#define WSPR_LOWALT_SCHEDULE_DEF {TYPE1, ALTITUDE, ALTITUDE, ALTITUDE, ALTITUDE}
#define WSPR_SCHEDULE_DEF {TYPE1, ALTITUDE, REAL_EXTENDED_LOCATION, ALTITUDE, REAL_EXTENDED_LOCATION}

typedef enum {
	THIRTY_M,
	TEN_M
} WSPRBand_t;

extern const uint8_t WSPR_SCHEDULE[];
extern const uint8_t WSPR_SCHEDULE_LENGTH;

extern const uint8_t WSPR_LOWALT_SCHEDULE[];
extern const uint8_t WSPR_LOWALT_SCHEDULE_LENGTH;

void prepareWSPRMessage(enum WSPR_MESSAGE_TYPE messageType, float txVoltage);
void WSPR_Transmit(
		uint8_t band,
		const PLL_Setting_t* setting,
		float stepModulation);
#endif /* WSPR_H_ */
