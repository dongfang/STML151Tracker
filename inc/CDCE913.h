/*
 * CDCE913.h
 *
 *  Created on: May 27, 2015
 *      Author: dongfang
 */

#ifndef CDCE913_H_
#define CDCE913_H_

#include "Types.h"
#include "ExperimentallyDerivedConstants.h"

#define CDCE913_SELFCALIBRATION_DIVISION 1000
#define CDCE913_PREFERRED_TRIM 13

// Settings set for the CDCE(L)913
typedef struct {
  double mul;	// Frequency multiplication given by this setting

  // The rest are implementation-specific and ca be changed if some
  // other PLL with different settings are used.
  uint16_t N;
  uint8_t P;
  uint8_t pdiv;
  uint8_t Q;
  uint8_t R;
  uint16_t M;
  uint8_t trim;
} CDCE913_PLLSetting_t;

extern const int16_t PLL_XTAL_TRIM_PP10M[];

// This is no typedef, as we want it to be representable as an uint8_t to thread though implementation-agnostic code.
typedef enum {
	CDCE913_OutputMode_SHUTDOWN,
	CDCE913_OutputMode_OUTPUT_1,
	CDCE913_OutputMode_OUTPUT_2,
	CDCE913_OutputMode_OUTPUT_3,
	CDCE913_OutputMode_SELFCALIBRATION_DIVISION_AT_1,
	CDCE913_OutputMode_XO_PASSTHROUGH
} CDCE913_OutputMode_t;

// Use feedthrough but do divide down
void CDCE913_setDirectModeWithDivision(uint8_t trim, uint16_t pdiv);

// Use the direct feedthrough (not even a divider is on the path)
// void CDCE913_setXOPassthroughMode(uint8_t trim);

// Use the direct feedthrough (not even a divider is on the path)
// void setPLL(CDCE913_OutputMode_t output,const CDCE913_PLLSetting_t* pllSetting);

#endif /* CDCE913_H_ */
