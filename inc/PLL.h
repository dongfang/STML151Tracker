/*
 * PLL.h
 *
 *  Created on: Jun 26, 2015
 *      Author: dongfang
 */

#ifndef INC_PLL_H_
#define INC_PLL_H_

#include "Types.h"
#include <stdint.h>

// the implementation PLL type decides what PLLSetting_t actually is.
#include "CDCE913.h"

#define PLL_XTAL_FREQUENCY CDCE913_XTAL_FREQUENCY

#define PLL_PREFERRED_TRIM CDCE913_PREFERRED_TRIM

// Implementation type of PLL setting (well, one of them)
typedef CDCE913_PLLSetting_t PLL_Setting_t;

/*
 * PLL options for different frequencies.
 */

boolean PLL_bestPLLSetting(
		uint32_t oscillatorFrequency,
		uint32_t desiredFrequency,
		double maxError,
		CDCE913_PLLSetting_t* result) ;

int8_t PLL_bestTrim(double desiredTrim);

void setPLL(uint8_t output, const PLL_Setting_t* setting);

void PLL_setXOPassthroughMode(uint8_t trim);

// Shut em down
void PLL_shutdown();

int16_t PLL_oscillatorError(uint32_t measuredFrequency);

// Print settings.
void PLL_printSettings();

#endif /* INC_PLL_H_ */
