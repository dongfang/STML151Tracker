/*
 * PLL.h
 *
 *  Created on: Jun 26, 2015
 *      Author: dongfang
 */

#ifndef INC_PLL_H_
#define INC_PLL_H_

#include <stdint.h>

// the implementation PLL type decides what PLLSetting_t actually is.
#include "CDCE913.h"

// Implementation nominal xtal frequency
#define PLL_XTAL_FREQUENCY CDCE913_XTAL_FREQUENCY

// Implementation type of PLL setting (well, one of them)
#define PLL_Setting_t CDCE913_PLLSetting_t

// Here, define PLL setting type for the Si4463 radio, probably just an uint32.

/*
 * PLL options for different frequencies.
 */

// These are different options for the same frequency.
extern const uint8_t NUM_PLL_OPTIONS_WSPR_10m;
extern const PLL_Setting_t PLL_OPTIONS_WSPR_10m[];

// These are different options for the same frequency.
extern const uint8_t NUM_PLL_OPTIONS_WSPR_30m;
extern const PLL_Setting_t PLL_OPTIONS_WSPR_30m[];

// These are different options for the same frequency (maybe one is enough).
extern const uint8_t NUM_PLL_OPTIONS_APRS_30m;
extern const PLL_Setting_t PLL_OPTIONS_APRS_30m[];

// These are different options but each for its OWN frequency.
extern const uint8_t NUM_PLL_OPTIONS_APRS_DIRECT_2m;
extern const PLL_Setting_t PLL_OPTIONS_APRS_DIRECT_2m[];

void setPLL(uint8_t output, const CDCE913_PLLSetting_t* setting, uint8_t trim);

#endif /* INC_PLL_H_ */
