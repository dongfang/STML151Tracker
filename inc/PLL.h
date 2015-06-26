/*
 * PLL.h
 *
 *  Created on: Jun 26, 2015
 *      Author: dongfang
 */

#ifndef INC_PLL_H_
#define INC_PLL_H_

#include <stdint.h>
// #include "Bands.h"

// the implementation PLL type decides what PLLSetting_t actually is.
#include "CDCE913.h"

// Implementation nominal xtal frequency
#define PLL_XTAL_FREQUENCY CDCE913_XTAL_FREQUENCY

// Implementation type of PLL setting (well, one of them)
typedef CDCE913_PLLSetting_t PLL_Setting_t;

/*
 * PLL options for different frequencies.
 */

void setPLL(uint8_t output, const PLL_Setting_t* setting, uint8_t trim);

#endif /* INC_PLL_H_ */
