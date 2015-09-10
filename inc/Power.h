/*
 * Power.h
 *
 *  Created on: Jun 27, 2015
 *      Author: dongfang
 */

#ifndef INC_POWER_H_
#define INC_POWER_H_

#include "Setup.h"
#include <stdint.h>

boolean isDaytimePower();

typedef enum {
	E_DEVICE_GPS,
	E_DEVICE_HF_TX,
	E_DEVICE_VHF_TX,
	E_DEVICE_END
} E_DEVICE;

// Safe if : Never turned on, if turned back off by program, or if voltage is better now than last time.
boolean PWR_isSafeToUseDevice(E_DEVICE device);
void PWR_startDevice(E_DEVICE device);
void PWR_stopDevice(E_DEVICE device);
uint8_t PWR_numBrownouts(E_DEVICE device);

#endif /* INC_POWER_H_ */
