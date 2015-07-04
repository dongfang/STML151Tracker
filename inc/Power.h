/*
 * Power.h
 *
 *  Created on: Jun 27, 2015
 *      Author: dongfang
 */

#ifndef INC_POWER_H_
#define INC_POWER_H_

#include "Setup.h"

// Safe if : Never turned on, if turned back off by program, or if voltage is better now than last time.
boolean PWR_isSafeToUseGPS();
boolean PWR_isSafeToUseHFTx();
boolean PWR_isSafeToUseVHFTx();

void PWR_startGPS();
void PWR_startHFTx();
void PWR_startVHFTx();

void PWR_stopGPS();
void PWR_stopHFTx();
void PWR_stopVHFTx();

#endif /* INC_POWER_H_ */
