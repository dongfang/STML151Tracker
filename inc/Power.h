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
boolean isSafeToUseGPS(float startupVoltage);
boolean isSafeToUseHFTx(float startupVoltage);
boolean isSafeToUseVHFTx(float startupVoltage);

void startGPS(float startupVoltage);
void startHFTx(float startupVoltage);
void startVHFTx(float startupVoltage);

void stopGPS();
void stopHFTx();
void stopVHFTx();

#endif /* INC_POWER_H_ */
