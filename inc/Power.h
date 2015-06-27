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
boolean isSafeToUseGPS(uint8_t startupVoltage);
boolean isSafeToUseWSPR(uint8_t startupVoltage);
boolean isSafeToUseSi4463(uint8_t startupVoltage);

void startGPS(uint8_t startupVoltage);
void startWSPR(uint8_t startupVoltage);
void startSi4463(uint8_t startupVoltage);

void stopGPS();
void stopWSPR();
void stopSi4463();

#endif /* INC_POWER_H_ */
