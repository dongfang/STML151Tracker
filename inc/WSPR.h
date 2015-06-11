/*
 * WSPR.h
 *
 *  Created on: Mar 12, 2015
 *      Author: dongfang
 */

#ifndef WSPR_H_
#define WSPR_H_

#include <stdint.h>
#include "DataTypes.h"

void prepareWSPRMessage(uint8_t type, enum FAKE_EXTENDED_LOCATION_t extendedFake, float txVoltage);
uint8_t WSPRDidUpdate();
uint8_t WSPREnded();
void WSPR_stop();

#endif /* WSPR_H_ */
