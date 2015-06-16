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

// WSPR all-inclusive settings
extern const WSPR_BandSetting_t WSPR_BAND_SETTINGS[2];
extern BandCalibration_t WSPR_BAND_CALIBRATIONS[2];

void prepareWSPRMessage(uint8_t type, enum WSPR_FAKE_EXTENDED_LOCATION extendedFake, float txVoltage);
uint8_t WSPRDidUpdate();
uint8_t WSPREnded();
void WSPR_stop();

#endif /* WSPR_H_ */
