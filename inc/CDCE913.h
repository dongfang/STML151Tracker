/*
 * CDCE913.h
 *
 *  Created on: May 27, 2015
 *      Author: dongfang
 */

#ifndef CDCE913_H_
#define CDCE913_H_

#include <stdint.h>
#include "DataTypes.h"

#define CDCE913_TRIM_PF 17
#define CDCE913_SELFCALIBRATION_DIVISION 1000

const TransmitterSetting_t* bestPLLSetting(const WSPR_BandSetting_t* bandSettings, double desiredMultiplication);
void CDCE913_init(uint8_t output, const TransmitterSetting_t* setting);
void CDCE913_setDirectModeWithDivision();
void CDCE913_shutdown();
void CECE913_print();

#endif /* CDCE913_H_ */
