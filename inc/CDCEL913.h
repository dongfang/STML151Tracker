/*
 * CDCEL913.h
 *
 *  Created on: May 27, 2015
 *      Author: dongfang
 */

#ifndef CDCEL913_H_
#define CDCEL913_H_

#include <stdint.h>

#include "DataTypes.h"

#define CDCEL913_SELFCALIBRATION_DIVISION 1000
#define CDCEL_TRIM_PF 15

void CDCEL913_init();
void CDCEL913_setPLL(const CDCEL913_PLL_Setting_t* setting);
void CDCEL913_setDirectModeWithDivision();
void CDCEL913_enableOutput(uint8_t whichOutput, uint16_t pDiv);
void CECEL913_print();

#endif /* CDCEL913_H_ */
