/*
 * DAC.h
 *
 *  Created on: Jun 3, 2015
 *      Author: dongfang
 */

#ifndef DAC_H_
#define DAC_H_

// Setup DAC ch 2 but leave Ch1 floating
void setupChannel2DAC();
void setDAC2(uint16_t value);

#endif /* DAC_H_ */
