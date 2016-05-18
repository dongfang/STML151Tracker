/*
 * HFDriver.h
 *
 *  Created on: May 18, 2016
 *      Author: dongfang
 */

#ifndef INC_HFDRIVER_H_
#define INC_HFDRIVER_H_

typedef enum {
	ONE_DRIVER,
	TWO_DRIVERS,
	FOUR_DRIVERS
} HF_POWER_LEVEL;

HF_POWER_LEVEL HF_power();
void HF_enableDriver(HF_POWER_LEVEL power);
void HF_shutdownDriver();

#endif /* INC_HFDRIVER_H_ */
