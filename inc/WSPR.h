/*
 * WSPR.h
 *
 *  Created on: Mar 12, 2015
 *      Author: dongfang
 */

#ifndef WSPR_H_
#define WSPR_H_

#include <stdint.h>

void prepareWSPRMessage(uint8_t type);
uint8_t WSPRDidUpdate();
uint8_t WSPREnded();
void WSPR_stop();

#endif /* WSPR_H_ */
