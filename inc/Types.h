/*
 * Types.h
 *
 *  Created on: Jun 26, 2015
 *      Author: dongfang
 */

#ifndef INC_TYPES_H_
#define INC_TYPES_H_

#include <stdint.h>

typedef uint8_t boolean;
#define true 1
#define false 0

typedef struct {
	uint8_t year100;
	uint8_t month;
	uint8_t date;
	boolean valid;
} Date_t;

typedef struct {
	uint8_t hours;
	uint8_t minutes;
	uint8_t seconds;
	boolean valid;
} Time_t;

typedef struct {
	double lat; // 1e-7
	double lon; // 1e-7
} Position_t;

typedef struct {
	double lat; // 1e-7
	double lon; // 1e-7
	Time_t fixTime;
	float alt;
	char valid; // The A or V of the position
} Location_t;

typedef enum {
	DAC1,
	DAC2
} DACChannel_t;


#endif /* INC_TYPES_H_ */
