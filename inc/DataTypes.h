/*
 * DataTypes.h
 *
 *  Created on: May 31, 2015
 *      Author: dongfang
 */

#ifndef DATATYPES_H_
#define DATATYPES_H_

#include <stdint.h>

typedef struct {
	double mul;
	uint16_t N;
	uint8_t P;
	uint8_t pdiv;
	uint8_t Q;
	uint8_t R;
} CDCEL913_PLL_Setting_t;

typedef struct {
	char callsign[7];
	uint8_t ssid;
} AX25_Address_t;

extern const char* MY_CALLSIGN;
extern const uint8_t MY_SSID;

extern const char* AX25_DEST_CALLSIGN;
extern const uint8_t AX25_DEST_SSID;

extern const char* DIGI_PATH1;
extern const uint8_t DIGI_PATH1_TTL;

extern const char* DIGI_PATH2;
extern const uint8_t DIGI_PATH2_TTL;

extern const uint8_t WSPR_POWER_LEVEL;

extern const CDCEL913_PLL_Setting_t *PLL_SETTINGS[];
extern const int32_t WSPRFrequencies[] ;

uint8_t CDCEL913_BestSettingIndex(uint8_t band, double desiredMultiplication);

#endif /* DATATYPES_H_ */
