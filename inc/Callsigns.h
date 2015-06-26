/*
 * Callsigns.h
 *
 *  Created on: May 31, 2015
 *      Author: dongfang
 */

#ifndef CALLSIGNS_H_
#define CALLSIGNS_H_

#include <stdint.h>

// An AX.25 address.
typedef struct {
	const char* callsign;
	uint8_t ssid;
} AX25_Address_t;

#define MYCALL "HB9FDK"

// AX.25 addresses; MY_ADDRESS.callsign is used for all modes.
extern const AX25_Address_t MY_ADDRESS;
extern const AX25_Address_t APRS_APSTM1_DEST;
extern const AX25_Address_t APRS_DEST;
extern const AX25_Address_t APRS_DIGI1;
extern const AX25_Address_t APRS_DIGI2;

#endif /* CALLSIGNS_H_ */
