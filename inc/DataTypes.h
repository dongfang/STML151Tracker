/*
 * DataTypes.h
 *
 *  Created on: May 31, 2015
 *      Author: dongfang
 */

#ifndef DATATYPES_H_
#define DATATYPES_H_

#include <stdint.h>
//#include "CDCE913.h"

typedef enum {
	THIRTY_M,
	TEN_M
} WSPRBand_t;

typedef enum {
	DAC1,
	DAC2
} DACChannel_t;

// An AX.25 address.
typedef struct {
	const char* callsign;
	uint8_t ssid;
} AX25_Address_t;

// Settings set for the CDCE(L)913
typedef struct {
  double mul;	// Frequency multiplication given by this setting

  // The rest are implementation-specific and ca be changed if some
  // other PLL with different settings are used.
  uint16_t N;
  uint8_t P;
  uint8_t pdiv;
  uint8_t Q;
  uint8_t R;
} TransmitterSetting_t;

// The fake subsubgrid message types for WSPR
enum WSPR_FAKE_EXTENDED_LOCATION {
  REAL_EXTENDED_LOCATION,
  SUPERFINE_EXTENDED_LOCATION,
  ALTITUDE,
  TELEMETRY
};

// Enough info to describe a band.
typedef struct {
	const uint8_t hardwareChannel;
	// const uint8_t DACChannel; // Don't bother it is the same for all.
	const uint8_t numPLLOptions;
	const TransmitterSetting_t* PLLOptions;
	const int32_t frequency;
} WSPR_BandSetting_t;

typedef struct {
	const TransmitterSetting_t* bestPLLSetting;
	// # DAC steps each symbol.
	float selfCalibrationSymbolSize;
	// # DAC steps offset to tune to exact freq.
	uint16_t selfCalibrationOffset;
	// Estimate of the frequency (not including above fine tuning)
	double estimatedFrequency;
} BandCalibration_t;

// AX.25 addresses; MY_ADDRESS.callsign is used for all modes.
extern const AX25_Address_t MY_ADDRESS;
extern const AX25_Address_t APRS_APSTM1_DEST;
extern const AX25_Address_t APRS_DEST;
extern const AX25_Address_t APRS_DIGI1;
extern const AX25_Address_t APRS_DIGI2;

typedef uint8_t boolean;
#define true 1
#define false 0

#endif /* DATATYPES_H_ */
