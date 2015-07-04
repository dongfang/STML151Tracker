#ifndef _NMEA_H_
#define _NMEA_H_

#include "Types.h"

typedef enum {
	CONSUMED,
	NEWDATA,
	INVALID,
} MessageState;

// various data in whatever units NMEA has chosen to use.
typedef struct {
	Time_t time;
	Date_t date;
} NMEA_TimeInfo_t;

typedef struct {
	float groundSpeed;
	float course;
} NMEA_CRS_SPD_Info_t;

typedef struct {
	double lat; // 1e-7
	double lon; // 1e-7
	Time_t fixTime;
	float alt;
	char valid; // The A or V of the position
} Position_t;

typedef struct {
	uint8_t fixMode;
	uint8_t numberOfSatellites;
	float horizontalAccuracy;
} NMEA_StatusInfo_t;

#define BAUDRATE 9600
#define DEBUG_GPS_DATA 0

boolean GPS_isGPSRunning();
// It is assumed that PWR_isSafeToUseGPS() was checked already.
void GPS_start();
void GPS_kill();
void GPS_stopUART();
void debugGPSTime();

uint8_t GPS_waitForTimelock(uint32_t maxTime);
uint8_t GPS_waitForPosition(uint32_t maxTime);
uint8_t GPS_waitForPrecisionPosition(uint32_t maxTime) ;

#endif // _NMEA_H_
