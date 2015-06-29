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

void GPS_init(void);
void GPS_shutdown(void);
void debugGPSTime(void);
uint8_t GPS_waitForTimelock(uint32_t maxTime);
uint8_t GPS_waitForPosition(uint32_t maxTime);
uint8_t GPS_waitForPrecisionPosition(uint32_t maxTime) ;

extern NMEA_TimeInfo_t GPSTime;
extern NMEA_CRS_SPD_Info_t GPSCourseSpeed;
extern Position_t GPSPosition;
extern NMEA_StatusInfo_t GPSStatus;
extern Position_t lastNonzeroGPSPosition;

#endif // _NMEA_H_
