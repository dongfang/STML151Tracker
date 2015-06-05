#ifndef _NMEA_H_
#define _NMEA_H_

#include <stdint.h>

typedef enum {
	CONSUMED,
	NEWDATA,
	INVALID,
} MessageState;

typedef struct {
	uint8_t hours;
	uint8_t minutes;
	uint8_t seconds;
	uint8_t valid;
} Time_t;

typedef struct {
	uint8_t year100;
	uint8_t month;
	uint8_t date;
	uint8_t valid;
} Date_t;

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
} NMEA_PositionInfo_t;

typedef struct {
	uint8_t fixMode;
	uint8_t numberOfSatellites;
	float horizontalAccuracy;
} NMEA_StatusInfo_t;

#define BAUDRATE 9600
#define DEBUG_GPS_DATA 0

void GPS_init(void);
void GPS_shutdown(void);
uint8_t GPS_waitForTimelock(uint32_t maxTime);
uint8_t GPS_waitForPosition(uint32_t maxTime);
uint8_t GPS_waitForPrecisionPosition(uint32_t maxTime) ;

extern NMEA_TimeInfo_t nmeaTimeInfo;
extern NMEA_CRS_SPD_Info_t nmeaCRSSPDInfo;
extern NMEA_PositionInfo_t nmeaPositionInfo;
extern NMEA_StatusInfo_t nmeaStatusInfo;

#endif // _NMEA_H_
