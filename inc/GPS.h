#ifndef _NMEA_H_
#define _NMEA_H_

#include "Types.h"

#define UBX_POLL_NAV5_MESSAGE {0x06,0x24,0x00,0x00}
#define UBX_INIT_NAV5_MESSAGE {0x06,0x24,0x24,0x00,0x05,0x00,/*mask allows setting of just the dynModel and fixmode*/\
	6,3, /*dynModel 6 (airborne < 1g) and 2/3D fixmode*/\
	0,0,0,0,0x10,0x27,0,0,/*2D fix alt=0 and 3D alt variance=10000 units*/\
	5,0,/*min elev 5 degrees (we are nice and high up and less might work, but the param seems not settable, drlimit=whatever)*/\
	0xfa,0,0xfa,0,/*Position DoP mask 250 and time DoP 250*/\
	0xfa,0,0x2c,1,/*Position accuracy mask 250m and time accuracy mask 300*/\
	0,0x3c,/*static hold threshold and dgps timeout*/\
	0,0,0,0,0,0,0,0,0,0,0,0/*all the rest are just left at zero */\
	}

/* Checksum algo (from the char after 0x62, till just before checksum bytes):
 * CK_A = 0, CK_B = 0
For(I=0;I<N;I++)
{
    CK_A = CK_A + Buffer[I]
    CK_B = CK_B + CK_A
}
 */
typedef struct {
	uint8_t length;
	uint8_t* message;
} UBX_MESSAGE;

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
#define DEBUG_GPS_SNR 0

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
