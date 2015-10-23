#include "Setup.h"

#if MODE==GROUNDTEST

#include "GPS.h"
#include "RTC.h"
#include <math.h>
#include <string.h>
#include "diag/trace.h"
#include "systick.h"
#include "Globals.h"

extern void onNewGPSData();

// #define CONST_ALT 3000

void simulatedStartTime(Time_t* time) {
	time->hours = 16;
	time->minutes = 0;
	time->seconds = 0;
}

void fakeLinearLocation(int secondsSinceStart) {
	GPSPosition.lat = 47.3416 + secondsSinceStart * 0.00015;
	GPSPosition.lon = 8.2645 + secondsSinceStart * 0.0005;
}

#define RADIUS_DEGREES 0.1
#define ORBIT_HOURS 1

void fakeCircularLocation(int secondsSinceStart) {
	float orbitPernHour = secondsSinceStart / (3600.0 * ORBIT_HOURS) * 6.28318530717959;
	GPSPosition.lat = 47.343 + sin(orbitPernHour)*RADIUS_DEGREES;
	GPSPosition.lon = 8.53 + cos(orbitPernHour)*RADIUS_DEGREES;
}

void GPS_getData() {
	// These are committed by the IRQ handler once messages are complete.
	Time_t time;
	RTC_getTime(&time);

	Time_t simulatedT0;
	simulatedStartTime(&simulatedT0);
	int secondsSinceStart = timeAfter_seconds(&simulatedT0, &time);

	GPSTime.date.date = 20;
	GPSTime.date.month = 10;
	GPSTime.date.valid = true;
	GPSTime.date.year100 = 15;

	GPSTime.time.hours = time.hours;
	GPSTime.time.minutes = time.minutes;
	GPSTime.time.seconds = time.seconds;
	GPSTime.time.valid = true;

	GPSCourseSpeed.course = 100;
	GPSCourseSpeed.groundSpeed = 3;

#ifdef CONST_ALT
	GPSPosition.alt = CONST_ALT;
#else
	GPSPosition.alt = secondsSinceStart > 12000 ? 12000 : secondsSinceStart;
#endif

	fakeCircularLocation(secondsSinceStart);

	GPSPosition.fixTime = GPSTime.time;
	GPSPosition.valid = 'A';

	GPSStatus.fixMode = 2;
	GPSStatus.horizontalAccuracy = 10;
	GPSStatus.numberOfSatellites = 8;

	onNewGPSData();
}

boolean isGPSOn = false;

void GPS_powerOn() {
	isGPSOn = true;
}

void GPS_stopListening() {
}

void GPS_powerOff() {
	isGPSOn = false;
}

boolean GPS_isGPSRunning() {
	return isGPSOn;
}

void GPS_invalidateDateTime() {
}

boolean GPS_isDateTimeValid() {
	return true;
}

void GPS_invalidatePosition() {
}

boolean GPS_isPositionValid() {
	return true;
}

void GPS_invalidateNumSatellites() {
}

uint8_t GPS_numberOfSatellites() {
	return 8;
}

// Set the fake clock
void GPS_powerUpInit() {
	Date_t date;
	date.date = 20;
	date.month = 10;
	date.valid = true;
	date.year100 = 15;

	Time_t time;
	simulatedStartTime(&time);
	RTC_setRTC(&date, &time);
}

#endif
