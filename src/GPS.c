#include <inttypes.h>
#include "GPS.h"
#include "RTC.h"
#include "Globals.h"
#include "Power.h"
#include "systick.h"
#include <diag/Trace.h>
#include "stm32l1xx.h"
#include <math.h>

NMEA_TimeInfo_t GPSTime;
NMEA_CRS_SPD_Info_t GPSCourseSpeed;
Location_t GPSPosition;
NMEA_StatusInfo_t GPSStatus;

uint16_t lastGPSFixTime;

Location_t lastNonzeroPosition; // __attribute__((section (".noinit")));
Location_t lastNonzero3DPosition; // __attribute__((section (".noinit")));

Position_t lastOdometeredPosition __attribute__((section (".noinit")));
double lastOdometeredPositionCheck __attribute__((section (".noinit")));
double odometer_nm __attribute__((section (".noinit")));
float speed_m_s;

Time_t lastOdometerTime __attribute__((section (".noinit")));

Time_t lastAltitudeTime;
float lastAltitude;
float climbRate;

void debugTime(const char* text, Time_t* time) {
	trace_printf("%s said %u:%u:%u\n", text, time->hours, time->minutes,
			time->seconds);
}

// Do some data event processing when data has arrived.
void onNewGPSData() {
	if (GPSPosition.lat != 0 && GPSPosition.lon != 0) {
		lastNonzeroPosition = GPSPosition;

		if (GPSPosition.alt != 0) {
			lastNonzero3DPosition = GPSPosition;

			int timeSinceLastAltitude = timeAfter_seconds(&lastAltitudeTime,
					&GPSTime.time);
			if (timeSinceLastAltitude >= 120) {
				lastAltitudeTime = GPSTime.time;
				float dAltitude = GPSPosition.alt - lastAltitude;
				lastAltitude = GPSPosition.alt;
				climbRate = dAltitude * 60.0 / timeSinceLastAltitude;
//				trace_printf("dAlt is %d, dt is %d, climb rate: %d\n", (int)dAltitude, timeSinceLastAltitude, (int)climbRate);
			}
		}
	}
}

void flashNumSatellites(uint8_t numSatellites) {
	static uint8_t cnt;

	boolean odd = (cnt & 1) != 0;

	if (odd) {
		GPIOB->ODR &= ~GPIO_Pin_6;
	} else {
		if (cnt <= numSatellites * 2) {
			GPIOB->ODR |= GPIO_Pin_6;
		}
	}

	cnt++;
	if (cnt >= 20)
		cnt = 0;
}

extern void GPS_invalidateTime();
extern boolean GPS_isTimeValid();
extern void GPS_invalidatePosition();
extern boolean GPS_isPositionValid();
extern void GPS_getData();
extern void GPS_invalidateNumSatellites();
extern uint8_t GPS_numberOfSatellites();
extern void GPS_powerOn();
extern void GPS_powerOff();
extern boolean GPS_isGPSRunning();
// Implementation of stopListening is elsewhere. It depends on the IO type to GPS.
extern void GPS_stopListening();
// Implementation of GPS_kill is elsewhere. It depends on the power control mechanism.

uint8_t GPS_waitForTimelock(uint32_t maxTime) {
	timer_mark();
	GPS_invalidateTime();

	trace_printf("Waiting for GPS time\n");

	do {
		GPS_getData();
		flashNumSatellites(GPSStatus.numberOfSatellites);
		timer_sleep(200);
	} while ((!GPS_isTimeValid()
			|| (GPSTime.time.hours == 0 && GPSTime.time.minutes == 0
					&& GPSTime.time.seconds == 0)) && !timer_elapsed(maxTime));
	GPIOB->ODR &= ~GPIO_Pin_6;
	GPS_getData();
	if (GPSTime.time.valid) {
		// debugGPSTime();
		return 1;
	} else {
		trace_printf("GPS wait for timelock: FAIL\n");
		return 0;
	}
}

boolean GPS_waitForPosition(uint32_t maxTime) {
	timer_mark();

	GPS_invalidatePosition();

	do {
		GPS_getData();
		flashNumSatellites(GPSStatus.numberOfSatellites);
		timer_sleep(200);
	} while (!GPS_isPositionValid() && !timer_elapsed(maxTime));
	GPIOB->ODR &= ~GPIO_Pin_6;
	GPS_getData();
	if (GPS_isPositionValid()) {
		trace_printf("Got GPS position: %d, %d, %d\n",
				(int) (GPSPosition.lat * 1.0E7),
				(int) (GPSPosition.lon * 1.0E7), (int) GPSPosition.alt);
	} else {
		trace_printf(
				"GPS wait for position FAIL: valid:%c, fixMode:%u numSats:%u\n",
				GPSPosition.valid, GPSStatus.fixMode,
				GPSStatus.numberOfSatellites);
	}
	return GPSPosition.valid == 'A';
}

static void GPS_debugGPSPosition() {
	trace_printf("GPS pos: lat %d, lon %d, alt %d, valid %c, fix %d, sat %d\n",
			(int) (GPSPosition.lat * 1000), (int) (GPSPosition.lon * 1000),
			(int) (GPSPosition.alt), GPSPosition.valid, GPSStatus.fixMode,
			GPSStatus.numberOfSatellites);
}

boolean GPS_waitForPrecisionPosition(uint32_t maxTime) {
	timer_mark();
	GPS_invalidateNumSatellites();
	boolean timeout;
	uint8_t debugPrintCnt = 0;
	do {
		GPS_getData();
		flashNumSatellites(GPSStatus.numberOfSatellites);
		debugPrintCnt++;
		if (debugPrintCnt == 10) {
			debugPrintCnt = 0;
			GPS_debugGPSPosition();
		}
	} while ((GPSPosition.valid != 'A'
			|| (GPS_numberOfSatellites() < REQUIRE_HIGH_PRECISION_NUM_SATS)
			|| (REQUIRE_HIGH_PRECISION_ALTITUDE && GPSPosition.alt == 0)
			|| GPSStatus.fixMode < REQUIRE_HIGH_PRECISION_FIXLEVEL)
			&& !(timeout = timer_elapsed(maxTime)) && timer_sleep(200));

	GPIOB->ODR &= ~GPIO_Pin_6;
	GPS_getData();
	GPS_debugGPSPosition();

	if (timeout) {
		trace_printf("GPS wait for precision pos: FAIL\n");
	} else if (GPSPosition.lat !=0 || GPSPosition.lon != 0) {
		if (lastOdometeredPosition.lat + lastOdometeredPosition.lon + 1
				== lastOdometeredPositionCheck) {
			// valid
			double latFactor = cos(GPSPosition.lat * 0.01745329251994); // convert to radians
			double dist = (GPSPosition.lat - lastOdometeredPosition.lat)
					* (GPSPosition.lat - lastOdometeredPosition.lat);
			dist += (GPSPosition.lon - lastOdometeredPosition.lon)
					* (GPSPosition.lon - lastOdometeredPosition.lon)
					* latFactor;
			dist = sqrt(dist);
			// now it is in degrees latitude, each of which is 60nm, or 1852 * 60m

			dist = dist * 60.0; // now it's in nautical miles
			odometer_nm += dist;

			dist *= 1852; // now it's in m.

			Time_t now;
			RTC_getTime(&now);
			int time_s = timeAfter_seconds(&lastOdometerTime, &now);
			speed_m_s = dist / time_s;

			trace_printf("Dist, time, speed: %d,%d,%d\n", (int)dist, time_s, (int)speed_m_s);

			lastOdometerTime = now;
		} else {
			odometer_nm = 0;
			speed_m_s = 0;
		}

		lastOdometeredPosition.lat = GPSPosition.lat;
		lastOdometeredPosition.lon = GPSPosition.lon;
		lastOdometeredPositionCheck = lastOdometeredPosition.lat
				+ lastOdometeredPosition.lon + 1;
	}

	return !timeout;
}

void GPS_start() {
// GPS on
	PWR_startDevice(E_DEVICE_GPS);
	GPS_powerOn();
}

void GPS_shutdown() {
// Stop IO
	GPS_stopListening();
// Cut power
	GPS_powerOff();
// And note for the crash log that GPS was off.
	PWR_stopDevice(E_DEVICE_GPS);
}
