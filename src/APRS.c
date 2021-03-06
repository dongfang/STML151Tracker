/* trackuino copyright (C) 2010  EA5HAV Javi
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include "../inc/APRS.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "AX25.h"
#include "diag/Trace.h"
#include "Physics.h"
#include "Types.h"
#include "PLL.h"
#include "Power.h"
#include "Globals.h"
#include "Setup.h"
#include "StabilizedOscillator.h"

const AX25_Address_t APRS_APSTM1_DEST = {"APSTM1",0};

const AX25_Address_t APRS_DEST = {"APRS",0};
const AX25_Address_t MY_ADDRESS = {APRS_CALL,MY_APRS_SSID};
const AX25_Address_t APRS_DIGI1 = {"",0};
const AX25_Address_t APRS_DIGI2 = {"",0};

const char WSPR_CALLSIGN[] = WSPR_CALL;

// Module functions
static float meters_to_feet(float m) {
	// 10000 ft = 3048 m
	// return m / 0.3048;
	return m * 3.28084; // m to ft.
}

/*
 void makeTimestamp(char* timestamp) {
 unsigned long now = millis();
 now += START_DATE * (1000UL * 3600UL * 24UL)
 + START_HOURS * (1000UL * 3600UL) + START_MINUTES * (1000UL * 60UL);
 int days = now / (1000UL * 3600UL * 24UL);
 now -= days * (1000UL * 3600UL * 24UL);
 if (days > 30)
 days -= 30;
 days += 26;
 int hours = now / (1000UL * 3600UL);
 now -= hours * (1000UL * 3600UL);
 int minutes = now / (1000UL * 60UL);
 now -= minutes * (1000UL * 30UL);
 int seconds = now / (1000UL);
 sprintf_P(timestamp, PSTR("%02d%02d%02dh"), hours, minutes, seconds);
 }
 */
/*
 void makeLatitude(char* out) {
 int minutes_int = (int) latitude_minutes;
 int minutes_frac = (latitude_minutes - minutes_int) * 100;
 if (latitude_degrees < 0) {
 sprintf(out, "%02d", -latitude_degrees);
 sprintf_P(out + 2, PSTR("%02d.%0.2dS"), minutes_int, minutes_frac);
 } else {
 sprintf(out, "%02d", latitude_degrees);
 sprintf_P(out + 2, PSTR("%02d.%0.2dN"), minutes_int, minutes_frac);
 }
 }

 void makeLongitude(char* out) {
 int minutes_int = (int) longitude_minutes;
 int minutes_frac = (longitude_minutes - minutes_int) * 100;
 if (longitude_degrees < 0) {
 sprintf_P(out, PSTR("%03d"), -longitude_degrees);
 sprintf_P(out + 3, PSTR("%02d.%0.2dW"), minutes_int, minutes_frac);
 } else {
 sprintf_P(out, PSTR("%03d"), longitude_degrees);
 sprintf_P(out + 3, PSTR("%02d.%0.2dE"), minutes_int, minutes_frac);
 }
 }
 */

static void base91encode2char(uint16_t val, char* out) {
	if (val > 8280)
		val = 8280;
	out[0] = val / 91 + 33;
	out[1] = val % 91 + 33;
}

static uint8_t statusMessageValue(char memo, float in, char* out) {
	int i_Hundredths = (int) (in * 100);
	int i_Tenths = (int) (in * 10);
	int i_Whole = (int) (in);
	int i_hundredthdigit = i_Hundredths % 10;
	if (i_hundredthdigit < 0)
		i_hundredthdigit = -i_hundredthdigit;
	int i_tenthdigit = i_Tenths % 10;
	if (i_tenthdigit < 0)
		i_tenthdigit = -i_tenthdigit;
	return sprintf(out, ",%c%d.%c%c", memo, i_Whole, (i_tenthdigit + '0'), (i_hundredthdigit + '0'));
}

/*
 * See http://he.fi/doc/aprs-base91-comment-telemetry.txt for specification of this.
 */
static uint8_t compressTelemetry(uint16_t seq, uint8_t nval, uint16_t* vals, char* out) {
	out[0] = '|';
	base91encode2char(seq, out + 1);
	uint8_t pos = 3;
	for (uint8_t j = 0; j < nval; j++) {
		base91encode2char(vals[j], out + pos);
		pos += 2;
	}
	// and at end
	out[pos++] = '|';
	return pos;
}

static void base91encode4digit(uint32_t n, char* out) {
	uint8_t q = n % 91;
	out[3] = q + 33;
	n /= 91;

	q = n % 91;
	out[2] = q + 33;
	n /= 91;

	q = n % 91;
	out[1] = q + 33;
	n /= 91;
	q = n % 91;

	out[0] = q + 33;
}

static uint8_t compressPosition(float lat, float lon, float alt, char* out) {
	// /YYYYXXXX$csT
	// YYYY is 380926 x (90 – latitude) [base 91]
	// latitude is positive for north, negative for south, in degrees.
	// XXXX is 190463 x (180 + longitude) [base 91]
	// longitude is positive for east, negative for west, in degrees.
	// $ is Symbol Code
	// cs is course-speed
	// T is compression type
	out[0] = '/';
	int32_t ilat = 380926 * (90 - lat);
	base91encode4digit(ilat, out + 1);

	int32_t ilon = 190463 * (180 + lon);
	base91encode4digit(ilon, out + 5);
	out[9] = 'O'; // this should be ????? really... ah o for balloon.

	alt = meters_to_feet(alt);
	alt = log(alt) / log(1.002);
	base91encode2char(alt, out + 10);
	out[12] = 0b110101 + 33;
	return 13;
}

static uint8_t compressedTimestamp(uint8_t date, Time_t* time, char* out) {
	return sprintf(out, "%02d%02d%02dz", date, time->hours, time->minutes);
}

static void aprs_send_header(const AX25_Address_t* destination, uint16_t txDelay) {
	ax25_begin(txDelay);

	uint8_t numAddresses = 0;
	const AX25_Address_t* addresses[4];

	addresses[numAddresses++] = destination;
	addresses[numAddresses++] = &MY_ADDRESS;

	if (strlen(APRS_DIGI1.callsign)) {
		addresses[numAddresses++] = &APRS_DIGI1;
	}
	if (strlen(APRS_DIGI2.callsign)) {
		addresses[numAddresses++] = &APRS_DIGI2;
	}

	ax25_send_header(addresses, numAddresses);
}

static uint16_t statusMessageSequence __attribute__((section (".noinit")));

// Exported functions
void APRS_marshallStatusMessage(
		uint32_t txFrequency,
		uint32_t referenceFrequency,
		uint16_t txDelay
		// Something about uptime, brownout resets, ...
	) {

	char temp[12];

	if (statusMessageSequence > 999)
		statusMessageSequence = 0;

	aprs_send_header(&APRS_DEST, txDelay);
	ax25_send_byte('>');

	sprintf(temp, "q%u", statusMessageSequence);
	ax25_send_string(temp);

	sprintf(temp, ",a%d", (int) lastNonzero3DPosition.alt);
	ax25_send_string(temp);

	sprintf(temp, ",d%d", (int) odometer_nm);
	ax25_send_string(temp);

	statusMessageValue('v', climbRate, temp);
	ax25_send_string(temp);

	sprintf(temp, ",r%u", numRestarts);
	ax25_send_string(temp);

	sprintf(temp, ",f%lu", txFrequency / 1000);
	ax25_send_string(temp);

	statusMessageValue('b', batteryVoltage, temp);
	ax25_send_string(temp);

	statusMessageValue('s', solarVoltage, temp);
	ax25_send_string(temp);

	statusMessageValue('g', PHY_batteryAfterGPSVoltage(), temp);
	ax25_send_string(temp);

	statusMessageValue('h', PHY_batteryAfterHFVoltage(), temp);
	ax25_send_string(temp);

	statusMessageValue('t', temperature, temp);
	ax25_send_string(temp);

	const CalibrationRecord_t* cal = getCalibration(simpleTemperature);
	int32_t freqError = cal->transmitterOscillatorFrequencyAtDefaultTrim - PLL_XTAL_DEFAULT_FREQUENCY;
	sprintf(temp, ",o%ld", freqError);
	ax25_send_string(temp);

	char safe = (PWR_isSafeToUseDevice(E_DEVICE_GPS) ? 1 : 0)
			+ (PWR_isSafeToUseDevice(E_DEVICE_HF_TX) ? 2 : 0)
			+ (PWR_isSafeToUseDevice(E_DEVICE_VHF_TX) ? 4 : 0);
	sprintf(temp, ",G%c,", '0' + safe);
	ax25_send_string(temp);

	ax25_send_byte('m');
	ax25_send_byte(scheduleName);

	ax25_end();

	statusMessageSequence++;
}

static uint16_t telemetrySequence __attribute__((section (".noinit")));

void APRS_marshallPositionMessage(uint16_t txDelay) {
	// We offset temp. by 100 degrees so it is never negative.
	// Reportable range is thus: -100C to 728C with 1 decimal.
	float _temperature = temperature + 100;
	if (_temperature < 0)
		_temperature = 0;

	uint16_t telemetryValues[] = {
			(uint16_t) (batteryVoltage * 1000.0f),
			(uint16_t) (solarVoltage * 1000.0f),
			_temperature * 10,	// Will require an offset of 100
			lastGPSFixTime,
			(int16_t)(speed_kts * 10) // conversion to 1/10 kts
	};

	aprs_send_header(&APRS_APSTM1_DEST, txDelay);
	char temp[40];                   // Temperature (int/ext)
	ax25_send_byte('!'); // Report w/o timestamp, no APRS messaging.
	uint8_t end = 0;

	Location_t location = lastNonzeroPosition;
	excludeZones(&location);

	float alt = GPSPosition.alt;
	if (alt < 0) alt = 0;

	end = compressPosition(location.lat, location.lon, alt , temp);

	telemetrySequence++;
	if (telemetrySequence > 8280) {
		telemetrySequence = 0;
	}

	end += compressTelemetry(telemetrySequence, 5, telemetryValues, temp + end);

	temp[end] = 0;
	ax25_send_string(temp);
	ax25_end();
}

void APRS_marshallStoredPositionMessage(
		StoredPathRecord_t* record,
		uint16_t txDelay) {
	aprs_send_header(&APRS_APSTM1_DEST, txDelay);
	char temp[40];
	ax25_send_byte('/'); // Report w timestamp, no APRS messaging.
	uint8_t end = 0;

	Time_t uncompressedTime;
	uint8_t date = uncompressDateHoursMinutes(record, &uncompressedTime);
	end = compressedTimestamp(date, &uncompressedTime, temp);
	end += compressPosition(record->lat, record->lon, record->alt, temp + end);

	int16_t speed = (int16_t)(record->speed * 10); // conversion to 1/10 kts

	uint16_t telemetryValues[] = {
			uncompressBatteryVoltageTomV(record),
			uncompressSolarVoltageTomV(record),
			(record->simpleTemperature +100)*10,			// Will require an offset of 100
			0,
			speed };

	end += compressTelemetry(telemetrySequence, 5, telemetryValues, temp + end);

	telemetrySequence++;
	if (telemetrySequence > 8280) {
		telemetrySequence = 0;
	}

	temp[end] = 0;
	ax25_send_string(temp);
	ax25_end();
}
