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
#include "ADC.h"
//#include "DAC.h"
#include "Types.h"
#include "PLL.h"
#include "Power.h"
#include "Globals.h"

const uint8_t NUM_APRS_PARAMS = 1;
const struct APRS_PARAM APRS_PARAMS[] = { { "foo", "V", { 0.01, 0.1, 1 } } };

// Module functions
float meters_to_feet(float m) {
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
/*
 * Base91.c
 *
 *  Created on: Apr 11, 2015
 *      Author: dongfang
 */
#include "Types.h"

void base91encode2char(uint16_t val, char* out) {
	if (val > 8280)
		val = 8280;
	out[0] = val / 91 + 33;
	out[1] = val % 91 + 33;
}

/*
 * See http://he.fi/doc/aprs-base91-comment-telemetry.txt for specification of this.
 */
uint8_t compressTelemetry(uint16_t seq, uint8_t nval, uint16_t* vals, char* out) {
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

void base91encode4digit(uint32_t n, char* out) {
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

uint8_t compressPosition(float lat, float lon, float alt, char* out) {
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

uint8_t compressedTimestamp(uint8_t date, Time_t* time, char* out) {
	return sprintf(out, "%02d%02d%02dz", date, time->hours, time->minutes);
}

void aprs_send_header(const AX25_Address_t* destination, uint16_t txDelay) {
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

void aprs_parameterNameMessage(char* out) {
	uint8_t pos = sprintf(out, "PARM.");
	uint8_t needsComma = 0;
	for (uint8_t i = 0; i < NUM_APRS_PARAMS; i++) {
		if (needsComma)
			out[pos++] = ',';
		else
			needsComma = 1;
		for (uint8_t j = 0; j < 8; j++) {
			char c = APRS_PARAMS[i].name[j];
			if (c == 0)
				break;
			out[pos++] = c;
		}
	}
}

void aprs_unitNameMessage(char* out) {
	uint8_t pos = sprintf(out, "PARM.");
	uint8_t needsComma = 0;
	for (uint8_t i = 0; i < NUM_APRS_PARAMS; i++) {
		if (needsComma)
			out[pos++] = ',';
		else
			needsComma = 1;
		for (uint8_t j = 0; j < 8; j++) {
			char c = APRS_PARAMS[i].unit[j];
			if (c == 0)
				break;
			out[pos++] = c;
		}
	}
}

void aprs_coefficientsMessage(char* out) {
	sprintf(out, "EQNS.");
	uint8_t needsComma = 0;
	for (uint8_t i = 0; i < NUM_APRS_PARAMS; i++) {
		for (uint8_t j = 0; j < 3; j++) {
			if (needsComma)
				out[i++] = ',';
			else
				needsComma = 1;
			float p = APRS_PARAMS[i].coeff[j];
			i += sprintf(out + i, "%f", p);
		}
	}
}

// Exported functions
/*
 void aprs_uncompressedPositionMessage(long pressure, float altitude,
 float climb, float temperature) {
 char temp[12];                   // Temperature (int/ext)
 const AX25_Address_t addresses[] = { { D_CALLSIGN, D_CALLSIGN_ID }, // Destination callsign
 { S_CALLSIGN, S_CALLSIGN_ID }, // Source callsign (-11 = balloon, -9 = car)
 #ifdef DIGI_PATH1
 { DIGI_PATH1, DIGI_PATH1_TTL }, // Digi1 (first digi in the chain)
 #endif
 #ifdef DIGI_PATH2
 {	DIGI_PATH2, DIGI_PATH2_TTL}, // Digi2 (second digi in the chain)
 #endif
 };

 ax25_send_header(addresses, sizeof(addresses) / sizeof(AX25_Address_t));
 ax25_send_byte('!'); // Report w/o timestamp, no APRS messaging. $ = NMEA raw data
 //  makeTimestamp(temp);
 //  ax25_send_string(temp);
 makeLatitude(temp);
 ax25_send_string(temp);
 ax25_send_byte('/');                // Symbol table
 makeLongitude(temp);
 ax25_send_string(temp);
 ax25_send_byte('O');                // Symbol: O=balloon, -=QTH
 snprintf_P(temp, 4, PSTR("%03d"), (int) (0)); // Course
 ax25_send_string(temp);             // Course (degrees)
 ax25_send_byte('/');                // and
 snprintf_P(temp, 4, PSTR("%03d"), (int) (0));
 ax25_send_string(temp);             // speed (knots)
 ax25_send_string("/A="); // Altitude (feet). Goes anywhere in the comment boundary
 snprintf_P(temp, 7, PSTR("%06ld"), (long) (meters_to_feet(altitude)));
 ax25_send_string(temp);             // speed (knots)
 ax25_send_string("/P="); // Altitude (feet). Goes anywhere in the comment boundary
 snprintf_P(temp, 7, PSTR("%06ld"), pressure);
 ax25_send_string(temp);             // speed (knots)
 ax25_send_string("/C="); // Climb (feet.min). Goes anywhere in the comment boundary
 snprintf_P(temp, 7, PSTR("%d"), (int) (meters_to_feet(climb)));
 ax25_send_string(temp);
 ax25_send_string("/Vc=");
 snprintf_P(temp, 6, PSTR("%d"), Vcc);
 ax25_send_string(temp);
 ax25_send_string("/Vb=");
 snprintf_P(temp, 6, PSTR("%d"), Vbatt);
 ax25_send_string(temp);
 ax25_send_string("/T=");
 int i_temperarure10 = (int) (temperature * 10);
 int i_temperarure = i_temperarure10 / 10;
 int i_temperature_decimal = i_temperarure10 % 10;
 if (i_temperature_decimal < 0)
 i_temperature_decimal = -i_temperature_decimal;
 sprintf_P(temp, PSTR("%d.%d"), i_temperarure, i_temperature_decimal);
 ax25_send_string(temp);
 ax25_send_byte(' ');
 ax25_send_string(APRS_COMMENT);     // Comment
 ax25_send_footer();
 }
 */

uint8_t sillyOneDecimalValue(char memo, float in, char* out) {
	int i_Tenths = (int) (in * 10);
	int i_Whole = (int) (in);
	int i_decimal = i_Tenths % 10;
	if (i_decimal < 0)
		i_decimal = -i_decimal;
	return sprintf(out, ",%c%d.%d", memo, i_Whole, i_decimal);
}

static uint16_t StatusMessageSequence __attribute__((section (".noinit")));

// Exported functions
void APRS_marshallStatusMessage(
		uint32_t txFrequency,
		uint32_t referenceFrequency,
		uint16_t txDelay
		// Something about uptime, brownout resets, ...
	) {

	char temp[12];
	aprs_send_header(&APRS_DEST, txDelay);
	ax25_send_byte('>');
	ax25_send_byte('?');

	sprintf(temp, "r%u", numRestarts);
	ax25_send_string(temp);

	sprintf(temp, ",s%u", StatusMessageSequence);
	ax25_send_string(temp);

	sprintf(temp, ",f%lu", txFrequency/1000);
	ax25_send_string(temp);

	sillyOneDecimalValue('b',batteryVoltage, temp);
	ax25_send_string(temp);

	sillyOneDecimalValue('l',ADC_batteryLoadedVoltage(), temp);
	ax25_send_string(temp);

	sillyOneDecimalValue('s',solarVoltage, temp);
	ax25_send_string(temp);

	sillyOneDecimalValue('t',temperature, temp);
	ax25_send_string(temp);

	sprintf(temp, ",o%d", PLL_oscillatorError(referenceFrequency));
	ax25_send_string(temp);

	sprintf(temp, ",G%cH%cV%c",
			(PWR_isSafeToUseGPS() ? '1' : '0'),
			(PWR_isSafeToUseHFTx() ? '1' : '0'),
			(PWR_isSafeToUseVHFTx() ? '1' : '0'));
	ax25_send_string(temp);

	sprintf(temp, ",p%u", scheduleSeconds*mainPeriod);
	ax25_send_string(temp);

	sprintf(temp, ",w%u/%u", WSPRCnt, scheduleSeconds*mainPeriod*WSPRPeriod);
	ax25_send_string(temp);

	ax25_end();

	StatusMessageSequence++;
}

static uint16_t telemetrySequence;

uint16_t combineVoltages(float batteryVoltage, float solarVoltage) {
	// the 1000's digit is solar voltage * 8 * 1000 truncated to 1000s and the rest is batt * 100
	// Example: Solar = 0.71 and battery = 3.92:
	// result = 5000 + 392
	// in other words, 1st digit is solar in 1/8 volts steps.
	uint32_t result = solarVoltage * 8000;
	result = result - result%1000;
	if (result > 7000) result = 7000;

	result += batteryVoltage*100;
	return result;
}

void APRS_marshallPositionMessage(uint16_t txDelay) {

	// We offset temp. by 100 degrees so it is never negative.
	// Reportable range is thus: -100C to 728C with 1 decimal.
	float _temperature = temperature + 100;
	if (_temperature < 0)
		_temperature = 0;

	uint16_t WSPRWindowWaitTime = lastWSPRWindowWaitTime / 5;
	if (WSPRWindowWaitTime > 120/5) {
		WSPRWindowWaitTime = 120/5; // Should normally not happen. Why wait more than 120 s max?
	}

	uint16_t GPSFixTime = lastGPSFixTime;
	if (GPSFixTime > 8280 / (120/5) - (120/5)) {	// max value we have space for is 8280 minus the space for max. WSPR wait time = 120 s / compacting factor 5
		GPSFixTime = 8280 / (120/5) - (120/5);	// that's 345 seconds.
	}

	int32_t mainOscillatorError =
			PLL_oscillatorError(currentCalibration->transmitterOscillatorFrequencyAtDefaultTrim) + 8280/2;
	if (mainOscillatorError > 8120) mainOscillatorError = 8120;
	else if (mainOscillatorError < 0) mainOscillatorError = 0;

	uint16_t telemetryValues[] = {
			combineVoltages(batteryVoltage, solarVoltage),
			_temperature*10,			// Will require an offset of 100
			mainOscillatorError,
			GPSFixTime*(120/5) + WSPRWindowWaitTime
			};

	aprs_send_header(&APRS_APSTM1_DEST, txDelay);
	char temp[40];                   // Temperature (int/ext)
	ax25_send_byte('!'); // Report w/o timestamp, no APRS messaging.
	uint8_t end = 0;
	end = compressPosition(
			GPSPosition.lat,
			GPSPosition.lon,
			GPSPosition.alt,
			temp);
	end += compressTelemetry(telemetrySequence, 4, telemetryValues, temp + end);
	telemetrySequence++;
	if (telemetrySequence > 8280) {
		telemetrySequence = 0;
	}

	temp[end] = 0;
	// trace_printf("Sending an APRS message of %d bytes\n", end);

	ax25_send_string(temp);
	ax25_end();
}

void APRS_marshallStoredPositionMessage(StoredPathRecord_t* record, uint16_t txDelay) {
	aprs_send_header(&APRS_APSTM1_DEST, txDelay);
	char temp[40];
	ax25_send_byte('/'); // Report w timestamp, no APRS messaging.
	uint8_t end = 0;

	Time_t uncompressedTime;
	uint8_t date = uncompressDateHoursMinutes(record, &uncompressedTime);
	end = compressedTimestamp(date, &uncompressedTime, temp);
	end += compressPosition(record->lat, record->lon, record->alt, temp + end);

	int16_t mainOscillatorError = record->mainOscillatorError + 8120/2;
		if (mainOscillatorError > 8120) mainOscillatorError = 8120;
		else if (mainOscillatorError < 0) mainOscillatorError = 0;

	uint16_t telemetryValues[] = {
			combineVoltages(uncompressBatteryVoltage(record), uncompressSolarVoltage(record)),
			record->simpleTemperature*10+100,			// Will require an offset of 100
			mainOscillatorError,
			record->GPSAcqTime*(120/5)
			};

	end += compressTelemetry(telemetrySequence, 4, telemetryValues, temp + end);

	telemetrySequence++;
	if (telemetrySequence > 8280) {
		telemetrySequence = 0;
	}

	temp[end] = 0;
	ax25_send_string(temp);
	ax25_end();
}

