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

#include "ax25.h"
#include "aprs.h"
#include "DataTypes.h"
#include "GPS.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#ifdef DEBUG
#include "SerialStream.h"
extern UARTSerial serial0;
#endif

const uint8_t NUM_APRS_PARAMS = 1;
const struct APRS_PARAM APRS_PARAMS[] = {
		{ "foo", "V", { 0.01, 0.1, 1 } } };

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
#include <stdint.h>

void base91encode2char(uint16_t val, char* out) {
	if (val > 8280)
		val = 8280;
	out[0] = val / 91 + 33;
	out[1] = val % 91 + 33;
}

uint8_t compressTelemetry(uint16_t seq, uint8_t nval, uint16_t* vals,
		char* out) {
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
	/*
	 uint8_t q = n / (91UL * 91 * 91);
	 out[0] = q + 33;
	 n -= q * (91UL * 91 * 91);
	 q = n / (91 * 91);

	 out[1] = q + 33;
	 n -= q * (91 * 91);
	 q = n / 91;

	 out[2] = q + 33;
	 n -= q * 91;
	 out[3] = n + 33;
	 */
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

#ifdef DEBUG
	printf_P(PSTR("Alt ft. %u\r\n"), (uint16_t)alt);
#endif

	alt = log(alt) / log(1.002);

	base91encode2char(alt, out + 10);

	out[12] = 0b110101 + 33;

	return 13;
}

void sendCompressedMessage(float alt, float climb, float temperature, uint16_t turbulence) {
	static int seq;
	uint16_t vals[] = {
			1,//(uint16_t)climb,
			2,//(uint16_t)(temperature*10),
			3,//vbatt_unloaded.getValue(),
			4//vbatt_loaded.getValue()
	};
	char temp[30];                   // Temperature (int/ext)
	const AX25_Address_t addresses[] = { { AX25_DEST_CALLSIGN, AX25_DEST_SSID }, // Destination callsign
			{ MY_CALLSIGN, MY_SSID }, // Source callsign (-11 = balloon, -9 = car)
#ifdef DIGI_PATH1
			{ DIGI_PATH1, DIGI_PATH1_TTL }, // Digi1 (first digi in the chain)
#endif
#ifdef DIGI_PATH2
			{	DIGI_PATH2, DIGI_PATH2_TTL}, // Digi2 (second digi in the chain)
#endif
		};

	ax25_send_header(addresses, sizeof(addresses) / sizeof(AX25_Address_t));
	ax25_send_byte('!'); // Report w/o timestamp, no APRS messaging. $ = NMEA raw data
	uint8_t end = 0;
	end = compressPosition(nmeaPositionInfo.lat,nmeaPositionInfo.lon,nmeaPositionInfo.alt, temp);
	end += compressTelemetry(seq, 4, vals, temp + end);
	seq++;
	if (seq > 8280) {
		seq = 0;
	}
	temp[end] = 0;

	ax25_send_string(temp);
	ax25_send_footer();
	ax25_flush_frame();                 // Tell the modem to go
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
	ax25_send_string("/A="); // Altitude (feet). Goes anywhere in the comment area
	snprintf_P(temp, 7, PSTR("%06ld"), (long) (meters_to_feet(altitude)));
	ax25_send_string(temp);             // speed (knots)
	ax25_send_string("/P="); // Altitude (feet). Goes anywhere in the comment area
	snprintf_P(temp, 7, PSTR("%06ld"), pressure);
	ax25_send_string(temp);             // speed (knots)
	ax25_send_string("/C="); // Climb (feet.min). Goes anywhere in the comment area
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

	ax25_flush_frame();                 // Tell the modem to go
}
*/

// Exported functions
void aprs_statusMessage(uint16_t sequence, float altitude, float climb, float temperature, uint16_t turbulence) {
	char temp[12];                   // Temperature (int/ext)
	const AX25_Address_t addresses[] = { { AX25_DEST_CALLSIGN, AX25_DEST_SSID }, // Destination callsign
			{ MY_CALLSIGN, MY_SSID }, // Source callsign (-11 = balloon, -9 = car)
#ifdef DIGI_PATH1
			{ DIGI_PATH1, DIGI_PATH1_TTL }, // Digi1 (first digi in the chain)
#endif
#ifdef DIGI_PATH2
			{	DIGI_PATH2, DIGI_PATH2_TTL}, // Digi2 (second digi in the chain)
#endif
		};

	ax25_send_header(addresses, sizeof(addresses) / sizeof(AX25_Address_t));
	ax25_send_byte('>');
	ax25_send_byte('?');

	sprintf(temp, ",%u", sequence++);
	ax25_send_string(temp);             // speed (knots)
	sprintf(temp, ",%ld", (long) altitude);
	ax25_send_string(temp);             // speed (knots)
	sprintf(temp, ",%d", (int) climb);
	ax25_send_string(temp);
	sprintf(temp, ",%u", 100); //vbatt_unloaded.getValue());
	ax25_send_string(temp);
	sprintf(temp, ",%u", 80);// vbatt_loaded.getValue());
	ax25_send_string(temp);
	int i_temperarure10 = (int) (temperature * 10);
	int i_temperarure = i_temperarure10 / 10;
	int i_temperature_decimal = i_temperarure10 % 10;
	if (i_temperature_decimal < 0)
		i_temperature_decimal = -i_temperature_decimal;
	sprintf(temp, ",%d.%d", i_temperarure, i_temperature_decimal);
	ax25_send_string(temp);
#ifdef USE_MOTION_SENSOR
	sprintf_P(temp, PSTR(",%u"), turbulence);
	ax25_send_string(temp);
#endif

	ax25_send_footer();
	ax25_flush_frame();                 // Tell the modem to go

#ifdef DEBUG
printf_P(PSTR("marshalled.\r\n"));
serial0.flush();
#endif

}

// Exported functions
#ifdef USE_MOTION_SENSOR
void aprs_send(uint16_t sequence, float altitude, float climb, float temperature, uint16_t turbulence) {
	aprs_statusMessage(sequence, altitude, climb, temperature, turbulence);
	//sendCompressedMessage(altitude, climb, temperature);
}
#else
void aprs_send(uint16_t sequence, float altitude, float climb, float temperature) {
	aprs_statusMessage(sequence, altitude, climb, temperature, -1);
}
#endif
