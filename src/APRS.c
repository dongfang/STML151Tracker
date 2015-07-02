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
#include "APRSWorldMap.h"
#include "Callsigns.h"
#include "diag/Trace.h"
#include "DAC.h"
#include "Types.h"
#include "PLL.h"
#include "Globals.h"

static const APRSPolygonVertex_t boundary144390[] = BOUNDARY_144390;
static const APRSPolygonVertex_t boundary144620[] = BOUNDARY_144620;
static const APRSPolygonVertex_t boundary144640[] = BOUNDARY_144640;
static const APRSPolygonVertex_t boundary144660[] = BOUNDARY_144660;
static const APRSPolygonVertex_t boundary144800[] = BOUNDARY_144800;
static const APRSPolygonVertex_t boundary144930[] = BOUNDARY_144930;
static const APRSPolygonVertex_t boundary145010[] = BOUNDARY_145010;
static const APRSPolygonVertex_t boundary145175[] = BOUNDARY_145175;
static const APRSPolygonVertex_t boundary145525[] = BOUNDARY_145525;
static const APRSPolygonVertex_t boundary145575[] = BOUNDARY_145575;

static const APRSPolygonVertex_t core144390[] = CORE_144390;
static const APRSPolygonVertex_t core144620[] = CORE_144620;
static const APRSPolygonVertex_t core144640[] = CORE_144640;
static const APRSPolygonVertex_t core144660[] = CORE_144660;
static const APRSPolygonVertex_t core144800[] = CORE_144800;
// 144930 missing
// 145010 missing
static const APRSPolygonVertex_t core145175[] = CORE_145175; // CORE_145175 has no boundary
//145525 missing
static const APRSPolygonVertex_t core145575[] = CORE_145575;

const APRSFrequencyRegion_t APRS_WORLD_MAP[] = { { .frequency = 144390,
		.boundary = boundary144390, .core = core144390 }, { .frequency = 144620,
		.boundary = boundary144620, .core = core144620 }, { .frequency = 144640,
		.boundary = boundary144640, .core = core144640 }, { .frequency = 144660,
		.boundary = boundary144660, .core = core144660 }, { .frequency = 144800,
		.boundary = boundary144800, .core = core144800 }, { .frequency = 144930,
		.boundary = boundary144930, .core = 0 }, { .frequency = 145010,
		.boundary = boundary145010, .core = 0 }, { .frequency = 145175,
		.boundary = boundary145175, .core = core145175 }, { .frequency = 145525,
		.boundary = boundary145525, .core = 0 }, { .frequency = 145575,
		.boundary = boundary145575, .core = core145575 }, };

const uint8_t APRS_WORLD_MAP_LENGTH = sizeof(APRS_WORLD_MAP)
		/ sizeof(APRSFrequencyRegion_t);

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
	alt = log(alt) / log(1.002);
	base91encode2char(alt, out + 10);
	out[12] = 0b110101 + 33;
	return 13;
}

uint8_t compressedTimestamp(uint8_t date, Time_t* time, char* out) {
	return sprintf(out, "%02d%02d%02dz", date, time->hours, time->minutes);
}

void aprs_send_header(const AX25_Address_t* destination) {
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

// Exported functions
void APRS_marshallStatusMessage(
		uint32_t txFrequency,
		uint32_t referenceFrequency
		// Something about uptime, brownout resets, ...
	) {
	static uint16_t sequence;

	char temp[12];                   // T{emperature (int/ext)
	aprs_send_header(&APRS_DEST);
	ax25_send_byte('>');
	ax25_send_byte('?');

	sprintf(temp, "%u", sequence);
	ax25_send_string(temp);

	sprintf(temp, ",%u", 100);
	ax25_send_string(temp);

	sprintf(temp, ",%u", 80);
	ax25_send_string(temp);

	int i_temperarureTenths = (int) (temperature * 10);
	int i_temperatureWhole = (int) (temperature);
	int i_temperature_decimal = i_temperarureTenths % 10;
	if (i_temperature_decimal < 0)
		i_temperature_decimal = -i_temperature_decimal;
	sprintf(temp, ",%d.%d", i_temperatureWhole, i_temperature_decimal);
	ax25_send_string(temp);
	ax25_send_footer();
}

static uint16_t telemetrySequence;

uint16_t combineVoltages(float batteryVoltage, float solarVoltage) {
	// the 1000's digit is solar voltage * 4 * 1000 truncated to 1000s and the rest is batt * 100
	// Example: Solar = 1.23 and battery = 3.92:
	// result = 4000 + 392
	uint32_t result = solarVoltage * 4000;
	result = result - result%4000;
	result += batteryVoltage*100;
	return result;
}

void APRS_marshallPositionMessage() {

	// We offset temp. by 100 degrees so it is never negative.
	// Reportable range is thus: -100C to 728C with 1 decimal.
	int _temperature = temperature + 100;
	if (_temperature < 0)
		_temperature = 0;

	uint16_t GPSFixTime = lastGPSFixTime;
	if (GPSFixTime > 8280 / (120/5)) {	// max value we have space for is 8280 minus the space for max. WSPR wait time = 120 s / compacting factor 5
		GPSFixTime = 8280 / (120/5);	// that's 345 seconds.
	}
	uint16_t WSPRWindowWaitTime = lastWSPRWindowWaitTime / 5;
	if (WSPRWindowWaitTime > 120/5) {
		WSPRWindowWaitTime = 120/5; // Should normally not happen. Why wait more than 120 s max?
	}

	int32_t mainOscillatorError =
			PLL_oscillatorError(currentCalibration->transmitterOscillatorFrequencyAtDefaultTrim) - 8280/2;
	if (mainOscillatorError > 8120) mainOscillatorError = 8120;
	else if (mainOscillatorError < 0) mainOscillatorError = 0;

	uint16_t telemetryValues[] = {
			combineVoltages(batteryVoltage, solarVoltage),
			temperature*10,			// Will require an offset of 100
			mainOscillatorError,
			GPSFixTime*(120/5) + WSPRWindowWaitTime
			};

	aprs_send_header(&APRS_APSTM1_DEST);
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
	trace_printf("Sending an APRS message of %d bytes\n", end);

	ax25_send_string(temp);
	ax25_send_footer();
}

void APRS_marshallStoredPositionMessage(StoredPathRecord_t* record) {
	aprs_send_header(&APRS_APSTM1_DEST);
	char temp[40];
	ax25_send_byte('/'); // Report w timestamp, no APRS messaging.
	uint8_t end = 0;

	Time_t uncompressedTime;
	uint8_t date = uncompressDateHoursMinutes(record, &uncompressedTime);
	end = compressedTimestamp(date, &uncompressedTime, temp);
	end += compressPosition(record->lat, record->lon, record->alt, temp + end);

	int16_t mainOscillatorError = record->mainOscillatorError;
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
	ax25_send_footer();
}

static boolean checkWithinPolygon(int16_t lat, int16_t lon,
		const APRSPolygonVertex_t* boundaryList, uint16_t* index) {

	int16_t initialLat = boundaryList[*index].lat;
	int16_t initialLon = boundaryList[*index].lon;
	boolean result = true;
	// check for all segments AB in clockwise direction that angle (AB, AP) is positive
	do {
		int16_t previousLon = boundaryList[*index].lon;
		int16_t previousLat = boundaryList[*index].lat;
		// trace_printf("Trying: lat:%d, lon:%d @ index %d\n", previousLat, previousLon, *index);
		(*index)++;
		int ap0 = lon - previousLon;
		int ap1 = lat - previousLat;
		int ab0 = boundaryList[*index].lon - previousLon;
		int ab1 = boundaryList[*index].lat - previousLat;
		int cross = ap0 * ab1 - ap1 * ab0;
		if (cross < 0) {
			result = false;
		}
	} while (boundaryList[*index].lat != initialLat
			|| boundaryList[*index].lon != initialLon);
	// Now index points at the last vertex which is same values as the first..
	// Take it one step further.
	// trace_printf("Return to origin detected (or went stoopid)\n");
	(*index)++;
	return result;
}

boolean checkWithinRegion(int16_t lat, int16_t lon,const APRSFrequencyRegion_t* region) {
	boolean within = false;
	uint16_t vertexIndex = 0;
	//trace_printf("Trying region: %u\n", region->frequency);
	while (!within
			&& region->boundary[vertexIndex].lat != POLYGON_LIST_END_DEGREES) {
		within = checkWithinPolygon(lat, lon, region->boundary, &vertexIndex);
	}
	return within;
}

boolean checkWithinCore(int16_t lat, int16_t lon, const APRSFrequencyRegion_t* region) {
	boolean within = false;
	uint16_t vertexIndex = 0;
	while (!within
			&& region->core[vertexIndex].lat != POLYGON_LIST_END_DEGREES) {
		within = checkWithinPolygon(lat, lon, region->core, &vertexIndex);
	}
	return within;
}

void APRS_frequencies(int16_t lat, int16_t lon, boolean* frequenciesVector, boolean* isCoreVector) {
	for (int i = 0; i < APRS_WORLD_MAP_LENGTH; i++) {
		boolean isWithinRegion = checkWithinRegion(lat, lon, APRS_WORLD_MAP+i);
		if (isWithinRegion) {
			frequenciesVector[i] = true;
			isCoreVector[i] = checkWithinCore(lat, lon, APRS_WORLD_MAP+i);
		} else {
			frequenciesVector[i] = false;
			isCoreVector[i] = false;
		}
	}
}

void APRS_frequenciesFromPosition(
		const Position_t* position,
		boolean* frequenciesVector,
		boolean* isCoreVector) {
	APRS_frequencies((int16_t)(position->lat + 0.5),
			(int16_t)(position->lon + 0.5), frequenciesVector, isCoreVector);
}


/**
 * All debugging stuff below!
 */
void checkWorldMap_Orientation_Polygon(const APRSPolygonVertex_t* boundaryList,
		uint16_t* index) {
	int16_t initialLat = boundaryList[*index].lat;
	int16_t initialLon = boundaryList[*index].lon;
	int16_t previousLonVector = 0;									// n-1 to n
	int16_t previousLatVector = 0;
	uint8_t ttl = 100;
	// check for all segments AB in clockwise direction that angle (AB, AP) is positive
	do {
		int16_t previousLon = boundaryList[*index].lon;
		int16_t previousLat = boundaryList[*index].lat;
		(*index)++;
		int16_t nextLonVector = boundaryList[*index].lon - previousLon;	// n to n+1
		int16_t nextLatVector = boundaryList[*index].lat - previousLat;
		int cross = previousLatVector * nextLonVector
				- previousLonVector * nextLatVector;

		if (cross < 0) {
			trace_printf("Concave polygon near {%d,%d}\n", previousLon,
					previousLat);
		}

		previousLonVector = nextLonVector;
		previousLatVector = nextLatVector;
		if (--ttl == 0) {
			trace_printf("Never ending loop near {%d,%d}\n", previousLon,
					previousLat);
		}
	} while (boundaryList[*index].lat != initialLat
			|| boundaryList[*index].lon != initialLon);
	// Now index points at the last vertex which is same values as the first..
	// Take it one step further.
	// trace_printf("Return to origin detected (or went stoopid)\n");
	(*index)++;
}

void APRS_checkWorldMapBoundaries_convex_polygons() {
	trace_printf("Boundary convex Check\n");
	for (int i = 0; i < APRS_WORLD_MAP_LENGTH; i++) {
		uint16_t vertexIndex = 0;
		const APRSFrequencyRegion_t* region = &APRS_WORLD_MAP[i];
		//trace_printf("Trying region: %u\n", region->frequency);
		while (region->boundary[vertexIndex].lat != POLYGON_LIST_END_DEGREES) {
			checkWorldMap_Orientation_Polygon(region->boundary, &vertexIndex);
		}
	}
	trace_printf("Boundary convex Check done!\n");
}

void APRS_checkWorldMapCoreAreas_convex_polygons() {
	trace_printf("Core Area convex Check\n");
	for (int i = 0; i < APRS_WORLD_MAP_LENGTH; i++) {
		uint16_t vertexIndex = 0;
		const APRSFrequencyRegion_t* region = &APRS_WORLD_MAP[i];
		// trace_printf("Trying cores for frequency %u\n", region->frequency);
		if (region->core == 0)
			continue;
		while (region->core[vertexIndex].lat != POLYGON_LIST_END_DEGREES) {
			// trace_printf("Trying poly starting at {%d,%d}\n",region->core[vertexIndex].lon,region->core[vertexIndex].lat);
			checkWorldMap_Orientation_Polygon(region->core, &vertexIndex);
		}
	}
	trace_printf("Core Area convex done!\n");
}

void APRS_checkCoreVertexExcursion() {
	trace_printf("Core area excursion check\n");
	for (int i = 0; i < APRS_WORLD_MAP_LENGTH; i++) {
		uint16_t coreVertexIndex = 0;
		const APRSFrequencyRegion_t* region = &APRS_WORLD_MAP[i];
		// trace_printf("Trying cores for frequency %u\n", region->frequency);
		if (region->core == 0)
			continue;
		while (region->core[coreVertexIndex].lat != POLYGON_LIST_END_DEGREES) {
			// trace_printf("Trying poly starting at {%d,%d}\n",region->core[vertexIndex].lon,region->core[vertexIndex].lat);
			if (!checkWithinRegion(region->core[coreVertexIndex].lat, region->core[coreVertexIndex].lon, region)) {
				trace_printf("Excursing core vertex at {%d,%d}\n", region->core[coreVertexIndex].lon, region->core[coreVertexIndex].lat);
			}
			coreVertexIndex++;
		}
	}
	trace_printf("Core area excursion check done!\n");
}

void APRS_debugWorldMap() {
	boolean frequencyList[APRS_WORLD_MAP_LENGTH];
	boolean core[APRS_WORLD_MAP_LENGTH];

	for (int16_t lat = 70; lat >= -70; lat--) {
		for (int16_t lon = -179; lon <= 180; lon++) {
			APRS_frequencies(lat, lon, frequencyList, core);
			trace_printf("%d;%d;", lat, lon);
			boolean needsComma = false;
			for (int i = 0; i < APRS_WORLD_MAP_LENGTH; i++) {
				if (frequencyList[i]) {
					if (needsComma)
						trace_printf(",");
					else
						needsComma = true;
					trace_printf("%u", APRS_WORLD_MAP[i].frequency);
				}
			}
			trace_printf("\n");
		}
	}
}

void APRS_debugFrequency(boolean* result) {
	for (int i = 0; i < APRS_WORLD_MAP_LENGTH; i++) {
		trace_printf("Frequency %u:", APRS_WORLD_MAP[i].frequency);
		if (result[i]) {
			trace_printf("*");
		}
		trace_printf("\n");
	}
}
