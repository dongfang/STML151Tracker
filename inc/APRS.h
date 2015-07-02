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

#ifndef __APRS_H__
#define __APRS_H__

#include "GPS.h"
#include "RecordStorage.h"

struct APRS_PARAM {
	char name[8];
	char unit[8];
	float coeff[3];
};

typedef enum {
	COMPRESSED_POSITION_MESSAGE,
	STORED_POSITION_MESSAGE,
	STATUS_MESSAGE
} APRS_MessageType_t;

// Right. Lon is first. That's because Google Earth KML does same, just reduces data conversion effort.
typedef struct {
	const int16_t lon;
	const int16_t lat;
} APRSPolygonVertex_t;

typedef struct {
	const uint32_t frequency;
	// Special out of range vertices mark end of polygon or end of list.
	const APRSPolygonVertex_t* boundary;
	const APRSPolygonVertex_t* core;
} APRSFrequencyRegion_t;

typedef enum {
	AFSK, GFSK
} APRSModulationMode_t;

// APRS
typedef struct {
	// const DACChannel_t DACChannel;
	const APRSModulationMode_t modulationMode;	// FSK or AFSK
	const float modulationAmplitude; 			// Amplitude of sine wave for FM
	// const uint8_t hardwareChannel;			// Interpretation is up to the transmitter HW implementation used.
	// Function that sets up the radio for transmission, transmits and shuts down
	void (*initTransmitter) (uint32_t frequency, uint32_t referenceFrequency);
	void (*shutdownTransmitter) ();
} APRSTransmission_t;

extern const APRSFrequencyRegion_t APRS_WORLD_MAP[];
extern const uint8_t APRS_WORLD_MAP_LENGTH;

extern const uint8_t NUM_APRS_PARAMS;
extern const struct APRS_PARAM APRS_PARAMS[];
extern volatile APRSModulationMode_t currentMode;
extern volatile uint16_t packet_cnt;
extern volatile uint8_t packetTransmissionComplete;

void APRS_frequenciesFromPosition(
		const Position_t* position,
		boolean* frequencyVector,
		boolean* coreVector);
void APRS_debugFrequency(boolean* result) ;
void APRS_debugWorldMap();

void APRS_transmitRandomMessage(
		const APRSTransmission_t* mode,
		APRS_MessageType_t messageType,
		uint32_t frequency,
		uint32_t referenceFrequency);

void APRS_transmitStoredMessage(const APRSTransmission_t* mode,
		StoredPathRecord_t* storedMessage, uint32_t frequency,
		uint32_t referenceFrequency);

extern const APRSTransmission_t APRS_TRANSMISSIONS[];

#endif
