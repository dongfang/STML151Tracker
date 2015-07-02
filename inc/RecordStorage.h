/*
 * RecordStorage.h
 *
 *  Created on: Jun 26, 2015
 *      Author: dongfang
 */

#ifndef INC_RECORDSTORAGE_H_
#define INC_RECORDSTORAGE_H_

#include "Types.h"

typedef struct {
	float initialVoltageValue;
	boolean wasGPSRunning;
	boolean wasHFTxRunning;
	boolean wasVHFTxRunning;

	// Not used for anything else than show / diags!
	boolean wasGPSSuccessful;
	boolean wasHFTxSuccessful;
	boolean wasVHFTxSuccessful;

	uint16_t checksum;
} StartupRecord_t;

// Should be 4*8 = 32 bytes
typedef struct {
	float lat;
	float lon;
	uint16_t alt;
	uint16_t compressedTime;

	uint8_t compressedBatteryVoltage;
	uint8_t compressedSolarVoltage;
	uint8_t simpleTemperature;
	uint8_t GPSAcqTime;

	int16_t mainOscillatorError;
} StoredPathRecord_t;

// We can store, say, 1 per hour for 4 days
#define NUM_STORED_RECORDS 100

extern StartupRecord_t startupLog;

// 6000 bytes or there about.
extern StoredPathRecord_t storedRecords[NUM_STORED_RECORDS];

uint16_t startupRecordChecksum();
void setStartupRecordChecksum();
boolean checkStartupRecordValid();
void invalidateStartupLog();

// Use "current" global values.
void compressRecord(StoredPathRecord_t* record);

boolean hasRecordOutForFirstTransmission();
boolean hasRecordOutForLastTransmission();

StoredPathRecord_t* nextRecordIn();
StoredPathRecord_t* nextRecordOutForFirstTransmission();
StoredPathRecord_t* nextRecordOutForLastTransmission();

void storeToRecord(StoredPathRecord_t* record) ;

//uint16_t compressDateHoursMinutes(uint8_t date, Time_t* time);
// Uncompress and return date.
uint8_t uncompressDateHoursMinutes(StoredPathRecord_t* record, Time_t* time);

float uncompressBatteryVoltage(StoredPathRecord_t* record);
float uncompressSolarVoltage(StoredPathRecord_t* record);
// Not needed. Just use simpleTemperature.
//float uncompressTemperature(StoredPathRecord_t* record);

#endif /* INC_RECORDSTORAGE_H_ */
