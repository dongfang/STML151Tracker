/*
 * RecordStorage.h
 *
 *  Created on: Jun 26, 2015
 *      Author: dongfang
 */

#ifndef INC_RECORDSTORAGE_H_
#define INC_RECORDSTORAGE_H_

#include "Types.h"
#include "Power.h"

typedef struct {
	// We don't bother to store or check the startup time temperature, as an improvement
	// in temperature will cause an improvement in voltage anyway.

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

typedef struct {
	float initialVoltage;
	int8_t initialTemperature;
	boolean wasDeviceRunning;
	boolean wasDeviceSuccesful;
	uint16_t checksum;
} NewStartupRecord_t;

// Should be 4*8 = 32 bytes
typedef struct {
	float lat __attribute__ ((packed));
	float lon __attribute__ ((packed));;
	uint16_t alt __attribute__ ((packed));
	uint16_t compressedTime __attribute__ ((packed));
	uint16_t seq;
	uint8_t compressedBatteryVoltage;
	uint8_t compressedSolarVoltage;
	uint8_t simpleTemperature;
	uint8_t speed;
} StoredPathRecord_t;

// We can store, say, 1 per hour for 4 days
#define NUM_STORED_RECORDS 100

extern NewStartupRecord_t startupLog[E_DEVICE_END];

// 6000 bytes or there about.
extern StoredPathRecord_t storedRecords[NUM_STORED_RECORDS];

uint16_t startupRecordChecksum(E_DEVICE device);
void setStartupRecordChecksum(E_DEVICE device);
boolean checkStartupRecordValid(E_DEVICE device);
void invalidateStartupLogs();

// Use "current" global values.
void compressRecord(StoredPathRecord_t* record);

boolean hasRecordOutForFirstTransmission();
boolean hasRecordOutForLastTransmission();

boolean timeToStoreRecord();
StoredPathRecord_t* nextRecordIn();
StoredPathRecord_t* nextRecordOutForFirstTransmission();
StoredPathRecord_t* nextRecordOutForLastTransmission();

void resetRecordStorageTimer();
void storeToRecord(StoredPathRecord_t* record);

//uint16_t compressDateHoursMinutes(uint8_t date, Time_t* time);
// Uncompress and return date.
uint8_t uncompressDateHoursMinutes(StoredPathRecord_t* record, Time_t* time);
uint16_t uncompressBatteryVoltageTomV(StoredPathRecord_t* record);
uint16_t uncompressSolarVoltageTomV(StoredPathRecord_t* record);
// Not needed. Just use simpleTemperature.
//float uncompressTemperature(StoredPathRecord_t* record);

#endif /* INC_RECORDSTORAGE_H_ */
