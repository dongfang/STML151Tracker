/*
 * RecordStorage.h
 *
 *  Created on: Jun 26, 2015
 *      Author: dongfang
 */

#ifndef INC_RECORDSTORAGE_H_
#define INC_RECORDSTORAGE_H_

typedef struct {
	uint8_t initialVoltageValue;
	boolean didStartGPS;
	boolean didSurviveGPS;
	boolean didStartWSPR;
	boolean didSurviveWSPR;
	uint8_t checksum;
} StartupRecord_t;

// Should be 5*8 = 40 bytes
typedef struct {
	uint32_t checksum;
	float lat;
	float lon;
	uint16_t timeMinutesOfWeek;
	uint16_t alt;
	uint8_t batteryVoltage;
	uint8_t chargeVoltage;
	uint8_t temperature;
	uint8_t ttl;
} StoredRecord_t;

#define NUM_STORED_RECORDS 150

// 6000 bytes or there about.
extern StoredRecord_t storedRecords[NUM_STORED_RECORDS];

#endif /* INC_RECORDSTORAGE_H_ */
