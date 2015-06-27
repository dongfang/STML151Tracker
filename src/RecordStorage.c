/*
 * RecordStorage.c

 *
 *  Created on: Jun 26, 2015
 *      Author: dongfang
 */
#include "RecordStorage.h"

StartupRecord_t startupLog __attribute__((section (".noinit")));
StoredRecord_t storedRecords[NUM_STORED_RECORDS] __attribute__((section (".noinit")));
uint16_t storedRecordIndexIn __attribute__((section (".noinit")));
uint16_t storedRecordIndexOut __attribute__((section (".noinit")));
uint16_t storedRecordIndexCheck __attribute__((section (".noinit")));

//uint32_t bkup = RTC_ReadBackupRegister(RTC_BKP_DR0);
//trace_printf("BKUP was %u\n", bkup);
//RTC_WriteBackupRegister(RTC_BKP_DR0, bkup + 1);

void setIndexCheck() {
	storedRecordIndexCheck = storedRecordIndexIn + storedRecordIndexOut * 7 + 0xa5a;
}

void checkIndices() {
	if (storedRecordIndexIn + storedRecordIndexOut * 7 + 0xa5a
			== storedRecordIndexCheck)
		return; // Was okay.
	storedRecordIndexIn = 0;
	storedRecordIndexOut = 0;
	setIndexCheck();
}

// Always has space for more. Else overwrite oldest.
StoredRecord_t* nextRecordIn() {
	checkIndices();
	uint16_t result = storedRecordIndexIn;
	storedRecordIndexIn = (storedRecordIndexIn + 1) % NUM_STORED_RECORDS;
	if (storedRecordIndexIn == storedRecordIndexOut) {
		// We ran out of space
		storedRecordIndexOut = (storedRecordIndexOut + 1) % NUM_STORED_RECORDS;
	}
	setIndexCheck();
	return storedRecords + result;
}

StoredRecord_t* nextRecordOut() {
	checkIndices();
	if (storedRecordIndexIn == storedRecordIndexOut)
		return (StoredRecord_t*) 0;
	uint16_t result = storedRecordIndexOut;
	storedRecordIndexOut = (storedRecordIndexOut + 1) % NUM_STORED_RECORDS;
	setIndexCheck();
	return storedRecords + result;
}

uint16_t startupRecordChecksum() {
	uint16_t result = startupLog.wasGPSRunning ? 13 : 17;
	result *= startupLog.wasSi4463Running ? 21 : 29;
	result *= startupLog.wasWSPRRunning ? 31 : 41;
	result *= (uint16_t)(startupLog.initialVoltageValue+1);
	return result;
}

void setStartupRecordChecksum() {
	startupLog.checksum = startupRecordChecksum();
}

void invalidateStartupLog() {
	startupLog.checksum = !startupLog.checksum;
}

boolean checkStartupRecordValid() {
	uint16_t checksum = startupRecordChecksum();
	if (checksum != startupLog.checksum) {
		trace_printf("Startup log checksum invalid\n");
		startupLog.wasGPSRunning = false;
		startupLog.wasSi4463Running = false;
		startupLog.wasWSPRRunning = false;

		startupLog.wasGPSSuccessful = false;
		startupLog.wasSi4463Successful = false;
		startupLog.wasWSPRSuccessful = false;

		startupLog.initialVoltageValue = 255;
		setStartupRecordChecksum();
		return false;
	}
	return true;
}
