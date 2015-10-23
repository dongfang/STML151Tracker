/*
 * RecordStorage.c

 *
 *  Created on: Jun 26, 2015
 *      Author: dongfang
 */
#include "RecordStorage.h"
#include "Globals.h"
#include "RTC.h"
#include "PLL.h"
#include <diag/trace.h>

// StartupRecord_t startupLog __attribute__((section (".noinit")));
NewStartupRecord_t startupLog[E_DEVICE_END] __attribute__((section (".noinit")));

StoredPathRecord_t storedRecords[NUM_STORED_RECORDS] __attribute__((section (".noinit")));
static uint16_t storedRecordIndexIn __attribute__((section (".noinit")));
static uint16_t storedRecordIndexOutHead __attribute__((section (".noinit")));
static uint16_t storedRecordIndexOutTail __attribute__((section (".noinit")));
static uint16_t storedRecordIndexCheck __attribute__((section (".noinit")));
static Time_t nextStorageTime __attribute__((section (".noinit")));

//uint32_t bkup = RTC_ReadBackupRegister(RTC_BKP_DR0);
//trace_printf("BKUP was %u\n", bkup);
//RTC_WriteBackupRegister(RTC_BKP_DR0, bkup + 1);

uint16_t checksum() {
	return storedRecordIndexIn + storedRecordIndexOutHead * 13
			+ storedRecordIndexOutTail * 17 + 0xa5a;
}

void setIndexCheck() {
	storedRecordIndexCheck = checksum();
}

void resetRecordStorageTimer() {
	RTC_getTime(&nextStorageTime); // we might as well start storing NOW.
}

void checkRecordIndexes() {
	if (storedRecordIndexCheck == checksum())
		return; // Was okay.
	trace_printf("Record storage checksum mismatch, resetting it all :( \n");
	storedRecordIndexIn = 0;
	storedRecordIndexOutHead = 0;
	storedRecordIndexOutTail = 0;
	setIndexCheck();
	resetRecordStorageTimer(); // we might as well start storing NOW.
}

// This is rather primitive! Only works within +- 12h. Should be okay though.
int secondsBeforeNextStorageTime() {
	Time_t time;
	RTC_getTime(&time);
	int timeToWait = timeDiffModulo24(&time, &nextStorageTime);
	return timeToWait;
}

boolean timeToStoreRecord() {
	checkRecordIndexes();
	int secondsTillStorageTime = secondsBeforeNextStorageTime();
	trace_printf("%d till storage time\n", secondsTillStorageTime);
	return secondsTillStorageTime <= 0;
}

void addIntervalToNextStorageTime() {
	int seconds = nextStorageTime.seconds + STORAGE_INTERVAL_S;
	nextStorageTime.seconds  = seconds % 60;
	int minutes = seconds / 60 + nextStorageTime.minutes + STORAGE_INTERVAL_S/60;
	nextStorageTime.minutes  = minutes % 60;
	int hours = minutes / 60 + nextStorageTime.hours + STORAGE_INTERVAL_S/3600;
	nextStorageTime.hours = hours % 24;
}

// Always has space for more. Else overwrite oldest.
StoredPathRecord_t* nextRecordIn() {
	addIntervalToNextStorageTime();

	checkRecordIndexes();
	uint16_t result = storedRecordIndexIn;
	storedRecordIndexIn = (storedRecordIndexIn + 1) % NUM_STORED_RECORDS;
	if (storedRecordIndexIn == storedRecordIndexOutTail) {
		// We ran out of space. Discard some old crap.
		storedRecordIndexOutTail = (storedRecordIndexOutTail + 1)
				% NUM_STORED_RECORDS;
	}
	if (storedRecordIndexIn == storedRecordIndexOutHead) {
		// We ran out of space. Discard some old crap.
		storedRecordIndexOutHead = (storedRecordIndexOutHead + 1)
				% NUM_STORED_RECORDS;
	}
	trace_printf("Stored a record, indices are now %d %d %d\n",
			storedRecordIndexIn, storedRecordIndexOutHead,
			storedRecordIndexOutTail);
	setIndexCheck();
	return storedRecords + result;
}

StoredPathRecord_t* nextRecordOutForFirstTransmission() {
	uint16_t result = storedRecordIndexOutHead;
	storedRecordIndexOutHead = (storedRecordIndexOutHead + 1)
			% NUM_STORED_RECORDS;
	trace_printf("Returned a record for 1, indices are now %d %d %d\n",
			storedRecordIndexIn, storedRecordIndexOutHead,
			storedRecordIndexOutTail);
	setIndexCheck();
	return storedRecords + result;
}

StoredPathRecord_t* nextRecordOutForLastTransmission() {
	uint16_t result = storedRecordIndexOutTail;
	storedRecordIndexOutTail = (storedRecordIndexOutTail + 1)
			% NUM_STORED_RECORDS;
	trace_printf("Returned a record for 2, indices are now %d %d %d\n",
			storedRecordIndexIn, storedRecordIndexOutHead,
			storedRecordIndexOutTail);
	setIndexCheck();
	return storedRecords + result;
}

boolean hasRecordOutForFirstTransmission() {
	checkRecordIndexes();
	if (storedRecordIndexIn == storedRecordIndexOutHead) {
		trace_printf("No more records for 1\n");
		return false;
	}
	return true;
}

boolean hasRecordOutForLastTransmission() {
	checkRecordIndexes();
	if (storedRecordIndexOutHead == storedRecordIndexOutTail) {
		trace_printf("No more records for 2\n");
		return false;
	}
	return true;
}

/*
uint16_t startupRecordChecksum() {
	uint16_t result = startupLog.wasGPSRunning ? 13 : 17;
	result *= startupLog.wasHFTxRunning ? 21 : 29;
	result *= startupLog.wasVHFTxRunning ? 31 : 41;
	result += (uint16_t)(startupLog.initialVoltageValue * 1000);
	return result;
}
*/

uint16_t startupRecordChecksum(E_DEVICE device) {
	uint16_t result = startupLog[device].wasDeviceRunning ? 13 : 17;
	result *= startupLog[device].wasDeviceSuccesful ? 21 : 29;
	result += (uint16_t)(startupLog[device].initialVoltage * 1000);
	result += (uint16_t)(startupLog[device].initialTemperature << 8);
	return result;
}

void setStartupRecordChecksum(E_DEVICE device) {
	startupLog[device].checksum = startupRecordChecksum(device);
}

void invalidateStartupLogs() {
	for (E_DEVICE device = 0; device < E_DEVICE_END; device++)
	startupLog[device].checksum = !startupLog[device].checksum;
}

/*
 * boolean checkStartupRecordValid() {
	uint16_t checksum = startupRecordChecksum();
	if (checksum != startupLog.checksum) {
		trace_printf("Startup log checksum invalid\n");
		startupLog.wasGPSRunning = false;
		startupLog.wasHFTxRunning = false;
		startupLog.wasVHFTxRunning = false;

		startupLog.wasGPSSuccessful = false;
		startupLog.wasHFTxSuccessful = false;
		startupLog.wasVHFTxSuccessful = false;

		startupLog.initialVoltageValue = 255;
		setStartupRecordChecksum();
		return false;
	}
	return true;
}
*/

boolean checkStartupRecordValid(E_DEVICE device) {
	uint16_t checksum = startupRecordChecksum(device);
	if (checksum == startupLog[device].checksum) {
		return true;
	} else {
		trace_printf("Startup log checksum invalid\n");
		/*
		startupLog[device].wasDeviceRunning = false;
		startupLog[device].wasDeviceSuccesful = false;
		startupLog[device].initialVoltage = 255;
		startupLog[device].initialTemperature = 0;
		setStartupRecordChecksum(device);
		*/
		return false;
	}
}

uint16_t compressGPSTime(const NMEA_TimeInfo_t* datetime) {
	uint16_t result = datetime->date.date;
	result = result * 24 + datetime->time.hours;
	result = result * 60 + datetime->time.minutes;
//	result = result * 60 + datetime->time.seconds;
	return result;
}

uint16_t compressRTCDatetime() {
	uint8_t date;
	uint8_t hours24;
	uint8_t minutes;
	RTC_getDHM(&date, &hours24, &minutes);
	uint16_t result = date;
	result = result * 24 + hours24;
	result = result * 60 + minutes;
//	result = result * 60 + datetime->time.seconds;
	return result;
}

// Uncompress and return date.
uint8_t uncompressDateHoursMinutes(StoredPathRecord_t* record, Time_t* time) {
	//time->seconds = compressed % 60;
	//compressed /= 60;
	uint16_t compressed = record->compressedTime;
	time->seconds = 0;
	time->minutes = compressed % 60;
	compressed /= 60;
	time->hours = compressed % 24;
	compressed /= 24;
	return compressed;
}

// Use "current" global values.
void storeToRecord(StoredPathRecord_t* record) {

	record->compressedBatteryVoltage = batteryVoltage * 50; // 5.12V->256
	record->compressedSolarVoltage = solarVoltage * 50;
	uint16_t gpsFixTime = lastGPSFixTime/2;
	if (gpsFixTime > 255) gpsFixTime = 255;
	record->speed = (uint8_t)speed_kts;

	record->lat = lastNonzeroPosition.lat;
	record->lon = lastNonzeroPosition.lon;
	record->alt = lastNonzero3DPosition.alt;

	record->simpleTemperature = simpleTemperature;
	record->compressedTime = compressRTCDatetime(); //compressTime(&GPSTime);
}

uint16_t uncompressBatteryVoltageTomV(StoredPathRecord_t* record) {
	return record->compressedBatteryVoltage * (1000/50);
}

uint16_t uncompressSolarVoltageTomV(StoredPathRecord_t* record) {
	return record->compressedSolarVoltage * (1000/50);
}
