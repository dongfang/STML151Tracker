/*
 * Power.c
 *
 *  Created on: Jun 27, 2015
 *      Author: dongfang
 */

#include "RecordStorage.h"
#include "Power.h"

// Safe if : Never turned on, if turned back off by program, or if voltage is better now than last time.
boolean isSafeToUseGPS(uint8_t startupVoltage) {
	return !checkStartupRecordValid() || (!startupLog.wasGPSRunning ||
			startupVoltage > startupLog.initialVoltageValue);
}

boolean isSafeToUseWSPR(uint8_t startupVoltage) {
	return !checkStartupRecordValid() || (!startupLog.wasWSPRRunning ||
			startupVoltage > startupLog.initialVoltageValue);
}

boolean isSafeToUseSi4463(uint8_t startupVoltage) {
	return !checkStartupRecordValid() || (!startupLog.wasSi4463Running ||
			startupVoltage > startupLog.initialVoltageValue);
}

void startGPS(uint8_t startupVoltage) {
	startupLog.wasGPSRunning = true;
	startupLog.wasGPSSuccessful = false;
	startupLog.initialVoltageValue = startupVoltage;
	setStartupRecordChecksum();
}

void startWSPR(uint8_t startupVoltage) {
	startupLog.wasWSPRRunning = true;
	startupLog.wasWSPRSuccessful = false;
	startupLog.initialVoltageValue = startupVoltage;
	setStartupRecordChecksum();
}

void startSi4463(uint8_t startupVoltage) {
	startupLog.wasSi4463Running = true;
	startupLog.wasSi4463Successful = false;
	startupLog.initialVoltageValue = startupVoltage;
	setStartupRecordChecksum();
}

void stopGPS() {
	startupLog.wasGPSRunning = false;
	startupLog.wasGPSSuccessful = true;
	setStartupRecordChecksum();
}

void stopWSPR() {
	startupLog.wasWSPRRunning = false;
	startupLog.wasWSPRSuccessful = true;
	setStartupRecordChecksum();
}

void stopSi4463() {
	startupLog.wasSi4463Running = false;
	startupLog.wasSi4463Successful = true;
	setStartupRecordChecksum();
}

