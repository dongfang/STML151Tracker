/*
 * Power.c
 *
 *  Created on: Jun 27, 2015
 *      Author: dongfang
 */

#include "RecordStorage.h"
#include "Power.h"

// Safe if : Never turned on, if turned back off by program, or if voltage is better now than last time.
boolean isSafeToUseGPS(float startupVoltage) {
	return !checkStartupRecordValid() || (!startupLog.wasGPSRunning ||
			startupVoltage > startupLog.initialVoltageValue);
}

boolean isSafeToUseHFTx(float startupVoltage) {
	return !checkStartupRecordValid() || (!startupLog.wasHFTxRunning ||
			startupVoltage > startupLog.initialVoltageValue);
}

boolean isSafeToUseVHFTx(float startupVoltage) {
	return !checkStartupRecordValid() || (!startupLog.wasVHFTxRunning ||
			startupVoltage > startupLog.initialVoltageValue);
}

void startGPS(float startupVoltage) {
	startupLog.wasGPSRunning = true;
	startupLog.wasGPSSuccessful = false;
	startupLog.initialVoltageValue = startupVoltage;
	setStartupRecordChecksum();
}

void startHFTx(float startupVoltage) {
	startupLog.wasHFTxRunning = true;
	startupLog.wasHFTxSuccessful = false;
	startupLog.initialVoltageValue = startupVoltage;
	setStartupRecordChecksum();
}

void startVHFTx(float startupVoltage) {
	startupLog.wasVHFTxRunning = true;
	startupLog.wasVHFTxSuccessful = false;
	startupLog.initialVoltageValue = startupVoltage;
	setStartupRecordChecksum();
}

void stopGPS() {
	startupLog.wasGPSRunning = false;
	startupLog.wasGPSSuccessful = true;
	setStartupRecordChecksum();
}

void stopHFTx() {
	startupLog.wasHFTxRunning = false;
	startupLog.wasHFTxSuccessful = true;
	setStartupRecordChecksum();
}

void stopVHFTx() {
	startupLog.wasVHFTxRunning = false;
	startupLog.wasVHFTxSuccessful = true;
	setStartupRecordChecksum();
}

