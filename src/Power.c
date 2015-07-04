/*
 * Power.c
 *
 *  Created on: Jun 27, 2015
 *      Author: dongfang
 */

#include "RecordStorage.h"
#include "Power.h"
#include "Setup.h"
#include "Globals.h"

// Safe if : Never turned on, if turned back off by program, or if voltage is better now than last time.
boolean isSafeToUseEquipment(boolean previouslyCrashed) {
	// No history means no objections.
	if (!checkStartupRecordValid()) return true;

	if (batteryVoltage >= BATTERY_FAILSAFE_ALWAYS_SAFE_VOLTAGE
			&& simpleTemperature >= BATTERY_FAILSAFE_ALWAYS_SAFE_TEMPERATURE)
		return true;

	// Might have crashed, but now voltage is significantly better.
	if (batteryVoltage >= startupLog.initialVoltageValue)
		return true;

	return !previouslyCrashed;
}

// Safe if : Never turned on, if turned back off by program, or if voltage is better now than last time.
boolean PWR_isSafeToUseGPS() {
	return isSafeToUseEquipment(startupLog.wasGPSRunning);
}

boolean PWR_isSafeToUseHFTx() {
	return isSafeToUseEquipment(startupLog.wasHFTxRunning);
}

boolean PWR_isSafeToUseVHFTx() {
	return isSafeToUseEquipment(startupLog.wasVHFTxRunning);
}

void PWR_startGPS() {
	startupLog.wasGPSRunning = true;
	startupLog.wasGPSSuccessful = false;
	startupLog.initialVoltageValue = batteryVoltage;
	setStartupRecordChecksum();
}

void PWR_startHFTx() {
	startupLog.wasHFTxRunning = true;
	startupLog.wasHFTxSuccessful = false;
	startupLog.initialVoltageValue = batteryVoltage;
	setStartupRecordChecksum();
}

void PWR_startVHFTx() {
	startupLog.wasVHFTxRunning = true;
	startupLog.wasVHFTxSuccessful = false;
	startupLog.initialVoltageValue = batteryVoltage;
	setStartupRecordChecksum();
}

void PWR_stopGPS() {
	startupLog.wasGPSRunning = false;
	startupLog.wasGPSSuccessful = true;
	setStartupRecordChecksum();
}

void PWR_stopHFTx() {
	startupLog.wasHFTxRunning = false;
	startupLog.wasHFTxSuccessful = true;
	setStartupRecordChecksum();
}

void PWR_stopVHFTx() {
	startupLog.wasVHFTxRunning = false;
	startupLog.wasVHFTxSuccessful = true;
	setStartupRecordChecksum();
}

