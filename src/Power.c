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

boolean isDaytimePower() {
	return batteryVoltage >= BATTERY_FAILSAFE_ALWAYS_SAFE_VOLTAGE
			&& temperature >= BATTERY_FAILSAFE_ALWAYS_SAFE_TEMPERATURE;
}

/*
 // Safe if : Never turned on, if turned back off by program, or if voltage is better now than last time.
 boolean isSafeToUseEquipment(boolean previouslyCrashed) {
 // No history means no objections.
 if (!checkStartupRecordValid()) return true;

 if (batteryVoltage >= BATTERY_FAILSAFE_ALWAYS_SAFE_VOLTAGE
 && simpleTemperature >= BATTERY_FAILSAFE_ALWAYS_SAFE_TEMPERATURE)
 return true;

 // Might have crashed, but now voltage is significantly better.
 if (batteryVoltage >= startupLog.initialVoltageValue + BATTERY_FAILSAFE_VOLTAGE_DELTA)
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
*/

boolean PWR_isSafeToUseDevice(E_DEVICE device) {
	// If conditions are must-be-good, accept.
	if (isDaytimePower())
		return true;

	// No history means no objections.
	if (!checkStartupRecordValid(device))
		return true;

	// Might have crashed, but now voltage or temperature is significantly better.
	if ((batteryVoltage
			>= startupLog[device].initialVoltage
					+ BATTERY_FAILSAFE_VOLTAGE_DELTA)
			|| (simpleTemperature
					>= startupLog[device].initialTemperature
							+ BATTERY_FAILSAFE_TEMPERATURE_DELTA))
		return true;

	return !startupLog[device].wasDeviceRunning;
}

void PWR_startDevice(E_DEVICE device) {
	startupLog[device].wasDeviceRunning = true;
	startupLog[device].wasDeviceSuccesful = false;
	startupLog[device].initialVoltage = batteryVoltage;
	startupLog[device].initialTemperature = simpleTemperature;
	setStartupRecordChecksum(device);
}

void PWR_stopDevice(E_DEVICE device) {
	startupLog[device].wasDeviceRunning = false;
	startupLog[device].wasDeviceSuccesful = true;
	setStartupRecordChecksum(device);
}
