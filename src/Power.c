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
			// && temperature >= BATTERY_FAILSAFE_ALWAYS_SAFE_TEMPERATURE;
			&& solarVoltage >= BATTERY_FAILSAFE_ALWAYS_SAFE_SOLAR_VOLTAGE;
}

boolean PWR_isSafeToUseDevice(E_DEVICE device) {
#if defined(SIMPLE_BROWNOUT_MODE)
	// Do not use a device log. Just make an xx hours sleep penalty after any reset.
	// Therefore, the device log test here never has any objections.
	return true;
#else
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
#endif
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
