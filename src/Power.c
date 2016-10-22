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
#include "stm32l1xx.h"
#include "Globals.h"

boolean isDaytimePower() {
	return batteryVoltage >= DAY_MODE_VOLTAGE
			&& solarVoltage >= DAY_MODE_SOLAR_VOLTAGE;
}

boolean PWR_isSafeToUseDevice(E_DEVICE device) {
	switch (device) {
	case E_DEVICE_GPS:
		return batteryVoltage >= 3.8;
	case E_DEVICE_HF_TX:
		return batteryVoltage >= 3.9;
	case E_DEVICE_VHF_TX:
		return batteryVoltage >= 2.8;
	}
	return true;
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

HF_POWER_LEVEL HF_power() {
	HF_POWER_LEVEL power;
	if (batteryVoltage >= 4.2)
		power = FOUR_DRIVERS;
	else if (batteryVoltage >= 4)
		power = TWO_DRIVERS;
	else
		power = ONE_DRIVER;
	return power;
}

void HF_enableDriver(HF_POWER_LEVEL power) {
	GPIOB->ODR &= ~GPIO_Pin_14; // botch transistor
	switch (power) {
	case FOUR_DRIVERS:
		GPIOB->ODR |= GPIO_Pin_6;
		// no break here!
	case TWO_DRIVERS:
		GPIOB->ODR |= GPIO_Pin_4;
		// no break here!
	case ONE_DRIVER:
		GPIOB->ODR |= GPIO_Pin_5;
		// no break here!
	}
}

void HF_shutdownDriver() {
	GPIOB->ODR &= ~(GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6);
	GPIOB->ODR |= GPIO_Pin_14;
}
