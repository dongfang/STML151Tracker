/*
 * ADCPhysics.c

 *
 *  Created on: Aug 26, 2015
 *      Author: dongfang
 */

#include "Setup.h"
#include "Globals.h"
#include <stdint.h>

// For ground test, see the DummyPhysics source.
#if MODE==GROUNDTEST

#include "Physics.h"
#include "RTC.h"
#include "GPS.h"
#include <diag/trace.h>
#include <math.h>

// #define OPTIMISTIC

static int secondsIntoFlight() {
	Time_t time;
	RTC_getTime(&time);
	int secondsIntoFlight = secondsOfDay(&time);
	return secondsIntoFlight;
}

float daynight() {
	float day;
	int timeSeconds = secondsIntoFlight();
	float hours = timeSeconds / 3600.0;
	if (hours >= 6 && hours < 18) {
		if (hours < 7) { // dawn
			day = hours - 6; // ramp from 0 at 6 to 1 at 7
		} else {
			day = 1; // day
		}
	} else if (hours >= 18) {
		if (hours < 19) { // dusk
			day = 19 - hours; // ramp from 1 at 18 to 0 at 19
		} else {
			day = 0; // night
		}
	} else {
		day = 0; // night
	}
	return day;
}

#ifdef OPTIMISTIC
float PHY_temperature() {
	return 20;
}
float PHY_batteryUnloadedVoltage() {
	return 4.1;
}
float PHY_batteryLoadedVoltage() {
	return 4.0;
}
#else

float PHY_temperature() {
	// Night: 15C + (-50 - 15)/10000 * altitude.
	// Day:   15 degrees warmer.
	float temp = 15.0f
			+ (-50 - 15) / 10000.0f * lastNonzero3DPosition.alt;
	float day = daynight();
	temp = temp + day * 15.0f;
	return temp;
}

float PHY_batteryUnloadedVoltage() {
	/*
	float temperature = PHY_temperature();
	int timeSeconds = secondsIntoFlight();
	float hours = timeSeconds / 3600.0f;
	float charge;
	if (hours >= 6 && hours < 18) {
		if (hours < 9) { // dawn
			charge = (9-hours) / 3; // ramp from 0 at 6 to 1 at 12
		} else {
			charge = 1; // day
		}
	} else if (hours >= 18) {
		charge = (hours - 18) / 12; // ramp from 1 at 18 to 0.5 at 24
	} else {
		charge = (6 - hours) / 12; // ramp from 0.5 at 0 to 0 at 6;
	}

	// Simulate some drop at low temp.
	charge = 3.1 + charge * (4.2 - 3.1) + (temperature - 20) * 0.005;
	// trace_printf("Fake batt volt. mV %d\n", (int)(charge*1000));
	return charge;
	*/
	return 4.2;
}

void ADC_ensureVDDMeasured() {}

float PHY_batteryLoadedVoltage() {
	float temperature = PHY_temperature();
	float voltage = PHY_batteryUnloadedVoltage();
	// Simulate some extra voltage drop.
	return voltage + (temperature - 20) * 0.01;
}

#endif

float PHY_solarVoltage() {
	/*
	float day = daynight();
	return day * 0.7;
	*/
	return 0.7;
}

float PHY_internalTemperature() {
	return PHY_temperature();
}

#endif
