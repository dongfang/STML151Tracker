/*
 * Globals.h
 *
 *  Created on: Jul 1, 2015
 *      Author: dongfang
 */

#ifndef INC_GLOBALS_H_
#define INC_GLOBALS_H_

#include "GPS.h"
#include "SelfCalibration.h"
#include <stdint.h>

extern float temperature;
extern int8_t simpleTemperature;
//extern float internalTemperature;
extern float batteryVoltage;
extern float solarVoltage;
extern uint8_t numRestarts;

extern NMEA_TimeInfo_t GPSTime;
extern NMEA_CRS_SPD_Info_t GPSCourseSpeed;
extern Location_t GPSPosition;
extern NMEA_StatusInfo_t GPSStatus;
extern Location_t lastNonzeroPosition;
extern Location_t lastNonzero3DPosition;
extern uint16_t lastGPSFixTime;
extern uint16_t lastWSPRWindowWaitTime;
extern boolean latestAPRSRegions[12]; 	 // 12 is sufficently large for the world map...
extern boolean latestAPRSCores[12];	 // 12 is sufficently large for the world map...
extern const CalibrationRecord_t* currentCalibration;

extern char scheduleName;

#endif /* INC_GLOBALS_H_ */
