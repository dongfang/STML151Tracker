#ifndef PHYSICS_H
#define PHYSICS_H

#include <stdint.h>

#define BATT_ADC_FACTOR (BATT_MEASURED_VOLTAGE / BATT_READ_ADC12)
#define REV_BATT_ADC_FACTOR (BATT_READ_ADC12 / BATT_MEASURED_VOLTAGE)

// what we need to multiply ADC's n value with to get volts.
#define SOLAR_ADC_FACTOR (VCC/4096.0)

#define BATT_ADC_2V7 (REV_BATT_ADC_FACTOR * 2.7)
#define BATT_ADC_3V0 (REV_BATT_ADC_FACTOR * 3.0)
#define BATT_ADC_3V3 (REV_BATT_ADC_FACTOR * 3.3)
#define BATT_ADC_3V7 (REV_BATT_ADC_FACTOR * 3.7)
#define BATT_ADC_4V2 (REV_BATT_ADC_FACTOR * 4.2)

#define COARSE(x) ((x)*16)
#define REV_COARSE(x) ((x)/16)

float PHY_batteryUnloadedVoltage();
float PHY_batteryLoadedVoltage();
float PHY_solarVoltage();

float PHY_temperature();
// float PHY_internalTemperature();

int8_t PHY_simpleTemperature(float temp);

#endif
