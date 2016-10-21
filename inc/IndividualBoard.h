/*
 * IndividualBoard.h
 *
 *  Created on: Jul 19, 2015
 *      Author: dongfang
 */

#ifndef INC_INDIVIDUALBOARD_H_
#define INC_INDIVIDUALBOARD_H_

#define BATT_ADC_FACTOR (2.5)
// (BATT_MEASURED_VOLTAGE / BATT_READ_ADC12)
// what we need to multiply ADC's n value with to get volts.
// #define SOLAR_ADC_FACTOR (VCC/4096.0)

// Implementation nominal xtal frequency
// This is assumed to be the real frequency, and this is again assumed to suffice in precision for VHF-only.
#define PLL_XTAL_DEFAULT_FREQUENCY (26000900)

// The only trim to use. There is no auto-trim for VHF-only.
#define PLL_PREFERRED_TRIM_VALUE 10

#define PLL_MIN_TRIM_INDEX_VALUE 2
#define PLL_MAX_TRIM_INDEX_VALUE 20
#define PLL_XTAL_TRIM_PP10M_VALUES {\
2851,2160,1677,1284,1004,752,557,373,238,109,0,-103,-178,-259,-321,-385,-436,-491,-537,-580,-621}
//2811,2167,1705,1315,1034,773,571,386,246,114,0,-107,-186,-269,-336,-403,-456,-510,-559,-604,-646 }
//2849,2197,1729,1332,1047,788,581,393,249,115,0,-106,-189,-274,-341,-411,-463,-520,-566,-615,-659}
#define WSPR_DEVIATION_DAC_30m 17
#define HF_APRS_DEVIATION_DAC_30m 2047
#define APRS_FM_DEVIATION_2m 250

#endif /* INC_INDIVIDUALBOARD_H_ */
