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
#define PLL_XTAL_DEFAULT_FREQUENCY 16000000

// The only trim to use. There is no auto-trim for VHF-only.
#define PLL_PREFERRED_TRIM_VALUE 9

#define PLL_MIN_TRIM_INDEX_VALUE 2
#define PLL_MAX_TRIM_INDEX_VALUE 20

#define PLL_XTAL_TRIM_PP10M_VALUES { \
1617,1214,934,706,537,388,268,162,78,0,-66,-126,-176,-225,-264,-303,-335,-366,-395,-421,-446}

#define WSPR_DEVIATION_DAC_30m 28
#define HF_APRS_DEVIATION_DAC_30m 3793
#define APRS_FM_DEVIATION_2m 500

#endif /* INC_INDIVIDUALBOARD_H_ */
