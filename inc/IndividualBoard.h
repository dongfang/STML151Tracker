/*
 * IndividualBoard.h
 *
 *  Created on: Jul 19, 2015
 *      Author: dongfang
 */

#ifndef INC_INDIVIDUALBOARD_H_
#define INC_INDIVIDUALBOARD_H_

#include <stdint.h>

// Implementation nominal xtal frequency
// This is assumed to be the real frequency, and this is again assumed to suffice in precision for VHF-only.
#define PLL_XTAL_DEFAULT_FREQUENCY 24576100

// The only trim to use. There is no auto-trim for VHF-only.
#define PLL_PREFERRED_TRIM_VALUE 10

//#4
#define BATT_MEASURED_VOLTAGE 4.002
#define BATT_READ_ADC12 3598.0
#define VCC 2.074

//#1
//#define BATT_MEASURED_VOLTAGE 3.999
//#define BATT_READ_ADC12 3697
//#define VCC 2.059

#define PLL_MIN_TRIM_INDEX_VALUE 4
#define PLL_MAX_TRIM_INDEX_VALUE 20

#define PLL_XTAL_TRIM_PP10M_VALUES { \
1663,1273,997,762,596,442,326,220,140,63,0,-57,-104,-150,-187,-225,-251,-283,-309,-333,-357}
// #1
//	1930,1430,1093,825,635,471,344,229,146,67,0,-62,-103,-153,-188,-229,-256,-288,-314,-340,-361}


// For #4 board
#define WSPR_DEVIATION_DAC_30m 33
#define HF_APRS_DEVIATION_DAC_30m 4095

// For #1 board.
//#define WSPR_DEVIATION_DAC_30m 29
//#define HF_APRS_DEVIATION_DAC_30m 3982

#endif /* INC_INDIVIDUALBOARD_H_ */
