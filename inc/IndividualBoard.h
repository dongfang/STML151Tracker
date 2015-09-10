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
#define PLL_XTAL_NOMINAL_FREQUENCY 24576000
#define PLL_XTAL_DEFAULT_FREQUENCY 24576100

// The only trim to use. There is no auto-trim for VHF-only.
#define PLL_PREFERRED_TRIM_VALUE 15

#define BATT_MEASURED_VOLTAGE 4.038
#define BATT_READ_ADC12 3643.0

#define PLL_MIN_TRIM_INDEX_VALUE 5
#define PLL_MAX_TRIM_INDEX_VALUE 18

#define PLL_XTAL_TRIM_PP10M_VALUES {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20}
#define WSPR_DEVIATION_DAC_30m 23
#define HF_APRS_DEVIATION_DAC_30m 400

#define VCC 2.059
#endif /* INC_INDIVIDUALBOARD_H_ */
