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
#define PLL_XTAL_DEFAULT_FREQUENCY 24576425

// The only trim to use. There is no auto-trim for VHF-only.
#define PLL_PREFERRED_TRIM_VALUE 10

//#4
#define BATT_MEASURED_VOLTAGE 4.033
#define BATT_READ_ADC12 3502.0
#define VCC 2.198

#define PLL_MIN_TRIM_INDEX_VALUE 4
#define PLL_MAX_TRIM_INDEX_VALUE 20

#define PLL_XTAL_TRIM_PP10M_VALUES { \
2387,1781,1369,1041,806,601,441,296,187,85,0,\
	-78,-137,-200,-251,-301,-339,-379,-415,-448,-478}

// For #1 board.
#define WSPR_DEVIATION_DAC_30m 27
#define HF_APRS_DEVIATION_DAC_30m 3669

#endif /* INC_INDIVIDUALBOARD_H_ */
