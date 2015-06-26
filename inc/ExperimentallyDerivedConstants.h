/*
 * ExperimentallyDerivedConstants.h
 *
 *  Created on: Jun 26, 2015
 *      Author: dongfang
 */

#ifndef INC_EXPERIMENTALLYDERIVEDCONSTANTS_H_
#define INC_EXPERIMENTALLYDERIVEDCONSTANTS_H_

#define PLL_PREFERRED_TRIM_VALUE 13


// Derived experimentally. The values are relative to CDCE913_PREFERRED_TRIM_VALUE,
// which was pretty much in center of the useful range.
#define PLL_XTAL_TRIM_PP10M_VALUES {\
		3042,\
		2410,\
		1951,\
		1566,\
		1289,\
		1037,\
		837,\
		654,\
		515,\
		382,\
		269,\
		164,\
		83,\
		0,\
		-66,\
		-134,\
		-187,\
		-243,\
		-290,\
		-337,\
		-379,\
}

#define PLL_MAX_TRIM_INDEX_VALUE 20
#define PLL_MIN_TRIM_INDEX_VALUE 3

#endif /* INC_EXPERIMENTALLYDERIVEDCONSTANTS_H_ */
