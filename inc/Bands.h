/*
 * Bands.h
 *
 *  Created on: Jun 26, 2015
 *      Author: dongfang
 */

#ifndef INC_BANDS_H_
#define INC_BANDS_H_

#include "PLL.h"

/*
 * For transmitting on a frequency, which options are aviailable
 */
typedef struct {
	const uint8_t hardwareChannel;	// Which hardware output is used (implementation-specific interpretation)
	const uint8_t numPLLOptions;	// How many different PLL settings can we try using
	const PLL_Setting_t* PLLOptions;// The PLL settings (implementation-specific data format)
	const uint32_t frequency;		// WSPR frequency of band (the
} HF_BandDef_t;

extern const HF_BandDef_t HF_BAND_DEFS[];

/*
 * For transmitting VHF from the PLL (like CDCE913) directly.
 */
typedef struct {
	const PLL_Setting_t PLLSetting;	// We don't bother to maintain many options here. FM is not so critical
	const uint32_t frequency;		//
} VHF_ChannelDef_PLL_t;

extern const VHF_ChannelDef_PLL_t VHF_PLL_BAND_DEFS[];
extern const uint8_t NUM_VHF_PLL_BAND_DEFS;

/*
 * Using Si4463, there is no need to store PLL settings; the driver can set it.
 */
typedef uint32_t VHF_ChannelDef_Si6643_t;

extern const VHF_ChannelDef_Si6643_t VHF_SI4463_BAND_DEFS[];
extern const uint8_t NUM_VHF_SI4463_BAND_DEFS;

/*
 * For transmitting on a frequency, which options are aviailable
typedef struct {
	const uint8_t hardwareChannel;	// Which hardware output is used (implementation-specific interpretation)
	const uint8_t numFrequencies;	// How many different frequencies in the list
	const PLL_Setting_t* PLLOptions;// The PLL settings (implementation-specific data format)
	const uint32_t* frequencies;	// The frequencies
} VHF_Direct_PLL_BandDef_t;
 */


#endif /* INC_BANDS_H_ */
