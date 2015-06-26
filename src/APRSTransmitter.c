/*
 * APRSTransmitter.c
 * Control of APRS RF circuitry.
 *
 *  Created on: Jun 5, 2015
 *      Author: dongfang
 */

#include "APRS.h"
#include "DAC.h"
#include "PLL.h"

const uint32_t frequencies2m[] = APRS_FREQUENCIES_2M;
volatile APRSModulationMode_t currentMode;
volatile uint16_t packet_cnt;
volatile uint8_t packetTransmissionComplete;

const APRS_Mode_t TWO_M_APRS_COMPRESSED_MESSAGE = {
		.DACChannel = DAC1,
		.modulationMode = AFSK,
		.modulationAmplitude = 1,
		.hardwareChannel = 1,
};

const APRS_Mode_t TWO_M_APRS_STATUS_MESSAGE = {
		.DACChannel = DAC1,
		.modulationMode = AFSK,
		.modulationAmplitude = 1,
		.hardwareChannel = 1,
		// .buildMessage = &aprs_statusMessage
};

const APRS_Mode_t HF_APRS_COMPRESSED_MESSAGE = {
		.DACChannel = DAC1,
		.modulationMode = GFSK,
		.modulationAmplitude = 0.1,
		.hardwareChannel = 2,
		// .buildMessage = &aprs_compressedMessage
};


const PLL_Setting_t* transmitterSetting2M_DirectlyFromPLL(uint32_t frequency) {
	for (uint8_t i=0; i<NUM_PLL_OPTIONS_APRS_DIRECT_2m; i++) {
		if (frequency == frequencies2m[i])
			return PLL_OPTIONS_APRS_DIRECT_2m + i;
	}
	return 0;
}

/*
void APRS_transmit2M_DirectlyFromPLL(CDCE913_OutputMode_t output, uint32_t frequency) {
	const PLLSetting_t* transmitterSetting = transmitterSetting2M_DirectlyFromPLL(frequency);
	CDCE913_init(output, CDCE913_PREFERRED_TRIM, transmitterSetting);
}

// Meaning: The CDCEL913 is used as a VCXO at a fixed frequency, which clocks an external tx circuit.
void APRS_transmit2M_PLL_With_ExternalTransmitter(CDCE913_OutputMode_t output, uint32_t frequency) {
	const PLLSetting_t* transmitterSetting = &settingsForSi4463VCXO;
	CDCE913_init(output, CDCE913_PREFERRED_TRIM, transmitterSetting);
}

void APRS_transmit30M_DirectlyFromPLL(CDCE913_OutputMode_t output, uint32_t frequency) {
	// Ignore the frequency param, we only support one.
	const PLLSetting_t* transmitterSetting = &settingsFor30m_APRS;
	CDCE913_init(output, CDCE913_PREFERRED_TRIM, transmitterSetting);
}
*/
