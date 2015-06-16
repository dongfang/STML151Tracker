/*
 * APRSTransmitter.c
 * Control of APRS RF circuitry.
 *
 *  Created on: Jun 5, 2015
 *      Author: dongfang
 */

#include "APRS.h"
#include "DAC.h"
// Using a 26MHz xtal on the CDCEL913 today.
#include "CDCE913_26MHzXtal.h"

const uint32_t frequencies2m[] = APRS_FREQUENCIES_2M;
volatile APRSModulationMode_t currentMode;
volatile uint16_t packet_cnt;
volatile uint8_t packetTransmissionComplete;

const TransmitterSetting_t settingsFor2m_APRS[] = PLL_SETTINGS_2m_APRS;
const TransmitterSetting_t settingsFor30m_APRS = PLL_SETTING_30m_APRS;
const TransmitterSetting_t settingsForSi4463VCXO= PLL_SETTING_1_1_VCXO;

const APRS_Mode_t TWO_M_APRS_COMPRESSED_MESSAGE = {
		.DACChannel = DAC1,
		.modulationMode = AFSK,
		.modulationAmplitude = 1,
		.hardwareChannel = 1,
		// .buildMessage = aprs_compressedMessage,
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


const TransmitterSetting_t* transmitterSetting2M_DirectlyFromPLL(uint32_t frequency) {
	for (uint8_t i=0; i<sizeof(frequencies2m)/sizeof(uint32_t); i++) {
		if (frequency == frequencies2m[i])
			return settingsFor2m_APRS + i;
	}
	return 0;
}

void APRS_transmit2M_DirectlyFromPLL(uint32_t frequency, uint8_t hardwareChannel) {
	const TransmitterSetting_t* transmitterSetting = transmitterSetting2M_DirectlyFromPLL(frequency);
	CDCE913_init(hardwareChannel, transmitterSetting);
}

// Meaning: The CDCEL913 is used as a VCXO at a fixed frequency, which clocks an external tx circuit.
void APRS_transmit2M_PLL_With_ExternalTransmitter(uint32_t frequency, uint8_t hardwareChannel) {
	const TransmitterSetting_t* transmitterSetting = &settingsForSi4463VCXO;
	CDCE913_init(hardwareChannel, transmitterSetting);
}

void APRS_transmit30M_DirectlyFromPLL(uint32_t frequency, uint8_t hardwareChannel) {
	// Ignore the frequency param, we only support one.
	const TransmitterSetting_t* transmitterSetting = &settingsFor30m_APRS;
	CDCE913_init(hardwareChannel, transmitterSetting);
}
