/*
 * APRSTransmitter.c
 * Control of APRS RF circuitry.
 *
 *  Created on: Jun 5, 2015
 *      Author: dongfang
 */

#include "APRS.h"
#include "DAC.h"
#include "Bands.h"
#include "PLL.h"
#include "RF24Wrapper.h"
#include "Setup.h"
#include <diag/trace.h>

volatile APRSModulationMode_t currentMode;
volatile uint16_t packet_cnt;
volatile uint8_t packetTransmissionComplete;

/*
 const PLL_Setting_t* transmitterSetting2M_DirectlyFromPLL(uint32_t frequency) {
 for (uint8_t i=0; i<NUM_VHF_PLL_BAND_DEFS; i++) {
 if (frequency == VHF_PLL_BAND_DEFS[i].frequency)
 return &VHF_PLL_BAND_DEFS[i].PLLSetting;
 }
 return 0;
 }
 */

static void APRS_makeDirectTransmissionFrequency(uint32_t frequency,
		uint32_t referenceFrequency, uint8_t output) {
	PLL_Setting_t pllSetting;
	double maxError = 30E-6;
	if (PLL_bestPLLSetting(referenceFrequency, frequency, maxError,
			&pllSetting)) {
		trace_printf("Using N=%d, M=%d, trim=%d\n", pllSetting.N, pllSetting.M,
				pllSetting.trim);
		setPLL((CDCE913_OutputMode_t) output, &pllSetting);
	} else {
		trace_printf("Was not able to find a setting (weird)\n");
	}
}

static void APRS_initDirectHFTransmission(uint32_t frequency,
		uint32_t referenceFrequency) {
	APRS_makeDirectTransmissionFrequency(frequency, referenceFrequency,
	HF_30m_HARDWARE_OUTPUT);
}

static void APRS_initDirectVHFTransmission(uint32_t frequency,
		uint32_t referenceFrequency) {
	APRS_makeDirectTransmissionFrequency(frequency, referenceFrequency,
	DIRECT_2m_HARDWARE_OUTPUT);
}

void APRS_endDirectTransmission() {
	PLL_shutdown();
	FSK_shutdown();
}

void APRS_initSi4463Transmission(uint32_t frequency,
		uint32_t referenceFrequency) {
	double desiredTrim = (double) PLL_XTAL_NOMINAL_FREQUENCY
			/ (double) referenceFrequency - 1;
	int8_t trim = PLL_bestTrim(desiredTrim);
	PLL_setXOPassthroughMode(trim);
	RF24_initHW();
	RF24_initWarm(frequency, 10);
}

const APRSTransmission_t APRS_TRANSMISSIONS[] = { { .modulationMode = AFSK,
		.modulationAmplitude = 350, .initTransmitter =
				APRS_initDirectVHFTransmission, .shutdownTransmitter =
				APRS_endDirectTransmission }, { .modulationMode = GFSK,
		.modulationAmplitude = 3660, .initTransmitter =
				APRS_initDirectHFTransmission, .shutdownTransmitter =
				APRS_endDirectTransmission } };

extern void APRS_marshallStoredPositionMessage(StoredPathRecord_t* record);
extern void APRS_marshallPositionMessage();
// extern volatile uint16_t packet_size;

void APRS_transmitMessage(const APRSTransmission_t* mode,
		APRS_MessageType_t messageType, StoredPathRecord_t* storedMessage,
		uint32_t frequency, uint32_t referenceFrequency) {

	switch (messageType) {
	case COMPRESSED_POSITION_MESSAGE:
		APRS_marshallPositionMessage();
		break;
	case STORED_POSITION_MESSAGE:
		APRS_marshallStoredPositionMessage(storedMessage);
		break;
	case STATUS_MESSAGE:
		APRS_marshallStatusMessage(frequency, referenceFrequency);
		break;
	}

	// trace_printf("Internal length %d\n", packet_size);

	/* Avoid firing a transmission prematurely */
	packetTransmissionComplete = true;

	// Set up MPU hardware (DAC, timers, ...)
	switch (mode->modulationMode) {
	case GFSK:
		GFSK_init(mode->modulationAmplitude);
		break;
	case AFSK:
		AFSK_init(mode->modulationAmplitude);
		break;
	}

	packet_cnt = 0;
	mode->initTransmitter(frequency, referenceFrequency);

	// Go now.
	packetTransmissionComplete = false;

	// TODO: Sleepy-wait.
	while (!packetTransmissionComplete)
		;

	trace_printf("APRS done\n");

	// We are now done transmitting.
	mode->shutdownTransmitter();
}

void APRS_transmitRandomMessage(const APRSTransmission_t* mode,
		APRS_MessageType_t messageType, uint32_t frequency,
		uint32_t referenceFrequency) {
	APRS_transmitMessage(mode, messageType, (StoredPathRecord_t*) 0, frequency,
			referenceFrequency);
}

void APRS_transmitStoredMessage(const APRSTransmission_t* mode,
		StoredPathRecord_t* storedMessage, uint32_t frequency,
		uint32_t referenceFrequency) {
	APRS_transmitMessage(mode, STORED_POSITION_MESSAGE, (StoredPathRecord_t*) 0,
			frequency, referenceFrequency);
}
