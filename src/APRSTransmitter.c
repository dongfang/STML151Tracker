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
#include "Power.h"
#include "Setup.h"
#include "ADC.h"
#include <diag/trace.h>

volatile APRSModulationMode_t currentMode;
volatile uint16_t packet_cnt;
volatile uint8_t packetTransmissionComplete;

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
		.modulationAmplitude = 350, .txDelay = 15, .initTransmitter =
				APRS_initDirectVHFTransmission, .shutdownTransmitter =
				APRS_endDirectTransmission }, { .modulationMode = GFSK,
		.modulationAmplitude = 3660, .txDelay = 5, .initTransmitter =
				APRS_initDirectHFTransmission, .shutdownTransmitter =
				APRS_endDirectTransmission } };

extern void APRS_marshallPositionMessage(uint16_t txDelay);
extern void APRS_marshallStoredPositionMessage(StoredPathRecord_t* record,
		uint16_t txDelay);
extern void APRS_marshallStatusMessage(uint32_t frequency,
		uint32_t referenceFrequency, uint16_t txDelay);
// extern volatile uint16_t packet_size;

void APRS_transmitMessage(APRS_Band_t band, APRS_MessageType_t messageType,
		StoredPathRecord_t* storedMessage, uint32_t frequency,
		uint32_t referenceFrequency) {

	const APRSTransmission_t* mode = &APRS_TRANSMISSIONS[band];

	switch (messageType) {
	case COMPRESSED_POSITION_MESSAGE:
		APRS_marshallPositionMessage(mode->txDelay);
		break;
	case STORED_POSITION_MESSAGE:
		APRS_marshallStoredPositionMessage(storedMessage, mode->txDelay);
		break;
	case STATUS_MESSAGE:
		APRS_marshallStatusMessage(frequency, referenceFrequency,
				mode->txDelay);
		break;
	}

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

	switch (band) {
	case VHF:
		PWR_startVHFTx();
		break;
	case HF:
		PWR_startHFTx();
		GPIOB->ODR |=  (1 << 1);		// arm HF
		ADC_DMA_init(ADCLoadedValues); 	// Experiment: Loaded voltages measurement.
		break;
	}
	GPIOB->ODR |= GPIO_Pin_6; // LED

	packet_cnt = 0;
	mode->initTransmitter(frequency, referenceFrequency);

	// Go now.
	packetTransmissionComplete = false;

	while (!packetTransmissionComplete) {
		PWR_EnterSleepMode(PWR_Regulator_ON, PWR_SLEEPEntry_WFI);
	}

	// We are now done transmitting.
	mode->shutdownTransmitter();

	GPIOB->ODR &= ~(GPIO_Pin_6|GPIO_Pin_1); // LED and HF arm

	switch (band) {
	case VHF:
		PWR_stopVHFTx();
		break;
	case HF:
		PWR_stopHFTx();
		ADC_DMA_shutdown(); // we just assume it will work, if not, no prob.
		break;
	}
}

void APRS_transmitRandomMessage(APRS_Band_t band,
		APRS_MessageType_t messageType, uint32_t frequency,
		uint32_t referenceFrequency) {
	APRS_transmitMessage(band, messageType, (StoredPathRecord_t*) 0, frequency,
			referenceFrequency);
}

void APRS_transmitStoredMessage(APRS_Band_t band,
		StoredPathRecord_t* storedMessage, uint32_t frequency,
		uint32_t referenceFrequency) {
	APRS_transmitMessage(band, STORED_POSITION_MESSAGE, (StoredPathRecord_t*) 0,
			frequency, referenceFrequency);
}
