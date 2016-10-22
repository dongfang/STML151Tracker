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
#include "stm32l1xx.h"
#include "Systick.h"
#include "IndividualBoard.h"
#include "LED.h"
#include <diag/trace.h>

volatile APRSModulationMode_t currentMode;
volatile uint16_t packet_cnt;
volatile uint8_t packetTransmissionComplete;

static void APRS_initDirectVHFTransmission(uint32_t frequency, uint32_t referenceFrequency);
static void APRS_endDirectTransmission();
static void APRS_initDirectHFTransmission(uint32_t frequency, uint32_t referenceFrequency);

const APRSTransmission_t APRS_TRANSMISSIONS[] = { { 
		.modulationMode = AFSK,
		.modulationAmplitude = APRS_FM_DEVIATION_2m,
		.txDelay = 16,
		 .initTransmitter = APRS_initDirectVHFTransmission,
		 .shutdownTransmitter = APRS_endDirectTransmission },
		 { .modulationMode = GFSK,
		.modulationAmplitude = HF_APRS_DEVIATION_DAC_30m,
		.txDelay = 10,
		.initTransmitter = APRS_initDirectHFTransmission,
		.shutdownTransmitter = APRS_endDirectTransmission } };

void APRS_initSi4463Transmission(
		uint32_t frequency,
		uint32_t referenceFrequency) {
	PLL_setXOPassthroughMode(PLL_PREFERRED_TRIM_VALUE);
	RF24_initHW();
	uint8_t power = 100;// isDaytimePower() ? SI4463_DAYTIME_TX_POWER : SI4463_NIGHTTIME_TX_POWER;
	RF24_transmit(frequency, power);
}

void APRS_endSi4463Transmission() {
	RF24_stopTransmitting();
	RF24_shutdownHW();
	PLL_shutdown();
}

void APRS_makeDirectTransmissionFrequency(
		uint32_t frequency,
		uint32_t referenceFrequency,
		CDCE913_OutputMode_t output) {
	PLL_Setting_t pllSetting;
	double maxError = 30E-6;
	if (PLL_bestPLLSetting(referenceFrequency, frequency, maxError,
			&pllSetting)) {
		trace_printf("Using N=%d, M=%d, trim=%d\n", pllSetting.N, pllSetting.M,
				pllSetting.trim);
		setPLL(output, &pllSetting);
	} else {
		trace_printf("Was not able to find a setting (weird)\n");
	}
}

static void APRS_initDirectHFTransmission(uint32_t frequency, uint32_t referenceFrequency) {
	HF_enableDriver(HF_power());
	APRS_makeDirectTransmissionFrequency(frequency, referenceFrequency, HF_30m_HARDWARE_OUTPUT);
}

static void APRS_initDirectVHFTransmission(uint32_t frequency, uint32_t referenceFrequency) {
	APRS_makeDirectTransmissionFrequency(frequency, referenceFrequency, DIRECT_2m_HARDWARE_OUTPUT);
}

void APRS_endDirectTransmission() {
	HF_shutdownDriver();
	PLL_shutdown();
}

extern void APRS_marshallPositionMessage(uint16_t txDelay);
extern void APRS_marshallStoredPositionMessage(StoredPathRecord_t* record,
		uint16_t txDelay);
extern void APRS_marshallStatusMessage(uint32_t frequency,
		uint32_t referenceFrequency, uint16_t txDelay);
// extern volatile uint16_t packet_size;

static void _APRS_transmitMessage(
		APRS_Band_t band,
		APRS_MessageType_t messageType,
		StoredPathRecord_t* storedMessage,
		uint32_t frequency,
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
		APRS_marshallStatusMessage(frequency, referenceFrequency, mode->txDelay);
		break;
	}

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
		PWR_startDevice(E_DEVICE_VHF_TX);
		break;
	case HF:
		PWR_startDevice(E_DEVICE_HF_TX);
		break;
	}

	GPIOB->ODR |= GPIO_Pin_1; // 3.3V on.
	LED_PORT->ODR |= LED_PORTBIT; // LED

	mode->initTransmitter(frequency, referenceFrequency);

	// Go now.
	packet_cnt = 0;
	packetTransmissionComplete = false;

	// ADC_DMA_init(ADC_MEASUREMENT_POWER_LOADED); 	// Experiment: Loaded voltages measurement.

	while (!packetTransmissionComplete) {
		PWR_EnterSleepMode(PWR_Regulator_ON, PWR_SLEEPEntry_WFI);
	}

	// We are now done transmitting.
	FSK_shutdown();

#if defined(TEST_VHF_FREQUENCY) && MODE == GROUNDTEST
	timer_sleep(2500);
#endif
	mode->shutdownTransmitter();

	LED_PORT->ODR &= ~(LED_PORTBIT);	// LED off

	// ADC_DMA_shutdown(); // we just assume it will work, if not, no prob.

	switch (band) {
	case VHF:
		PWR_stopDevice(E_DEVICE_VHF_TX);
		break;
	case HF:
		PWR_stopDevice(E_DEVICE_HF_TX);
		break;
	}

	trace_printf("Tx-end\n");
}

void APRS_transmitMessage(
		APRS_Band_t band,
		APRS_MessageType_t messageType,
		uint32_t frequency,
		uint32_t referenceFrequency) {
	_APRS_transmitMessage(
			band,
			messageType,
			(StoredPathRecord_t*) 0,
			frequency,
			referenceFrequency);
}

void APRS_transmitStoredMessage(
		APRS_Band_t band,
		StoredPathRecord_t* storedMessage,
		uint32_t frequency,
		uint32_t referenceFrequency) {
	_APRS_transmitMessage(
			band,
			STORED_POSITION_MESSAGE,
			storedMessage,
			frequency,
			referenceFrequency);
}
