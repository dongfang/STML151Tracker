/*
 * SimpleIRQDrivenWSPR.c
 *
 *  Created on: Mar 12, 2015
 *      Author: dongfang
 */
#include "stm32l1xx.h"
#include "WSPR.h"
#include "DAC.h"
#include "GPS.h"
#include "StabilizedOscillator.h"
#include "Bands.h"
#include "Globals.h"
#include "Systick.h"
#include "LED.h"
#include <diag/Trace.h>
#include "PLL.h"
#include "HFDriver.h"

static volatile float symbolModulation;
static volatile int16_t symbolNumber;
static uint8_t symbolNumberChangeDetect;
extern uint8_t getWSPRSymbol(uint8_t i);
static long startTime;

void setWSPR_DAC(uint8_t symbol) {
	uint16_t dacData = (uint16_t)(
			symbol * symbolModulation + 2048 - 1.5 * symbolModulation + 0.5);
	setDAC(DAC2, dacData);
}

// WSPR DAC interrupt handler.
// Could use DMA instead.
void TIM2_IRQHandler(void) {
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET) {
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
		if (symbolNumber < 162) {
			uint8_t symbol = getWSPRSymbol(symbolNumber);
			trace_printf("wspr symbol #%d %d\n", symbolNumber, systemTimeMillis - startTime);
			setWSPR_DAC(symbol);
			trace_putchar('0' +  symbol);
		}
		// We just assume there is no prob with overflowing... should be impossible.
		symbolNumber++;
	}
}

uint8_t WSPRDidUpdate() {
	if (symbolNumber != symbolNumberChangeDetect) {
		symbolNumberChangeDetect = symbolNumber;
		return 1;
	}
	return 0;
}

boolean WSPREnded() {
	// 1 is is beginning of 1st
	// 2 is end of 1st
	// 162 is beginning of 162th
	// 163 is end of 162th
	return symbolNumber == 163;
}

static void WSPR_initHW(
		uint8_t band,
		const PLL_Setting_t* setting,
		float _symbolModulation,
		HF_POWER_LEVEL power) {

	// Start a little early, because the osc takes some time to get a steady freq.
	symbolNumber = 0;
	symbolModulation = _symbolModulation;
	DAC2_initHW();
	setWSPR_DAC(getWSPRSymbol(0));
	// Set the osc to play the 0th symbol but without arming the tx
	setPLL((CDCE913_OutputMode_t) HF_30m_HARDWARE_OUTPUT, setting);

	// and do that for 1.75 sec (apparently we are always ahead!)
	startTime = systemTimeMillis;
	trace_printf("wspr pll start using powerlevel %d at systime %d\n", power, startTime);
	timer_sleep(1750);

	// Enable driver.
	HF_enableDriver(power);
	trace_printf("wspr powerup after %d\n", systemTimeMillis - startTime);

	/* Periph clocks enable */
	// RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

	// Set up timer.
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.TIM_Prescaler = 4861 - 1;
	TIM_TimeBaseStructure.TIM_Period = 2247 - 1; // 16M / (4861*2247) is right within 0.04ppm...
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);

	// Set up timer interrupt.
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	// We have some funny problem with a slow discharge somewhere in the modulator
	// which causes a ramping drift of a few Hz in WSPR.
	// Tie GPIO4 to ground, just to get rid of it.
	// -- didn't help.
	/*
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIOA->ODR &= ~(GPIO_Pin_4);
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	*/
	TIM_Cmd(TIM2, ENABLE);
}

static void WSPR_shutdownHW() {
	trace_printf("wspr shutdown after %d\n", systemTimeMillis - startTime);
	HF_shutdownDriver();
	PLL_shutdown();
	DAC_Cmd(DAC_Channel_2, DISABLE);
	TIM_ITConfig(TIM2, TIM_IT_Update, DISABLE);
	TIM_Cmd(TIM2, DISABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, DISABLE);
}

void WSPR_Transmit(
		uint8_t band,
		const PLL_Setting_t* setting,
		float symbolModulation) {

	WSPR_initHW(band, setting, symbolModulation, HF_power());

	while (!WSPREnded()) {
		if (WSPRDidUpdate()) {
			LED_PORT->ODR ^= LED_PORTBIT; // Flash LED
		}
		PWR_EnterSleepMode(PWR_Regulator_ON, PWR_SLEEPEntry_WFI);
	}

	LED_PORT->ODR &= ~LED_PORTBIT;	// Turn off LED.

	trace_printf("\n");
	WSPR_shutdownHW();
}

/*
 void WSPRBestSettings(WSPRBand_t band, uint32_t oscillatorFrequencyMeasured,
 uint8_t* bestSettingIndex, uint8_t* bestTrim) {
 double targetFrequency = HF_BAND_DEFS[band].frequency;

 double desiredMultiplication = (double) targetFrequency
 / oscillatorFrequencyMeasured;

 bestStoredPLLSetting(HF_BAND_DEFS[band].PLLOptions, HF_BAND_DEFS[band].numPLLOptions, desiredMultiplication,
 bestSettingIndex, bestTrim);
 }

 void WSPR_transmit(WSPRBand_t band, uint32_t oscillatorFrequencyMeasured, PLL_Setting_t* pllSetting, float stepModulation) {
 // WSPRBestSettings(band, oscillatorFrequencyMeasured, &bestSetting, &bestTrim);
 // trace_printf("Chose setting #%d with trim %d\n", bestSetting, bestTrim);

 WSPR_TransmitCycle(
 band,
 HF_BAND_DEFS[band].PLLOptions + bestSetting,
 bestTrim,
 stepModulation);
 }
 */

/*
 void WSPRSynthesisExperiment(uint32_t oscillatorFrequencyMeasured) {
 for (WSPRBand_t band = THIRTY_M; band <= TEN_M; band++) {
 trace_printf("Trying band %d\n", band);
 double targetFrequency = HF_BAND_DEFS[band].frequency;

 double desiredMultiplication = (double) targetFrequency
 / oscillatorFrequencyMeasured;

 PLLSettingExperiment(HF_BAND_DEFS[band].PLLOptions, HF_BAND_DEFS[band].numPLLOptions, desiredMultiplication);
 }
 }
 */
