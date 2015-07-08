/*
 * SimpleIRQDrivenWSPR.c
 *
 *  Created on: Mar 12, 2015
 *      Author: dongfang
 */
#include "stm32l1xx_conf.h"
#include "WSPR.h"
#include "DAC.h"
#include "GPS.h"
#include "StabilizedOscillator.h"
#include "Bands.h"
#include "Globals.h"
#include <diag/Trace.h>
#include "../inc/PLL.h"

static volatile float symbolModulation;
static volatile uint8_t symbolNumber;
static uint8_t symbolNumberChangeDetect;
extern uint8_t getWSPRSymbol(uint8_t i);

// WSPR DAC interrupt handler.
// Could use DMA instead.
void TIM2_IRQHandler(void) {
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET) {
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
		if (symbolNumber < 162) {
			uint8_t symbol = getWSPRSymbol(symbolNumber);
			uint16_t dacData = (uint16_t)(
					symbol * symbolModulation + 2048 - 1.5 * symbolModulation
							+ 0.5);
			setDAC(DAC2, dacData);
			trace_putchar('0' + symbol);
		}

		if (symbolNumber < 163) {
			symbolNumber++;
		}
	}
}

uint8_t WSPRDidUpdate() {
	if (symbolNumber != symbolNumberChangeDetect) {
		symbolNumberChangeDetect = symbolNumber;
		return 1;
	}
	return 0;
}

uint8_t WSPREnded() {
	return symbolNumber >= 163;
}

static void WSPR_shutdownHW() {
	DAC_Cmd(DAC_Channel_2, DISABLE);
	TIM_ITConfig(TIM2, TIM_IT_Update, DISABLE);
	TIM_Cmd(TIM2, DISABLE);
	PLL_shutdown();
}

static void WSPR_initHW(uint8_t band, const PLL_Setting_t* setting,
		float _symbolModulation) {

	symbolNumber = 0;
	symbolModulation = _symbolModulation;

	setPLL((CDCE913_OutputMode_t) HF_30m_HARDWARE_OUTPUT, setting);

	/* Periph clocks enable */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
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
	GPIO_InitTypeDef GPIO_InitStructure;
	/* Configure PA.04 (DAC_OUT1), PA.05 (DAC_OUT2) as analog */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIOA->ODR &= ~(GPIO_Pin_4);
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	DAC2_initHW();
	setDAC(DAC2, 2048);

	TIM_Cmd(TIM2, ENABLE);
}

void WSPR_Transmit(uint8_t band, const PLL_Setting_t* setting, float stepModulation) {

	// Just to be sure.. The GPS makes too much supply noise for WSPR
	GPS_kill();

	GPIOB->ODR |= GPIO_Pin_1;			// Turn on arm feature.

	WSPR_initHW(band, setting, stepModulation);

	while (!WSPREnded()) {
		if (WSPRDidUpdate()) {
			GPIOB->ODR ^= GPIO_Pin_6; // Flash LED
		}
		PWR_EnterSleepMode(PWR_Regulator_ON, PWR_SLEEPEntry_WFI);
	}

	GPIOB->ODR &= ~(GPIO_Pin_1 | GPIO_Pin_6);	// Turn off LED and arm feature.
//	GPIOB->ODR &= ~(1 << 1); 		// Turn off arm feature.

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
