/*
 * SimpleIRQDrivenWSPR.c
 *
 *  Created on: Mar 12, 2015
 *      Author: dongfang
 */
#include "stm32l1xx_conf.h"
#include "WSPR.h"
#include "DAC.h"
#include "PLL.h"
#include "StabilizedOscillator.h"
#include "Bands.h"
#include <diag/Trace.h>
#include "../inc/CDCE913.h"

static volatile float symbolModulation;
static volatile uint8_t symbolNumber;
static uint8_t symbolNumberChangeDetect;
extern uint8_t getWSPRSymbol(uint8_t i);

// WSPR DAC interrupt handler.
// Could use DMA instead.
void TIM2_IRQHandler(void) {
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET) {
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
		uint8_t symbol = getWSPRSymbol(symbolNumber);
		uint16_t dacData = (uint16_t)(
				symbol * symbolModulation + 2048
						- 1.5 * symbolModulation + 0.5);

		setDAC(DAC2, dacData);

		if (symbolNumber < 162) {
			symbolNumber++;
		} else {
			trace_printf("WSPR end\n");
		}
		trace_putchar('0' + symbol);
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
	return symbolNumber >= 162;
}

void WSPR_shutdownHW() {
	DAC_Cmd(DAC_Channel_2, DISABLE);
	TIM_ITConfig(TIM2, TIM_IT_Update, DISABLE);
	TIM_Cmd(TIM2, DISABLE);
	CDCE913_shutdown();
}

void WSPR_initHW(
		uint8_t band,
		const PLL_Setting_t* setting,
		uint8_t trim,
		float _symbolModulation) {
	symbolNumber = 0;
	symbolModulation = _symbolModulation;

	// TODO: Use derived trim.
	CDCE913_setPLL((CDCE913_OutputMode_t)1, setting, trim);
	trace_printf("PLL running\n");

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

	DAC2_initHW();

	TIM_Cmd(TIM2, ENABLE);
}

void WSPR_TransmitCycle(
		uint8_t band,
		const PLL_Setting_t* setting,
		uint8_t trim,
		float stepModulation) {

	WSPR_initHW(
			band,
			setting,
			trim,
			stepModulation);

	trace_printf("Timer and IRQ running\n");

	while (!WSPREnded()) {
		if (WSPRDidUpdate()) {
			GPIOB->ODR ^= (1 << 6);
		}
	}

	WSPR_shutdownHW();
}

void WSPRSynthesisExperiment(uint32_t oscillatorFrequencyMeasured) {
	for (WSPRBand_t band = THIRTY_M; band <= TEN_M; band++) {
		trace_printf("Trying band %d\n", band);
		double targetFrequency = HF_BAND_DEFS[band].frequency;

		double desiredMultiplication = (double) targetFrequency
				/ oscillatorFrequencyMeasured;

		PLLSettingExperiment(HF_BAND_DEFS[band].PLLOptions, HF_BAND_DEFS[band].numPLLOptions, desiredMultiplication);
	}
}
