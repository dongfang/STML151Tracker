/*
 * SelfCalibration.c
 *
 *  Created on: May 26, 2015
 *      Author: dongfang
 */
#include "stm32l1xx_conf.h"
#include "SelfCalibration.h"
#include "systick.h"
#include "CDCEL913.h"
#include <diag/trace.h>

#include "DAC.h"
/**
 * @brief  Configures the Timer.
 * @param  None
 * @retval None
 */
static void modulationCalibration_Config() {
	TIM_ICInitTypeDef TIM_ICInitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	/* TIM4 clock enable */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

	/* GPIOB clock enable */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);

	/* TIM4 channel 2 pin (PB.07) configuration */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_TIM4);

	/* TIM4 configuration: Input Capture mode ---------------------
	 The external signal is connected to TIM4 CH2 pin (PB.07)
	 The Rising edge is used as active edge,
	 The TIM4 CCR2 is used to compute the frequency value
	 ------------------------------------------------------------ */
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV8;
	TIM_ICInitStructure.TIM_ICFilter = 0x0;
	TIM_ICInit(TIM4, &TIM_ICInitStructure);

	/* TIM enable counter */
	TIM_Cmd(TIM4, ENABLE);

	/* Enable the CC2 Interrupt Request */
	TIM_ITConfig(TIM4, TIM_IT_CC2, ENABLE);
	TIM_ClearITPendingBit(TIM4, TIM_IT_CC2);

	/* Enable the TIM4 global Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

static void modulationCalibration_Shutdown() {
	NVIC_InitTypeDef NVIC_InitStructure;

	/* Disable the TIM4 global Interrupt, NVIC end. */
	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* Disable the CC2 Interrupt Request (timer end) */
	TIM_ITConfig(TIM4, TIM_IT_CC2, DISABLE);

	/* TIM disable counter */
	TIM_Cmd(TIM4, DISABLE);

	/* TIM4 clock disable */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, DISABLE);

	/* GPIOB clock disable (we assume that next user will enable again?
	 * Else we need to save old settings and re-apply. Slower clock probably.
	 */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, DISABLE);
}

void oscillatorCalibration_Config() {
	TIM_ICInitTypeDef TIM_ICInitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM10, ENABLE);

	// Jumper wired GPS timepulse
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz; // More was not better.
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_PinAFConfig(GPIOB, GPIO_PinSource12, GPIO_AF_TIM10);

	/* TIM10 configuration: Input Capture mode ---------------------
	 The external signal is connected to TIM10 CH1 pin (PB.12)
	 The Rising edge is used as active edge,
	 The TIM10 CCR2 is used to compute the frequency value
	 ------------------------------------------------------------ */
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	TIM_ICInitStructure.TIM_ICFilter = 0x4;
	TIM_ICInit(TIM10, &TIM_ICInitStructure);

	// Just to be sure ..
	TIM_TimeBaseInitTypeDef timerInitStructure;
	timerInitStructure.TIM_Prescaler = 0;
	timerInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	timerInitStructure.TIM_Period = 60000 - 1; // why does 0 not work??
	timerInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInit(TIM10, &timerInitStructure);

	/* TIM enable counter */
	TIM_Cmd(TIM10, ENABLE);

	/* Enable the CC1 Interrupt Request */
	TIM_ITConfig(TIM10, TIM_IT_CC1, ENABLE);
	TIM_ITConfig(TIM10, TIM_IT_Update, ENABLE);

	TIM_ClearITPendingBit(TIM10, TIM_IT_CC1);
	TIM_ClearITPendingBit(TIM10, TIM_IT_Update);

	/* Enable the TIM10 global Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = TIM10_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

static void oscillatorCalibration_Shutdown() {
	NVIC_InitTypeDef NVIC_InitStructure;

	/* Disable the TIM10 global Interrupt, NVIC end. */
	NVIC_InitStructure.NVIC_IRQChannel = TIM10_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* Disable the CC2 Interrupt Request (timer end) */
	TIM_ITConfig(TIM10, TIM_IT_CC2, DISABLE);

	/* TIM10 disable counter */
	TIM_Cmd(TIM10, DISABLE);

	/* TIM10 clock disable */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM10, DISABLE);
}

uint16_t previousTIM4TimerCapture, currentTIM4TimerCapture;
__IO uint16_t TIM4CaptureCount = 0;
__IO uint32_t TIM4CaptureValue32 = 0;

uint16_t previousTIM10TimerCapture, currentTIM10TimerCapture;
__IO uint16_t TIM10CaptureCount = 0;
__IO uint32_t TIM10CaptureValue32 = 0;

/*
 * Just sum up SELF_CALIBRATION_NUM_CAPTURE_CYCLES captured times in captureValue32.
 * This is done by, in each capture event:
 * - Calculate difference of current capture count and previous.
 * - If the counter overflowed, the subtraction will underflow and the result is still correct.
 *   (therefore the uint16_t casts)
 */
void TIM4_IRQHandler(void) {
	if (TIM_GetITStatus(TIM4, TIM_IT_CC2) != RESET) {
		/* Clear TIM4 Capture compare interrupt pending bit */
		TIM_ClearITPendingBit(TIM4, TIM_IT_CC2);
		if (TIM4CaptureCount == 0) {
			/* Get the initial Capture value */
			previousTIM4TimerCapture = TIM_GetCapture2(TIM4);
			TIM4CaptureCount++;
		} else if (TIM4CaptureCount < SELF_CALIBRATION_NUM_CAPTURE_CYCLES) {
			/* Get the Input Capture value */
			currentTIM4TimerCapture = TIM_GetCapture2(TIM4);
			TIM4CaptureValue32 += (uint16_t) (currentTIM4TimerCapture
					- previousTIM4TimerCapture);
			previousTIM4TimerCapture = currentTIM4TimerCapture;
			TIM4CaptureCount++;
		} else if (TIM4CaptureCount == SELF_CALIBRATION_NUM_CAPTURE_CYCLES) {
			/* Get the Input Capture value */
			currentTIM4TimerCapture = TIM_GetCapture2(TIM4);
			TIM4CaptureValue32 += (uint16_t) (currentTIM4TimerCapture
					- previousTIM4TimerCapture);
			TIM4CaptureCount++;
		} else {
			// do nothing more.
		}
	}
}

void TIM10_IRQHandler(void) {
	if (TIM_GetITStatus(TIM10, TIM_IT_CC1) != RESET) {
		TIM_ClearITPendingBit(TIM10, TIM_IT_CC1);
		trace_printf("pulse\n");
		if (TIM10CaptureCount < GPS_TIMEPULSE_WASTED_PULSES) {
			// waste the first 2 pulse.
			TIM10CaptureCount++;
		} else if (TIM10CaptureCount == GPS_TIMEPULSE_WASTED_PULSES) {
			TIM10CaptureValue32 = -TIM_GetCapture1(TIM10);
			TIM10CaptureCount++;
		} else if (TIM10CaptureCount == GPS_TIMEPULSE_WASTED_PULSES + 1) {
			// Any overflow was accounted for already.
			TIM10CaptureValue32 += TIM_GetCapture1(TIM10);
			TIM10CaptureCount++;
		}
		// trace_printf("TIM10 capture2 fired! %d %d\r\n", TIM10CaptureCount, TIM_GetCapture1(TIM10));
	}
	if (TIM_GetITStatus(TIM10, TIM_IT_Update) != RESET) {
		// trace_printf("TIM10OVF\r\n");
		TIM_ClearITPendingBit(TIM10, TIM_IT_Update);
		if (TIM10CaptureCount == GPS_TIMEPULSE_WASTED_PULSES + 1)
			TIM10CaptureValue32 += 60000;
	}
}

// Start over again, throwing away intermediate results
static void modulationCalibration_Reset() {
	NVIC_DisableIRQ(TIM4_IRQn);
	__DSB();
	__ISB();
	TIM4CaptureCount = 0;
	TIM4CaptureValue32 = 0;

	// Necessary?? Well an outstanding capture is not harmful. We can use it.
	// TIM_ClearITPendingBit(TIM4, TIM_IT_CC2);
	// __DSB();
	// __ISB();
	NVIC_EnableIRQ(TIM4_IRQn);
}

// Start over again, throwing away intermediate results
static void oscillatorCalibration_Reset() {
	TIM10CaptureCount = 0;
}

/**
 * Self calibrate : Set the deviation measured, and the measured frequency
 * (assuming a perfect SystemCoreClock)
 */
uint8_t selfCalibrateModulation(
		uint32_t maxTime,
		uint32_t fsys,
		double* deviationMeasured,
		double* oscillatorFrequencyMeasured) {

	setupChannel2DAC();
	modulationCalibration_Config();

	uint8_t offset = 0;

	uint32_t counts[2] = { 0, 0 };
	uint32_t totalCount = 0;
	uint16_t DAC_Out = 2048 - SELF_CALIBRATION_MODULATION_AMPL_PP / 2;
	uint8_t i;

	setDAC2(DAC_Out);
	timer_sleep(1);

	timer_mark();

	modulationCalibration_Reset();

	for (i = 0;
			(i < SELF_CALIBRATION_NUMCYCLES * 2) && (!timer_elapsed(maxTime));
			) {
		// True if count is complete
		if (TIM4CaptureCount > SELF_CALIBRATION_NUM_CAPTURE_CYCLES) {
			// trace_printf("DAC: %u, count: %u\n", DAC_Out, TIM4CaptureValue32);
			counts[offset] += TIM4CaptureValue32;
			totalCount += TIM4CaptureValue32;
			offset = !offset;
			DAC_Out = 2048 - SELF_CALIBRATION_MODULATION_AMPL_PP
					/ 2 + offset * SELF_CALIBRATION_MODULATION_AMPL_PP;
			setDAC2(DAC_Out);
			timer_sleep(1);
			modulationCalibration_Reset();
			i++;
		}
	}

	// Stop using hardware.
	modulationCalibration_Shutdown();

	int32_t diff = counts[0] - counts[1];

	trace_printf("Self-cal diff %d, counted total %u\n", diff, totalCount);
	*deviationMeasured = (double) diff / counts[0];

	 uint64_t oscFreq = (uint64_t)SELF_CALIBRATION_NUMCYCLES
			* 2 * 8		// this number of captures
			* (uint64_t)SELF_CALIBRATION_NUM_CAPTURE_CYCLES 			// all repeated this many times
			* (uint64_t)fsys													// counting this number of CPU cycles each pulse
			* (uint64_t)CDCEL913_SELFCALIBRATION_DIVISION						// the time of each pulse
			/ (uint64_t)totalCount;

	 trace_printf("Self-cal frequency %llu\n", oscFreq);
	 *oscillatorFrequencyMeasured = oscFreq;

	// true if we got finished.
	return (i == SELF_CALIBRATION_NUMCYCLES * 2);
}

int16_t getADCStepCount(double undermodulationFactor, double testedDeviation,
		double desiredDeviation) {
	double idealStepsPerSymbol = SELF_CALIBRATION_MODULATION_AMPL_PP
			* desiredDeviation / testedDeviation;
	// trace_printf("Best steps : %d\r\n", (int) idealStepsPerSymbol);
	return (int16_t) (idealStepsPerSymbol + 0.5);
}

uint8_t getOscillatorCalibration(uint32_t maxTime, uint32_t* result) {
	oscillatorCalibration_Config();
	oscillatorCalibration_Reset();

	timer_mark();

	while ((!timer_elapsed(maxTime)) && (TIM10CaptureCount != GPS_TIMEPULSE_WASTED_PULSES + 2))
		;

	oscillatorCalibration_Shutdown();

	if (TIM10CaptureCount == GPS_TIMEPULSE_WASTED_PULSES + 2) {
		*result = TIM10CaptureValue32;
		return 1;
	}

	return 0;
}
