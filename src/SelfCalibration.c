/*
 * SelfCalibration.c
 *
 *  Created on: May 26, 2015
 *      Author: dongfang
 */
#include <diag/trace.h>
#include "stm32l1xx_conf.h"
#include "systick.h"
#include "PLL.h"
#include "DAC.h"
#include "GPS.h"
#include "Power.h"
#include "Globals.h"
#include "SelfCalibration.h"

// Slow and fine.
const SelfCalibrationConfig_t WSPR_MODULATION_SELF_CALIBRATION = {
		.maxTimeMillis = 20000, .DACChannel = DAC2, .modulation_PP = 1000,
		.numCyclesPerRound = 5000, .numRounds = 3 };

// Not so fine, that is not needed.
const SelfCalibrationConfig_t HF_APRS_SELF_CALIBRATION = {
		.maxTimeMillis = 5000, .DACChannel = DAC2, .modulation_PP = 3660,
		.numCyclesPerRound = 2000, .numRounds = 3 };

// For measuring effects of xtal capacitor trimming. No modulation.
const SelfCalibrationConfig_t TRIM_SELF_CALIBRATION = { .maxTimeMillis = 5000,
		.DACChannel = DAC2, .modulation_PP = 0, .numCyclesPerRound = 800,
		.numRounds = 1 };

static void modulationCalibration_initHW() {
	TIM_ICInitTypeDef TIM_ICInitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	/* TIM4 clock enable */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

	/* GPIOB clock enable */
	//RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
	/* TIM4 channel 2 pin (PB.07) configuration */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_TIM4);

	// Tie the ADC1 output aka PA.4 to ground.
	// GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	// GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	// GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	// GPIOA->ODR &= ~(GPIO_Pin_4);
	// GPIO_Init(GPIOA, &GPIO_InitStructure);

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

static void modulationCalibration_shutdownHW() {
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
}

void TIM10Calibration_initHW() {
	TIM_ICInitTypeDef TIM_ICInitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM10, ENABLE);

	// Jumper wired GPS timepulse
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz; // More was not better. really? Not better resolution?
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	// Is source of TIM10
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
	TIM_ICInitStructure.TIM_ICFilter = 0x2;
	TIM_ICInit(TIM10, &TIM_ICInitStructure);

	// Just to be sure ..
	TIM_TimeBaseInitTypeDef timerInitStructure;
	timerInitStructure.TIM_Prescaler = 0;
	timerInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	timerInitStructure.TIM_Period = 60000 - 1; // why does 0 not work??
	timerInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInit(TIM10, &timerInitStructure);

	TIM_ClearITPendingBit(TIM10, TIM_IT_CC1);
	TIM_ClearITPendingBit(TIM10, TIM_IT_Update);

	/* Enable the CC1 Interrupt Request */
	TIM_ITConfig(TIM10, TIM_IT_CC1, ENABLE);
	TIM_ITConfig(TIM10, TIM_IT_Update, ENABLE);

	/* Enable the TIM10 global Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = TIM10_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* TIM enable counter */
	TIM_Cmd(TIM10, ENABLE);
}

void RTCCalibration_init() {
	TIM_ICInitTypeDef TIM_ICInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	/* TIM10 configuration: Input Capture mode ---------------------
	 The RTC is supposed to run this shit.
	 ------------------------------------------------------------ */

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM10, ENABLE);

	RTC_WakeUpCmd(DISABLE);
	RTC_WakeUpClockConfig(RTC_WakeUpClock_CK_SPRE_16bits);
	RTC_SetWakeUpCounter(0); // 1 second is enough.
	RTC_WakeUpCmd(ENABLE);

	// Dunno if needed. It really is..
	// OTOH we can just drop this RTC cal crap.
	RTC_ITConfig(RTC_IT_WUT, ENABLE);
	TIM_RemapConfig(TIM10, TIM10_RTC);

	TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	TIM_ICInitStructure.TIM_ICFilter = 0x0;
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

static void TIM10Calibration_shutdownHW() {
	NVIC_InitTypeDef NVIC_InitStructure;

	/* Disable the TIM10 global Interrupt, NVIC end. */
	NVIC_InitStructure.NVIC_IRQChannel = TIM10_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* Disable the CC2 Interrupt Request (timer end) */
	TIM_ITConfig(TIM10, TIM_IT_Update, DISABLE);
	TIM_ITConfig(TIM10, TIM_IT_CC1, DISABLE);

	/* TIM10 disable counter */
	TIM_Cmd(TIM10, DISABLE);

	/* TIM10 clock disable */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM10, DISABLE);
}

uint16_t previousTIM4TimerCapture, currentTIM4TimerCapture;
__IO uint16_t TIM4CaptureCount = 0;
__IO uint16_t TIM4CaptureStop = 0;
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
		// trace_printf("Tim4Cap\n");
		/* Clear TIM4 Capture compare interrupt pending bit */
		TIM_ClearITPendingBit(TIM4, TIM_IT_CC2);
		if (TIM4CaptureCount == 0) {
			/* Get the initial Capture value */
			previousTIM4TimerCapture = TIM_GetCapture2(TIM4);
			TIM4CaptureCount++;
		} else if (TIM4CaptureCount <= TIM4CaptureStop) {
			/* Get the Input Capture value */
			currentTIM4TimerCapture = TIM_GetCapture2(TIM4);
			TIM4CaptureValue32 += (uint16_t) (currentTIM4TimerCapture
					- previousTIM4TimerCapture);
			previousTIM4TimerCapture = currentTIM4TimerCapture;
			TIM4CaptureCount++;
		} else {
			// do nothing more.
		}
	}
}

// main oscillator calibration
void TIM10_IRQHandler(void) {
	// Capture event.
	if (TIM_GetITStatus(TIM10, TIM_IT_CC1) != RESET) {
		TIM_ClearITPendingBit(TIM10, TIM_IT_CC1);
		RTC_ClearITPendingBit(RTC_IT_WUT);
		trace_printf("Tim10Cap\n");
		if (TIM10CaptureCount == 0) {
			TIM10CaptureValue32 = -TIM_GetCapture1(TIM10);
			TIM10CaptureCount++;
		} else if (TIM10CaptureCount == 1) {
			// Any overflow was accounted for already.
			TIM10CaptureValue32 += TIM_GetCapture1(TIM10);
			TIM10CaptureCount++;
		}
	}
	if (TIM_GetITStatus(TIM10, TIM_IT_Update) != RESET) {
		// Normal overflow, compensate for that by adding to extended counter.
		TIM_ClearITPendingBit(TIM10, TIM_IT_Update);
		if (TIM10CaptureCount == 1)
			TIM10CaptureValue32 += 60000;
	}
}

// Start over again, throwing away intermediate results
static void modulationCalibration_reset() {
	NVIC_DisableIRQ(TIM4_IRQn);
	__DSB();
	__ISB();
	TIM4CaptureCount = 0;
	TIM4CaptureValue32 = 0;

	// Necessary?? Well an outstanding capture is not harmful. We can use it.
	TIM_ClearITPendingBit(TIM4, TIM_IT_CC2);
	NVIC_EnableIRQ(TIM4_IRQn);
}

// Start over again, throwing away intermediate results
static void TIM10Calibration_reset() {
	TIM10CaptureCount = 0;
	TIM10CaptureValue32 = 0;
}

/**
 * Self calibrate : Set the deviation measured, and the measured frequency
 * (assuming a very stable SystemCoreClock).
 */
boolean selfCalibrateModulation(
		uint32_t hseFrequency,
		const SelfCalibrationConfig_t* settings,
		uint8_t trim,
		double* deviationMeasured,
		uint32_t* oscillatorFrequencyMeasured) {

	TIM4CaptureStop = settings->numCyclesPerRound;
	PLL_setDirectModeWithDivision(trim, CDCE913_SELFCALIBRATION_DIVISION);
	DAC2_initHW();
	modulationCalibration_initHW();

	uint8_t offset = 0;
	uint32_t counts[2] = { 0, 0 };
	uint32_t totalCount = 0;
	uint16_t DAC_Out = 2048 - settings->modulation_PP / 2;
	uint8_t i;

	setDAC(settings->DACChannel, DAC_Out);
	timer_sleep(1);

	timer_mark();

	modulationCalibration_reset();

	for (i = 0;
			(i < settings->numRounds * 2)
					&& (!timer_elapsed(settings->maxTimeMillis));) {
		// True if count is complete
		if (TIM4CaptureCount > TIM4CaptureStop) {
			// trace_printf("DAC: %u, count: %u\n", DAC_Out, TIM4CaptureValue32);
			counts[offset] += TIM4CaptureValue32;
			totalCount += TIM4CaptureValue32;
			offset = !offset;
			DAC_Out = 2048 - settings->modulation_PP / 2
					+ offset * settings->modulation_PP;
			setDAC(settings->DACChannel, DAC_Out);
			timer_sleep(1);
			modulationCalibration_reset();
			i++;
		} else {
			PWR_EnterSleepMode(PWR_Regulator_ON, PWR_SLEEPEntry_WFI);
		}
	}

	// Stop using hardware.
	modulationCalibration_shutdownHW();

	int32_t diff = counts[0] - counts[1];

	//trace_printf("Self-cal diff %d, counted total %u\n", diff, totalCount);
	*deviationMeasured = (double) diff * 2 / (counts[0] + counts[1]);

	uint64_t oscFreq = (uint64_t) settings->numCyclesPerRound * 2 * 8 // this number of captures
			* (uint64_t) settings->numRounds	// all repeated this many times
			* (uint64_t) hseFrequency// counting this number of CPU cycles each pulse
			* (uint64_t) CDCE913_SELFCALIBRATION_DIVISION// the time of each pulse
	/ (uint64_t) totalCount;

	//trace_printf("Self-cal frequency %llu\n", oscFreq);
	*oscillatorFrequencyMeasured = oscFreq;

	// true if we got finished.
	return (i == settings->numRounds * 2);
}

// Ground use only (by our current plan anyway)
boolean selfCalibrateTrimming(double* relIncreaseFromInitialTrim,
		uint8_t trim_pF) {

	double deviationMeasuredGarbage;
	uint32_t frequencyMeasured1;
	uint32_t frequencyMeasured2;

	selfCalibrateModulation(16E6, &TRIM_SELF_CALIBRATION,
	PLL_PREFERRED_TRIM_VALUE, &deviationMeasuredGarbage, &frequencyMeasured1);

	selfCalibrateModulation(16E6, &TRIM_SELF_CALIBRATION, trim_pF,
			&deviationMeasuredGarbage, &frequencyMeasured2);

	// Divide 2*the diff by 2*the average.
	*relIncreaseFromInitialTrim = ((double) frequencyMeasured2
			- (double) frequencyMeasured1) * 2
			/ ((double) frequencyMeasured2 + (double) frequencyMeasured1);

	return true;
}

/*
 * Should not be necessary to use in-flight, as not temperature dependent.
 * Just run once on ground and put results in the CDCEL913_XTAL_TRIM_PP10M_VALUES definition.
 */
void printTrimmingCalibrationTable() {
	double freqRelDiffPerPico = 0;

	selfCalibrateTrimming(&freqRelDiffPerPico, 0);

	boolean needsComma = false;

	for (uint8_t trim = ABS_MIN_PLL_TRIM; trim <= ABS_MAX_PLL_TRIM; trim++) {
		selfCalibrateTrimming(&freqRelDiffPerPico, trim);
		if (needsComma) trace_printf(","); else needsComma = true;
		trace_printf("%d", (int) (freqRelDiffPerPico * 1E7));
	}

	trace_printf("\n");
}

static boolean HSECalibration(uint32_t maxTime, uint32_t* result) {
	TIM10Calibration_initHW();
	TIM10Calibration_reset();

	timer_mark();

	// ttrace_printf("Waiting for TIM10 captures\n");

	while ((!timer_elapsed(maxTime)) && (TIM10CaptureCount != 2)) {
		PWR_EnterSleepMode(PWR_Regulator_ON, PWR_SLEEPEntry_WFI);
	}

	TIM10Calibration_shutdownHW();

	if (TIM10CaptureCount == 2) {
		*result = TIM10CaptureValue32;
		return true;
	}

	return false;
}

/*
 // Be aware this kills WUT interrupt.
 boolean RTCCalibration(uint32_t maxTime, uint32_t* result) {
 RTC_ITConfig(RTC_IT_WUT, DISABLE); // Better remember to restore this crap .. both the timer latch and the interrupt enable stuff.
 RTCCalibration_init();
 TIM10Calibration_reset();

 timer_mark();

 while ((!timer_elapsed(maxTime)) && (TIM10CaptureCount != 2)) {
 PWR_EnterSleepMode(PWR_Regulator_ON, PWR_SLEEPEntry_WFI);
 }

 // This should be reusable for RTC purpose also .. the difference is after all only the capture trigger source.
 TIM10Calibration_shutdownHW();

 if (TIM10CaptureCount == 2) {
 *result = TIM10CaptureValue32;
 return 1;
 }

 return 0;
 }
 */

// Beware, this kills GPS and WUT interrupt.
boolean selfCalibrate(CalibrationRecord_t* target) {
	uint32_t hseFrequency;
	uint32_t pllFrequency;
	double deviationThrowAway;

	// We need GPS now. If it is not safe to power up and not already running, quit.
	// if (!PWR_isSafeToUseDevice(E_DEVICE_GPS))
	// 	return false;
	// GPS_start();

	boolean result = true;

	trace_printf("HSE cal\n");
	result = HSECalibration(MAX_HSE_CALIBRATION_TIME, &hseFrequency);
	target->HSEFrequency = hseFrequency;

	if (!result) {
		trace_printf("HSE cal failed\n");
	}

	if (result) {
		if (hseFrequency > 16E6 * 1.002 || hseFrequency < 16E6 * 0.998) {
			trace_printf("Unrealistic HSE freq measured: %u\n", hseFrequency);
			result = false;
		} else {
			trace_printf("HSE freq measured: %u\n", hseFrequency);
		}
	}

	// Sadly, we have to stop the GPS in the middle of calibration.
	GPS_shutdown();

	if (result) {
		// RTCCalibration(3000, &RTCPeriod); // no reason to do it. We get GPS updates all the time.

		result &= selfCalibrateModulation(hseFrequency, &TRIM_SELF_CALIBRATION,
		PLL_PREFERRED_TRIM_VALUE, &deviationThrowAway, &pllFrequency);

		target->transmitterOscillatorFrequencyAtDefaultTrim = pllFrequency;

		if (pllFrequency > PLL_XTAL_DEFAULT_FREQUENCY * 1.002
				|| pllFrequency < PLL_XTAL_DEFAULT_FREQUENCY * 0.998) {
			trace_printf("Unrealistic PLL freq measured: %u\n", pllFrequency);
			result = false;
		} else {
			trace_printf("PLL freq measured: %u\n", pllFrequency);
		}

		// double RTCError = (double) RTCPeriod / (double) hseFrequency;
		target->RTCNeededCorrectionPP10M = -800; // (RTCError - 1) * 1E7;

		// trace_printf("RTC period was %d, correction needed PP10M: %d\n",
		// 		RTCPeriod, target->RTCNeededCorrectionPP10M);
	}

	return result;
}

#include "WSPR.h"

void selfCalibrateForWSPR(uint32_t fSys) {
	double deviationMeasured;
	uint32_t oscillatorFrequencyMeasured;
	const SelfCalibrationConfig_t* settings = &WSPR_MODULATION_SELF_CALIBRATION;

	trace_printf("WSPR Self-cal with HSE=%lu, trim=%d, DAC %c\n",
			fSys,
			PLL_PREFERRED_TRIM_VALUE,
			(settings->DACChannel == DAC1 ? '1' : '2'));

	// Waste one run.
	selfCalibrateModulation(fSys, settings, PLL_PREFERRED_TRIM_VALUE, &deviationMeasured, &oscillatorFrequencyMeasured);

	boolean selfCalSuccess =
			selfCalibrateModulation(fSys, settings, PLL_PREFERRED_TRIM_VALUE, &deviationMeasured, &oscillatorFrequencyMeasured);

	trace_printf("Self-cal success : %u\n", selfCalSuccess);
	trace_printf("Self-cal deviation PPB : %d\n",
			(int) (deviationMeasured * 1.0E9));
	trace_printf("Self-cal osc freq: %u\n", oscillatorFrequencyMeasured);

	uint32_t targetFrequency = WSPR_FREQUENCIES[0];

	double DACStepsPerHz = settings->modulation_PP
			/ (deviationMeasured * targetFrequency);

	float wsprStepModulation = 12000.0 * DACStepsPerHz / 8192.0;
	float hfPacketStepModulation = 200 * DACStepsPerHz;

	trace_printf("%d DAC steps per Hz, %d per WSPR step, %d per HF packet step\n",
			(int) DACStepsPerHz, (int) (wsprStepModulation + 0.5),
			(int) (hfPacketStepModulation + 0.5));
}

