/**
 ******************************************************************************
 * @file    GPIO/IOToggle/main.c
 * @author  MCD Application Team
 * @version V1.2.0
 * @date    16-May-2014
 * @brief   Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT 2014 STMicroelectronics</center></h2>
 *
 * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
 * You may not use this file except in compliance with the License.
 * You may obtain a copy of the License at:
 *
 *        http://www.st.com/software_license_agreement_liberty_v2
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include <math.h>

#include "ADC.h"
#include "APRS.h"
#include "diag/Trace.h"
#include "Globals.h"
#include "PLL.h"
#include "Power.h"
#include "RecordStorage.h"
#include "RF24Wrapper.h"
#include "RTC.h"
#include "SelfCalibration.h"
#include "Setup.h"
#include "StabilizedOscillator.h"
#include "Systick.h"
#include "Types.h"
#include "WSPR.h"
#include "../Libraries/CMSIS/Device/ST/STM32L1xx/Include/stm32l1xx.h"
#include "../Libraries/STM32L1xx_StdPeriph_Driver/inc/stm32l1xx_gpio.h"
#include "../Libraries/STM32L1xx_StdPeriph_Driver/inc/stm32l1xx_rcc.h"

// Some globals.
// int8_t simpleTemperature;
float temperature;
int8_t simpleTemperature;
uint8_t numRestarts __attribute__((section (".noinit")));
float batteryVoltage;
float solarVoltage;
//float internalTemperature;
uint8_t nextWSPRMessageTypeIndex;
uint16_t lastWSPRWindowWaitTime;

int scheduleSeconds = -1;

uint8_t mainPeriod;
uint8_t mainPeriodCnt;

uint8_t WSPRPeriod;
uint8_t WSPRCnt;

uint8_t HFAPRSPeriod;
uint8_t HFAPRSCnt;

boolean latestAPRSRegions[12]; 	 // 12 is sufficently large for the world map...
boolean latestAPRSCores[12];	 // 12 is sufficently large for the world map...

const CalibrationRecord_t* currentCalibration = &defaultCalibration;

boolean interruptAlarm;

void diagsWhyReset() {
	uint8_t whyReset = (RCC->CSR >> 24);
	RCC_ClearFlag();
	if (whyReset & 1 << 7)
		trace_printf("Reset bc low power\n");
	if (whyReset & 1 << 6)
		trace_printf("Reset bc window WD\n");
	if (whyReset & 1 << 5)
		trace_printf("Reset bc independent WD\n");
	if (whyReset & 1 << 4)
		trace_printf("Reset bc software\n");
	if (whyReset & 1 << 3)
		trace_printf("Reset bc POR/PDR\n");
	if (whyReset & 1 << 2)
		trace_printf("Reset bc NRSTpin\n");
	if (whyReset & 1 << 1)
		trace_printf("Reset bc OBL\n");
}

void initGeneralIOPorts() {
	GPIO_InitTypeDef GPIO_InitStructure;

	/* GPIOA and B Periph clock enable */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);

	// 6: LED, 1: WSPR arm, 0: SPI1 NSS
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_1;	 // | GPIO_Pin_0;

	// GPIOB stuff: LED, WSPR arm Si4463 SS
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIOB->ODR = 0;	 //  GPIO_Pin_0; // LED off, disarm WSPR, disable Si4463 SS.

	/* Configure GPS power pin and Si4463 SDN pin */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;	 // | GPIO_Pin_8;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIOA->ODR = GPIO_Pin_0;	 // | GPIO_Pin_8;

	/*
	 // NIRQ pin from Si4463 as input.
	 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
	 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	 GPIO_Init(GPIOA, &GPIO_InitStructure);
	 */
}

void performPrecisionADC() {
	ADC_DMA_init(ADCUnloadedValues);

	// TODO: Power save sleep.
	while (!ADC_DMA_Complete) {
		PWR_EnterSleepMode(PWR_Regulator_ON, PWR_SLEEPEntry_WFI);
	}

	ADC_DMA_shutdown();

	batteryVoltage = ADC_batteryUnloadedVoltage();
	solarVoltage = ADC_solarVoltage();
	temperature = ADC_temperature();
	simpleTemperature = ADC_simpleTemperature(temperature);
	// internalTemperature = ADC_internalTemperature();
}

/*
 * Squeeze all relevant info out of GPS, including:
 * - Time
 * - Position
 * - Calibration if not already having one for current temperature
 * The GPS is left running (if it was ever started)
 */
boolean GPSCycle() {
	// Just in case. it bothers the GPS pretty bad. And it should never be running now anyway.
	PLL_shutdown();

	if (PWR_isSafeToUseGPS()) {
		GPS_start();

		if (GPS_waitForTimelock(MAX_GPS_TIMELOCK_TIME)) {
			RTC_setRTC(&GPSTime.date, &GPSTime.time);
			// trace_printf("Set RTC to %02d:%02d: %d\n", GPSTime.time.hours,
			//		GPSTime.time.minutes, success);
		}

		if (REQUIRE_HIGH_PRECISION_POSITIONS && batteryVoltage >= 3.3) {
			trace_printf("Waiting for HP position\n");
			GPS_waitForPrecisionPosition(
			REQUIRE_HIGH_PRECISION_MAX_TIME_S * 1000);
		} else {
			GPS_waitForPosition(POSITION_MAX_TIME_S * 1000);
		}

		/*
		 // This will (be warned) kill the GPS.
		 // Reason is that the GPS is so noisy on the supply that the PLL runs rough.
		 // WUT interrupt will also be disabled (check if that is really needed!).
		 currentCalibration = getCalibration(simpleTemperature, true);

		 // As a side effect, fix the RTC.
		 RTC_setCalibration(currentCalibration->RTCNeededCorrectionPP10M);

		 // One final chance, just in case the calibration saved the day:
		 if (!highPrecisionPositionOkay) {
		 trace_printf("Retrying HP position\n");
		 highPrecisionPositionOkay = GPS_waitForPrecisionPosition(1000);
		 }
		 */

		// Update APRS map
		APRS_frequenciesFromPosition(&lastNonzeroPosition, latestAPRSRegions,
				latestAPRSCores);
		return true;
	} else {
		trace_printf("GPS not safe to run.\n");
		return false;
	}
}

// If GPS was powered, it's possible there now is a position available that was not available before.
// Might as well collect it for later.
void GPS_lastChance() {
	GPS_waitForPrecisionPosition(1000);
}

void calibrationCycle() {
	// This may (be warned) kill the GPS.
	// Reason is that the GPS is so noisy on the supply that the PLL runs rough.
	// WUT interrupt will also be disabled if RTC is calibrated (currently disabled).
	currentCalibration = getCalibration(simpleTemperature, true);

	// As a side effect, fix the RTC.
	RTC_setCalibration(currentCalibration->RTCNeededCorrectionPP10M);
}

void doWSPR() {
	if (!PWR_isSafeToUseHFTx())
		return;

	currentCalibration = getCalibration(simpleTemperature, false);

	PLL_Setting_t pllSetting;

	double maxError = 10E-6;
	while (maxError < 100E-6) {
		if (PLL_bestPLLSetting(
				currentCalibration->transmitterOscillatorFrequencyAtDefaultTrim,
				WSPR_FREQUENCIES[THIRTY_M], maxError, &pllSetting)) {

			trace_printf("Using fOsc=%d, N=%d, M=%d, trim=%d\n",
					currentCalibration->transmitterOscillatorFrequencyAtDefaultTrim,
					pllSetting.N, pllSetting.M, pllSetting.trim);

			trace_printf("Waiting for WSPR window\n");
			lastWSPRWindowWaitTime = RTC_waitTillModuloMinutes(2, 1);

			uint8_t nextWSPRMessageType;

			if (lastNonzero3DPosition.alt < LOWALT_THRESHOLD) {
				if (nextWSPRMessageTypeIndex >= WSPR_LOWALT_SCHEDULE_LENGTH) {
					nextWSPRMessageTypeIndex = 0;
				}
				nextWSPRMessageType =
						WSPR_LOWALT_SCHEDULE[nextWSPRMessageTypeIndex];
			} else {
				if (nextWSPRMessageTypeIndex >= WSPR_SCHEDULE_LENGTH) {
					nextWSPRMessageTypeIndex = 0;
				}
				nextWSPRMessageType = WSPR_SCHEDULE[nextWSPRMessageTypeIndex];
			}

			prepareWSPRMessage(nextWSPRMessageType, batteryVoltage);
			nextWSPRMessageTypeIndex++;

			PWR_startHFTx(batteryVoltage);
			WSPR_Transmit(THIRTY_M, &pllSetting, WSPR_DEVIATION_DAC_30m);
			PWR_stopHFTx();

			// Recurse, do it again if we sent the rather uninformative TYPE1.
			if (nextWSPRMessageType == TYPE1) {
				doWSPR();
			}

			break;
		} else {
			trace_printf(
					"NO feasible PLL setting in range! Should not really happen. Anyway, we try again with more tolerance.\n");
			maxError += 10E-6;
		}
	}
}

void WSPRCycle() {
	WSPRCnt++;
	if (WSPRCnt >= WSPRPeriod) {
		WSPRCnt = 0;
		doWSPR();
	}
}

void HF_APRSCycle() {
	if (!PWR_isSafeToUseHFTx()) {
		trace_printf("Unsafe to HF tx\n");
		return;
	}

	APRS_transmitRandomMessage(HF, COMPRESSED_POSITION_MESSAGE, 10149300,
			currentCalibration->transmitterOscillatorFrequencyAtDefaultTrim);
}

void VHF_APRSCycle() {
	if (!PWR_isSafeToUseVHFTx()) {
		trace_printf("Unsafe to VHF tx\n");
		return;
	}

	boolean isOutsideAPRS = true;
	boolean isOutsideCore = true;

	// Send position on all relevant frequencies, without a break in between.
	for (uint8_t i = 0; i < APRS_WORLD_MAP_LENGTH; i++) {
		if (latestAPRSRegions[i]) {
			isOutsideAPRS = false;
			trace_printf("Sending a COMPRESSED_POSITION_MESSAGE on %d\n",
					APRS_WORLD_MAP[i].frequency * 1000);
			APRS_transmitRandomMessage(VHF, COMPRESSED_POSITION_MESSAGE,
					APRS_WORLD_MAP[i].frequency * 1000,
					currentCalibration->transmitterOscillatorFrequencyAtDefaultTrim);

		}
		if (latestAPRSCores[i])
			isOutsideCore = false;
	}

// Send status on all relevant frequencies, without a break in between but with a break before.
// If we are not in network coverage, don't bother to try or to delay first.
	if (!isOutsideAPRS) {
		timer_sleep(2000);
		for (uint8_t i = 0; i < APRS_WORLD_MAP_LENGTH; i++) {
			if (latestAPRSRegions[i]) {
				trace_printf("Sending a STATUS_MESSAGE on %d\n",
						APRS_WORLD_MAP[i].frequency * 1000);
				APRS_transmitRandomMessage(VHF, STATUS_MESSAGE,
						APRS_WORLD_MAP[i].frequency * 1000,
						currentCalibration->transmitterOscillatorFrequencyAtDefaultTrim);

			}
		}

// Test the stored record crap.
		if (isOutsideCore) {
			// Store a message.
			if (timeToStoreRecord()) {
				StoredPathRecord_t* recordToStore = nextRecordIn();
				storeToRecord(recordToStore);
			}
		} else {
			// Get rid of some storage backlog, first take that scheduled for a 2nd transmission
			int maxStored = 10;
			StoredPathRecord_t* storedRecord;
			while (hasRecordOutForLastTransmission() && maxStored-- > 0) {
				timer_sleep(1000);
				storedRecord = nextRecordOutForLastTransmission();
				for (uint8_t i = 0; i < APRS_WORLD_MAP_LENGTH; i++) {
					if (latestAPRSCores[i])
						APRS_transmitStoredMessage(VHF, storedRecord,
								APRS_WORLD_MAP[i].frequency * 1000,
								currentCalibration->transmitterOscillatorFrequencyAtDefaultTrim);
				}
			}

			maxStored = 5;

			// Get rid of some storage backlon, 1st transmission
			while (hasRecordOutForFirstTransmission() && maxStored-- > 0) {
				timer_sleep(1000);
				storedRecord = nextRecordOutForFirstTransmission();
				for (uint8_t i = 0; i < APRS_WORLD_MAP_LENGTH; i++) {
					if (latestAPRSCores[i])
						APRS_transmitStoredMessage(VHF, storedRecord,
								APRS_WORLD_MAP[i].frequency * 1000,
								currentCalibration->transmitterOscillatorFrequencyAtDefaultTrim);
				}
			}
		}
	}
}

void APRSCycle() {
	// Just do VHF in any case.
	VHF_APRSCycle();

///	trace_printf("HF period %d cnt %d\n", HFAPRSCnt,HFAPRSPeriod);
	HFAPRSCnt++;
	if (HFAPRSCnt >= HFAPRSPeriod) {
		HFAPRSCnt = 0;
		HF_APRSCycle();
	}
}

void reschedule(
		int _scheduleSeconds,
		uint8_t _mainPeriod,
		uint8_t _WSPRPeriod,
		uint8_t _HFAPRSPeriod) {
	// Apparently we often miss wakeups if not doing this over again all the time... rubbish.
	// Need fixing.
	RTC_setWakeup(_scheduleSeconds);

	if (scheduleSeconds != _scheduleSeconds
			||
			mainPeriod != _mainPeriod
			||
			WSPRPeriod != _WSPRPeriod
			||
			HFAPRSPeriod != _HFAPRSPeriod) {
		trace_printf("Rescheduling : %d seconds\n", _scheduleSeconds);
		mainPeriodCnt = 0;
		WSPRCnt = 0;
		HFAPRSCnt = 0;
	}

	scheduleSeconds = _scheduleSeconds;
	mainPeriod = _mainPeriod;
	WSPRPeriod = _WSPRPeriod;
	HFAPRSPeriod = _HFAPRSPeriod;
}

extern void SetSysClock(void);

void mainCycle() {
	if (batteryVoltage >= 3.7&& simpleTemperature >= NORMAL_SCHEDULE_MIN_TEMPERATURE) {
		boolean GPSStarted = GPSCycle();

		calibrationCycle();

		APRSCycle();
		WSPRCycle();
		if (GPS_isGPSRunning()) {
			GPS_lastChance();
		}
// This should ensure that GPS get some long run times in the day
// but OTOH does not drain the battery.
		if (GPSStarted && batteryVoltage >= 4.1) {
			// leave GPS running if it already was, or restart it.
			GPS_start();
		} else {
			GPS_kill();
		}
	} else if (batteryVoltage >= 3.3) {
// This should not happen in daytime anyway...
		GPSCycle();
		calibrationCycle();
		GPS_kill(); // if calibration did not use and kill GPS, do it now.
		APRSCycle();
		WSPRCycle();
	} else {
		APRSCycle(); // uses almost no power anyway
	}
}

void wakeupCycle() {
	performPrecisionADC();

	trace_printf("ADCValue0: %d\t", ADCUnloadedValues[0]);
	trace_printf("ADCValue1: %d\t", ADCUnloadedValues[1]);
	trace_printf("ADCValue2: %d\t", ADCUnloadedValues[2]);
	trace_printf("ADCValue3: %d\n", ADCUnloadedValues[3]);
	trace_printf("Precision batt voltage: %d\n", (int) (1000 * batteryVoltage));
	trace_printf("Temperature mC: %d\n", (int) (1000 * temperature));

	// Are we alive at all?
	if (batteryVoltage >= 3.0) {
		// yes, ok start the real HSE clock.
		SetSysClock();

		// Time to do main cycle? (the variables should have been set somewhere above, all of them)
		mainPeriodCnt++;
		if (mainPeriodCnt >= mainPeriod) {
			mainPeriodCnt = 0;
			mainCycle();
		}

		// Are we in a very good shape?
		if (batteryVoltage
				>= 3.7&& simpleTemperature >= NORMAL_SCHEDULE_MIN_TEMPERATURE) {
			trace_printf("Good batt\n");
			if (lastNonzero3DPosition.alt < LOWALT_THRESHOLD) {
				reschedule(LOWALT_SCHEDULE_TIME);
			} else {
				reschedule(DAY_SCHEDULE_TIME);
			}
			// Just Oki Doki state?
		} else if (batteryVoltage >= 3.3) {
			// This should not happen in daytime anyway...
			trace_printf("OK batt\n");
			reschedule(NIGHT_SCHEDULE_TIME);
			// Pretty bad?
		} else {
			trace_printf("Low batt\n");
			reschedule(LOWBATT_SCHEDULE_TIME);
		}

	} else { // we are below 3.0
		reschedule(CRISIS_SCHEDULE_TIME);
		GPS_kill();
	}
}

int main() {
	/*!< At this stage the microcontroller clock setting is already configured,
	 this is done through SystemInit() function which is called from startup
	 file (startup_stm32l1xx_xx.s) before to branch to application main.
	 To reconfigure the default setting of SystemInit() function, refer to
	 system_stm32l1xx.c file
	 */
	SetSysClock();
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
	initGeneralIOPorts();

	numRestarts++;

	// Fire up systick
	systick_start();

	// for debug only.
	// timer_sleep(1000);

	RTC_init();

	if (DEFEAT_VOLTAGE_CHECKS) {
		invalidateStartupLog();
	}

	double deviation = 0;
	uint32_t frequency = 0;

	/*
	timer_sleep(5000);
	 selfCalibrateModulation(16E6, &WSPR_MODULATION_SELF_CALIBRATION,
	 PLL_PREFERRED_TRIM, &deviation, &frequency);
	 trace_printf("SC HFAPRSmod: freq %d, dev %d at %d p-p\n", frequency,
	 (int) (deviation * 1E9),
	 WSPR_MODULATION_SELF_CALIBRATION.modulation_PP);

	 trace_printf("Deviation: %d\n", (int) (deviation * 1E9));

	 double WSPRdesiredDeviation = 12000.0 / 8192.0 / 10142200.0;
	 double APRSdesiredDeviation = 200.0 / 10149300.0;

	 // testpp / testdev = desiredpp / desireddev.
	 // desiredpp = (testpp / testdev) * desireddev
	 double WSPRmodulationNeeded = WSPR_MODULATION_SELF_CALIBRATION.modulation_PP/deviation * WSPRdesiredDeviation;
	 trace_printf("Need per WSPR step DEC count: %d\n", (int)(WSPRmodulationNeeded+0.5));

	 double APRSmodulationNeeded = WSPR_MODULATION_SELF_CALIBRATION.modulation_PP/deviation * APRSdesiredDeviation;
	 trace_printf("Need per APRS step DEC count: %d\n", (int)(APRSmodulationNeeded+0.5));

	timer_sleep(5000);
	while (1) {
		for (uint8_t i = 0; i < 21; i++) {
			selfCalibrateTrimming(&deviation, i);
			trace_printf("%d pF: %d\n", i, (int) (deviation * 1E7));
		}
	}
	*/

	while (1) {
		// Various stuff was disabled, either by us or by the stop mode.
		// Get it started again.
		PWR_RTCAccessCmd(ENABLE);
		systick_start();

		wakeupCycle();

		// Prepare sleep
		systick_end();
		GPS_stopUART(); // This will not cut power. Assume it was already done if desired.
		RTC_debugRTCTime();
		RTC_ClearITPendingBit(RTC_IT_WUT); // see what that does.
		interruptAlarm = true;
		trace_printf("Wakeup interval is %d seconds, ", scheduleSeconds);
		trace_printf("Gng to sleep\n");

		PWR_EnterSTOPMode(PWR_Regulator_ON, PWR_STOPEntry_WFI);

		interruptAlarm = false;
	}

	/*
	 WSPRSynthesisExperiment(25998440);

	 // APRS_debugWorldMap();

	 // Finito with GPS. Well we might want to have a position too.
	 // GPS_shutdown();

	 //aprs_compressedMessage(123, 28);

	 // RTC_TimeTypeDef rtcTime;
	 // RTC_GetTime(RTC_Format_BIN, &rtcTime);
	 // scheduleASAPAlarmInSlot(1);
	 // setWakeup(10);
	 while (1) {
	 timer_sleep(10000);
	 debugGPSTime();
	 debugRTCTime();
	 trace_printf("\n");

	 prepareWSPRMessage(wsprMessageSeq == 4 ? 1 : 3, wsprMessageSeq, 8);
	 do {
	 RTC_GetTime(RTC_HourFormat_24, &rtcTime);
	 } while ((rtcTime.RTC_Minutes % 2 != 0) || rtcTime.RTC_Seconds != 1);

	 trace_printf("WSPR at %02u:%02u:%02u\n", rtcTime.RTC_Hours,
	 rtcTime.RTC_Minutes, rtcTime.RTC_Seconds);

	 WSPR_TransmitCycle(bandSettings, calibration);

	 trace_printf("End WSPR!\n");

	 // Now we can turn on that stupid thing again.
	 // GPS_init();

	 wsprMessageSeq = (wsprMessageSeq + 1) % 5;
	 */
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
