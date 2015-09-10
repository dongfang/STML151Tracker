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
#include "GPS.h"
#include "IndividualBoard.h"
#include "Physics.h"
#include "Power.h"
#include "RecordStorage.h"
#include "RTC.h"
#include "SelfCalibration.h"
#include "Setup.h"
#include "StabilizedOscillator.h"
#include "Systick.h"
#include "Types.h"
#include "WSPR.h"
#include "CMSIS/Device/ST/STM32L1xx/Include/stm32l1xx.h"
#include "STM32L1xx_StdPeriph_Driver/inc/stm32l1xx_gpio.h"
#include "STM32L1xx_StdPeriph_Driver/inc/stm32l1xx_rcc.h"

// Some globals.
float temperature;
int8_t simpleTemperature;
uint8_t numRestarts __attribute__((section (".noinit")));
float batteryVoltage;
float solarVoltage;
//float internalTemperature;

char scheduleName = '-';
uint16_t mainPeriodWakeupCycles;
uint16_t mainPeriodCounter;
uint16_t wsprCounter;
uint16_t hfPacketCounter;

boolean latestAPRSRegions[12]; 	 // 12 is sufficently large for the world map...
boolean latestAPRSCores[12];	 // 12 is sufficently large for the world map...

uint8_t nextWSPRMessageTypeIndex;
uint16_t lastWSPRWindowWaitTime;

const CalibrationRecord_t* currentCalibration = &defaultCalibration;

void diagsWhyReset() {
	int whyReset = (RCC->CSR >> 24);
	RCC_ClearFlag();
	if (whyReset & (1 << 7))
		trace_printf("Reset low power\n");
	if (whyReset & (1 << 6))
		trace_printf("Reset WWD\n");
	if (whyReset & (1 << 5))
		trace_printf("Reset IWD\n");
	if (whyReset & (1 << 4))
		trace_printf("Reset software\n");
	if (whyReset & (1 << 3))
		trace_printf("Reset POR/PDR\n");
	if (whyReset & (1 << 2))
		trace_printf("Reset NRSTpin\n");
	if (whyReset & (1 << 1))
		trace_printf("Reset OBL\n");
}

void initGeneralIOPorts() {
	GPIO_InitTypeDef GPIO_InitStructure;

	/* GPIOA and B Periph clock were enabled in main */

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

}

void performPrecisionADC() {
	ADC_DMA_init(ADCUnloadedValues);

	// Danger, this requires that there is an interrupt to have us recover from the sleep!
	while (!ADC_DMA_Complete) {
		PWR_EnterSleepMode(PWR_Regulator_ON, PWR_SLEEPEntry_WFI);
	}

	ADC_DMA_shutdown();

	batteryVoltage = PHY_batteryUnloadedVoltage();
	solarVoltage = PHY_solarVoltage();
	temperature = PHY_temperature();
	simpleTemperature = PHY_simpleTemperature(temperature);
	// internalTemperature = PHY_internalTemperature();
}

void calibrationCycle() {
	// This may (be warned) kill the GPS.
	// Reason is that the GPS is so noisy on the supply that the PLL runs rough.
	// WUT interrupt will also be disabled if RTC is calibrated (currently disabled).
	currentCalibration = getCalibration(simpleTemperature, true);

	// As a side effect, fix the RTC.
	// RTC_setCalibration(currentCalibration->RTCNeededCorrectionPP10M);
}

/*
 * Squeeze all relevant info out of GPS, including:
 * - Time
 * - Position
 * - Calibration if not already having one for current temperature
 * The GPS is left running (if it was ever started)
 */
void GPSCycle() {
	// Just in case. it bothers the GPS pretty bad. And it should never be running now anyway.
	PLL_shutdown();

	if (PWR_isSafeToUseDevice(E_DEVICE_GPS)) {
		GPS_start();
		uint32_t gpsStartTime = systemTimeMillis;

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

		lastGPSFixTime = (systemTimeMillis - gpsStartTime) / 1000;

		// Update APRS map
		APRS_frequenciesFromPosition(&lastNonzeroPosition, latestAPRSRegions,
				latestAPRSCores);

		// This will trigger a new calibration if necessary.
		calibrationCycle();

		GPS_shutdown();
	}
}

void doWSPR() {
	if (!PWR_isSafeToUseDevice(E_DEVICE_HF_TX))
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

			PWR_startDevice(E_DEVICE_HF_TX);
			WSPR_Transmit(THIRTY_M, &pllSetting, WSPR_DEVIATION_DAC_30m);
			PWR_stopDevice(E_DEVICE_HF_TX);

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

void HF_APRSCycle() {
	if (!PWR_isSafeToUseDevice(E_DEVICE_HF_TX)) {
		trace_printf("Unsafe to HF tx\n");
		return;
	}

	APRS_transmitMessage(HF, COMPRESSED_POSITION_MESSAGE, 10149300,
			currentCalibration->transmitterOscillatorFrequencyAtDefaultTrim);
}

void VHF_APRSCycle() {
	boolean inCoreCoverage = false;

	// Send position on all relevant frequencies, without a break in between.
	for (uint8_t i = 0;
			i < APRS_WORLD_MAP_LENGTH && PWR_isSafeToUseDevice(E_DEVICE_VHF_TX);
			i++) {
		if (latestAPRSRegions[i]) {
			trace_printf("Sending a COMPRESSED_POSITION_MESSAGE on %d\n",
					APRS_WORLD_MAP[i].frequency * 1000);
			APRS_transmitMessage(VHF, COMPRESSED_POSITION_MESSAGE,
					APRS_WORLD_MAP[i].frequency * 1000,
					PLL_XTAL_NOMINAL_FREQUENCY);

		}
		if (latestAPRSCores[i])
			inCoreCoverage = true;
	}

// Send status on all relevant frequencies, without a break in between but with a break before.
// If we are not in network coverage, don't bother to try or to delay first.
	if (inCoreCoverage && PWR_isSafeToUseDevice(E_DEVICE_VHF_TX)) {
		timer_sleep(2000);
		for (uint8_t i = 0; i < APRS_WORLD_MAP_LENGTH; i++) {
			if (latestAPRSRegions[i]) {
				trace_printf("Sending a STATUS_MESSAGE on %d\n",
						APRS_WORLD_MAP[i].frequency * 1000);
				APRS_transmitMessage(VHF, STATUS_MESSAGE,
						APRS_WORLD_MAP[i].frequency * 1000,
						PLL_XTAL_NOMINAL_FREQUENCY);

			}
		}

		timer_sleep(2000);
		// Get rid of some storage backlog, first take that scheduled for a 2nd transmission

		int maxStored = 5;

		StoredPathRecord_t* storedRecord;
		while (hasRecordOutForLastTransmission() && maxStored-- > 0) {
			trace_printf("Dumping a record 2nd time\n");
			timer_sleep(1000);
			storedRecord = nextRecordOutForLastTransmission();
			for (uint8_t i = 0; i < APRS_WORLD_MAP_LENGTH; i++) {
				if (latestAPRSCores[i])
					APRS_transmitStoredMessage(VHF, storedRecord,
							APRS_WORLD_MAP[i].frequency * 1000,
							PLL_XTAL_NOMINAL_FREQUENCY);
			}
		}

		maxStored = 5;

		// Get rid of some storage backlog, 1st transmission
		while (hasRecordOutForFirstTransmission() && maxStored-- > 0) {
			trace_printf("Dumping a record 1st time\n");
			timer_sleep(1000);
			storedRecord = nextRecordOutForFirstTransmission();
			for (uint8_t i = 0; i < APRS_WORLD_MAP_LENGTH; i++) {
				if (latestAPRSCores[i])
					APRS_transmitStoredMessage(VHF, storedRecord,
							APRS_WORLD_MAP[i].frequency * 1000,
							PLL_XTAL_NOMINAL_FREQUENCY);
			}
		}
	} else { // Not core.
		trace_printf("Outside APRS core or VHF unsafe to run\n");
		// Store a message.
		if (timeToStoreRecord()) {
			trace_printf("Storing a record\n");
			StoredPathRecord_t* recordToStore = nextRecordIn();
			storeToRecord(recordToStore);
		} else {
			trace_printf("Not storing now\n");
		}
	}
}

void radioCycle() {
	// Just do VHF in any case.
	VHF_APRSCycle();

	hfPacketCounter++;
	if (hfPacketCounter >= HF_PACKET_DIVIDER) {
		hfPacketCounter = 0;
		HF_APRSCycle();
	}

	wsprCounter++;
	if (wsprCounter >= WSPR_DIVIDER) {
		wsprCounter = 0;
		doWSPR();
	}
}

void reschedule(char _scheduleName, uint8_t _mainPeriodWakeupCycles) {
	if (_scheduleName != scheduleName) {
		trace_printf("Rescheduling : %c\n", _scheduleName);
		// don't do this. If bouncing back and forth between 2 different schedules
		// because of unstable temperature or whatever, there will never be any main cycles.
		// mainPeriodCnt = 0;
	}
	scheduleName = _scheduleName;
	mainPeriodWakeupCycles = _mainPeriodWakeupCycles;
}

extern void SetSysClock(void);

void wakeupCycle() {
	performPrecisionADC();

	trace_printf("ADCValue0: %d\t", ADCUnloadedValues[0]);
	trace_printf("ADCValue1: %d\t", ADCUnloadedValues[1]);
	trace_printf("ADCValue2: %d\t", ADCUnloadedValues[2]);

	trace_printf("Precision batt voltage: %d\n", (int) (1000 * batteryVoltage));
	trace_printf("Ext mC: %d\n", (int) (1000 * temperature));

	// Are we alive at all?
	if (batteryVoltage >= 3.0) {
		// Are we in a very good shape?
		if (isDaytimePower()) {
			// trace_printf("Good batt\n");
			if (lastNonzero3DPosition.alt
					!= 0&& lastNonzero3DPosition.alt < LOWALT_THRESHOLD) {
				reschedule(LOWALT_SCHEDULE_TIME);
			} else {
				reschedule(DAY_SCHEDULE_TIME);
			}
			// Just Oki Doki state?
		} else if (batteryVoltage >= 3.3) {
			// This should not happen in daytime anyway...
			// trace_printf("OK batt\n");
			reschedule(NIGHT_SCHEDULE_TIME);
			// Pretty bad?
		} else {
			// trace_printf("Low batt\n");
			reschedule(LOWBATT_SCHEDULE_TIME);
		}

		// Time to do main cycle? (the variables should have been set somewhere above, all of them)
		trace_printf("To go until main %d\n",
				mainPeriodWakeupCycles - mainPeriodCounter);
		mainPeriodCounter++;
		if (mainPeriodCounter >= mainPeriodWakeupCycles) {
			mainPeriodCounter = 0;

			// start the real HSE clock.
			SetSysClock();

			// do some radio work :)
			GPSCycle();
			radioCycle();
		}
	} else { // we are below 3.0
		trace_printf("Battery too low for main-stuff (<3)\n");
		reschedule(CRISIS_SCHEDULE_TIME);
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

	/* Configure one bit for preemption priority */
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);

	initGeneralIOPorts();

	numRestarts++;
	if (numRestarts > 99)
		numRestarts = 0;

// for debug only.
#if MODE == GROUNDTEST
	systick_start();
	timer_sleep(3000);
#endif

	RTC_init();
	RTC_scheduleDailyEvent();
	RTC_setWakeup(RTC_WAKEUP_PERIOD_S);

	if (DEFEAT_VOLTAGE_CHECKS) {
		invalidateStartupLogs();
	}

	while (1) {
		// TODO something depends on systick running (or else gets stuck), what is it?
		systick_start();
		wakeupCycle();
		systick_end();

		// Prepare sleep. Clear any used-up RTC event out's.
		// Really, both of these 2 steps were found to be needed. No monkeying.
		PWR_RTCAccessCmd(ENABLE);
		RTC_ClearITPendingBit(RTC_IT_WUT | RTC_IT_ALRA);
		PWR_RTCAccessCmd(DISABLE);

		trace_printf("Sleeping...\n");

		PWR_EnterSTOPMode(PWR_Regulator_ON, PWR_STOPEntry_WFE);

		// GPIOB->ODR ^= GPIO_Pin_7;
	}
}
