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
float batteryVoltage;
float solarVoltage;

boolean latestAPRSRegions[12]; 	 // 12 is sufficently large for the world map...
boolean latestAPRSCores[12];	 // 12 is sufficently large for the world map...
const CalibrationRecord_t* currentCalibration = &defaultCalibration;

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
	/* TODO: Add WSPR "arm", default to disarm. */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);

	// 6: LED, 1: WSPR arm, 0: SPI1 NSS
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_1 | GPIO_Pin_0;

	// GPIOB stuff: LED, WSPR arm Si4463 SS
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIOB->ODR = GPIO_Pin_0; // LED off, disarm WSPR, disable Si4463 SS.

	/* Configure GPS power pin and Si4463 SDN pin */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_8;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIOA->ODR = GPIO_Pin_0 | GPIO_Pin_8;

	// NIRQ pin from Si4463 as input.
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
}

void performPrecisionADC() {
	ADC_DMA_init(ADCUnloadedValues);

	// TODO: Power save sleep.
	while (!ADC_DMA_Complete)
		;
	/*
	 trace_printf("ADCValue0: %d\t", ADCUnloadedValues[0]);
	 trace_printf("ADCValue1: %d\t", ADCUnloadedValues[1]);
	 trace_printf("ADCValue2: %d\n", ADCUnloadedValues[2]);
	 */
	ADC_DMA_shutdown();

	batteryVoltage = ADC_batteryUnloadedVoltage();
	trace_printf("Precision batt voltage: %d\n", (int) (1000 * batteryVoltage));

	temperature = ADC_temperature();
	simpleTemperature = ADC_simpleTemperature(temperature);
	trace_printf("Temperature mC: %d\n", (int) (1000 * temperature));
}

/*
 * Squeeze all relevant info out of GPS, including:
 * - Time
 * - Position
 * - Calibration if not already having one for current temperature
 */
void GPSCycle() {
	// Just in case. it bothers the GPS pretty bad.
	PLL_shutdown();

	if (isSafeToUseGPS(batteryVoltage)) {
		startGPS(batteryVoltage);
		GPS_init();

		if (GPS_waitForTimelock(300000)) {
			setRTC(&GPSTime.date, &GPSTime.time);
		}

		boolean highPrecisionPositionOkay = true;
		if (REQUIRE_HIGH_PRECISION_POSITIONS && batteryVoltage >= 3.3) {
			trace_printf("Waiting for HP position\n");
			highPrecisionPositionOkay = GPS_waitForPrecisionPosition(
			REQUIRE_HIGH_PRECISION_MAX_TIME_S * 1000);
		}

		// This will (be warned) kill the GPS.
		// Reason is that the GPS is such a hog on the supply that the PLL runs rough.
		// WUT interrupt will also be disabled (check if that is really needed!).
		currentCalibration = getCalibration(simpleTemperature, true);

		// As a side effect, fix the RTC.
		RTC_setCalibration(currentCalibration->RTCNeededCorrectionPP10M);

		// One final chance, just in case the calibration saved the day:
		if (!highPrecisionPositionOkay) {
			trace_printf("Retrying HP position\n");
			highPrecisionPositionOkay = GPS_waitForPrecisionPosition(1000);
		}

		// If calibration was not done above, just make sure the stupid thing is off.
		GPS_shutdown();
		// Store the fact that we successfully used GPS.
		stopGPS();

		// Update APRS map
		APRS_frequenciesFromPosition(&lastNonzeroGPSPosition, latestAPRSRegions,
				latestAPRSCores);

	} else {
		trace_printf(
				"Not starting GPS right now, it didn't go well last time\n");
	}
}

void WSPRCycle() {
	if (!isSafeToUseHFTx(batteryVoltage))
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
			RTC_waitTillModuloMinutes(2, 0);

			prepareWSPRMessage(1, REAL_EXTENDED_LOCATION, batteryVoltage);

			startHFTx(batteryVoltage);
			WSPR_Transmit(THIRTY_M, &pllSetting, 26);
			stopHFTx(batteryVoltage);

			break;
		} else {
			trace_printf(
					"NO feasible PLL setting in range! Should not really happen. Anyway, we try again with more tolerance.\n");
			maxError += 10E-5;
		}
	}
}

void VHF_APRSCycle() {
	if (!isSafeToUseVHFTx(batteryVoltage))
		return;

	boolean isOutsideAPRS = true;
	boolean isOutsideCore = true;

	// Send position on all relevant frequencies, without a break in between.
	for (uint8_t i = 0; i < APRS_WORLD_MAP_LENGTH; i++) {
		if (latestAPRSRegions[i]) {
			isOutsideAPRS = false;
			trace_printf("Sending a COMPRESSED_POSITION_MESSAGE on %d\n", APRS_WORLD_MAP[i].frequency * 1000);
			APRS_transmitRandomMessage(&APRS_TRANSMISSIONS[0],
					COMPRESSED_POSITION_MESSAGE,
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
				trace_printf("Sending a STATUS_MESSAGE on %d\n", APRS_WORLD_MAP[i].frequency * 1000);
				APRS_transmitRandomMessage(&APRS_TRANSMISSIONS[0],
						STATUS_MESSAGE, APRS_WORLD_MAP[i].frequency * 1000,
						currentCalibration->transmitterOscillatorFrequencyAtDefaultTrim);

			}
		}

		if (isOutsideCore) {
			// Store a message.
			StoredPathRecord_t* recordToStore = nextRecordIn();
			storeToRecord(recordToStore);

		} else {
			trace_printf("Ridding some backlog\n");
			// Get rid of some storage backlog, first take that scheduled for a 2nd transmission
			int maxStored = 10;
			StoredPathRecord_t* storedRecord;
			while (hasRecordOutForLastTransmission() && maxStored-- > 0) {
				timer_sleep(1000);
				storedRecord = nextRecordOutForLastTransmission();
				for (uint8_t i = 0; i < APRS_WORLD_MAP_LENGTH; i++) {
					APRS_transmitStoredMessage(&APRS_TRANSMISSIONS[0],
							storedRecord, APRS_WORLD_MAP[i].frequency * 1000,
							currentCalibration->transmitterOscillatorFrequencyAtDefaultTrim);
				}
			}

			maxStored = 10;

			// Get rid of some storage backlon, 1st transmission
			while (hasRecordOutForFirstTransmission() && maxStored-- > 0) {
				timer_sleep(1000);
				storedRecord = nextRecordOutForFirstTransmission();
				for (uint8_t i = 0; i < APRS_WORLD_MAP_LENGTH; i++) {
					APRS_transmitStoredMessage(&APRS_TRANSMISSIONS[0],
							storedRecord, APRS_WORLD_MAP[i].frequency * 1000,
							currentCalibration->transmitterOscillatorFrequencyAtDefaultTrim);
				}
			}
		}
	}
}

void commsCycle() {

}

void onWakeup() {
	performPrecisionADC();

	if (batteryVoltage >= 3.7) {
		trace_printf("Good batt\n");
		GPSCycle();
		VHF_APRSCycle();
		WSPRCycle();

		// Do the GPS acq and calibration
		// Check and tx APRS
		// Check and tx WSPR
		// Reschedule normal
	} else if (batteryVoltage >= 3.3) {
		trace_printf("OK batt\n");
		GPSCycle();
		VHF_APRSCycle();
		WSPRCycle();
		// Do the GPS acq and calibration
		// Check and tx APRS
		// Reschedule slow
	} else if (batteryVoltage >= 3) {
		trace_printf("Low batt\n");
		// Transmit APRS without GPS (using old position to decide, and assuming no core)
	} else {
		trace_printf("VERY low batt %d, gng back to sleep.\n",
				(int) batteryVoltage);
	}
}

int main() {
	/*!< At this stage the microcontroller clock setting is already configured,
	 this is done through SystemInit() function which is called from startup
	 file (startup_stm32l1xx_xx.s) before to branch to application main.
	 To reconfigure the default setting of SystemInit() function, refer to
	 system_stm32l1xx.c file
	 */

	// Fire up systick
	timer_start();
	RTC_init();

	timer_sleep(3000);

	initGeneralIOPorts();

	trace_printf("Mooh again.\n");

	if (DEFEAT_VOLTAGE_CHECKS) {
		invalidateStartupLog();
	}

	while (1) {
		onWakeup();

		/*
		double deviation;
		uint32_t frequency;

		selfCalibrateModulation(16E6, &HF_APRS_SELF_CALIBRATION,
		PLL_PREFERRED_TRIM, &deviation, &frequency);
		trace_printf("SC HFAPRSmod: freq %d, dev %d at %d p-p\n", frequency,
				(int) (deviation * 1E9),
				HF_APRS_SELF_CALIBRATION.modulation_PP);
		 */
		//selfCalibrateModulation(16E6, &WSPR_MODULATION_SELF_CALIBRATION, PLL_PREFERRED_TRIM, &deviation, &frequency);
		//trace_printf("SC WSPRmod: freq %d, dev %d at %d p-p\n", frequency, (int)(deviation*1E9), WSPR_MODULATION_SELF_CALIBRATION.modulation_PP);

		/* APRS_transmitMessage(
		 &APRS_TRANSMISSIONS[0],
		 COMPRESSED_POSITION_MESSAGE,
		 144800000,
		 currentCalibration->transmitterOscillatorFrequencyAtDefaultTrim);
		 timer_sleep(300000);
		 */
	}

	double deviation;
	uint32_t frequency;

	while (1) {
		for (uint8_t i = 0; i < 21; i++) {
			selfCalibrateTrimming(&deviation, i);
			trace_printf("%d pF: %d\n", i, (int) (deviation * 1E7));
		}

		selfCalibrateModulation(16E6, &WSPR_MODULATION_SELF_CALIBRATION, 13,
				&deviation, &frequency);

		trace_printf("WSPR dev %d\n", (int) (deviation * 1E7));

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
