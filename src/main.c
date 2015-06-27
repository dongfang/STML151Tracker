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
#include <diag/Trace.h>
#include <stdint.h>
#include <math.h>

#include "Types.h"
#include "systick.h"
#include "ADC.h"
#include "WSPR.h"
#include "RTC.h"
#include "GPS.h"
#include "StabilizedOscillator.h"
#include "APRSWorldMap.h"
#include "Power.h"
#include "SelfCalibration.h"
#include "RecordStorage.h"

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

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_0;

	// The 1 LED port
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
}

void performPrecisionADC() {
	ADC_DMA_init(ADCUnloadedValues);

	// TODO: Power save sleep.
	while (!ADC_DMA_Complete)
		;

	trace_printf("ADCValue0: %d\n", ADCUnloadedValues[0]);
	trace_printf("ADCValue1: %d\n", ADCUnloadedValues[1]);
	trace_printf("ADCValue2: %d\n", ADCUnloadedValues[2]);

	ADC_DMA_shutdown();

	trace_printf("Batt voltage: %d\n",
			(int) (1000 * batteryVoltage(ADCUnloadedValues[0])));
	trace_printf("Temperature mC: %d\n",
			(int) (1000 * temperature(ADCUnloadedValues[2])));
}

/*
 * Squeeze all relevant info out of GPS, including:
 * - Time
 * - Position
 * - Calibration if not already having one for current temperature
 */
void GPSCycle(uint8_t simpleBatteryVoltage, int8_t temperature) {
	if (isSafeToUseGPS(simpleBatteryVoltage)) {
		startGPS(simpleBatteryVoltage);
		GPS_init();

		if (GPS_waitForTimelock(300000)) {
			setRTC(&nmeaTimeInfo.date, &nmeaTimeInfo.time);
		}

		if (REQUIRE_HIGH_PRECISION_POSITIONS
				&& simpleBatteryVoltage >= COARSE_BATT_ADC_3V3) {
			trace_printf("Waiting for HP position\n");
			GPS_waitForPrecisionPosition(
			REQUIRE_HIGH_PRECISION_MAX_TIME_S * 1000);
		}

		// This will (be warned) kill the GPS.
		// Reason is that the GPS is such a hog on the supply that the PLL runs rough.
		// WUT interrupt will also be disabled (check if that is really needed!).
		const CalibrationRecord_t* cal = getCalibration(temperature, true);

		// As a side effect, fix the RTC.
		RTC_setCalibration(cal->RTCNeededCorrectionPP10M);

		// If calibration was not done above, just make sure the stupid thing is off.
		GPS_shutdown();
		// Store the fact that we successfully used GPS.
		stopGPS();
	} else {
		trace_printf(
				"Not starting GPS right now, it didn't go well last time\n");
	}
}

void WSPRCycle(int8_t temperature) {
	const CalibrationRecord_t* cal = getCalibration(temperature, false);
	while (1) {
		prepareWSPRMessage(1, REAL_EXTENDED_LOCATION, 3.5);
		RTC_waitTillModuloMinutes(2,1);
		WSPR_transmit(THIRTY_M, cal->transmitterOscillatorFrequencyAtDefaultTrim, 32);
	}
}

void commsCycle() {

}

void onWakeup() {
	uint16_t simpleBatteryVoltage = ADC_cheaplyMeasureBatteryVoltage();
	trace_printf("Simple voltage %d\n", simpleBatteryVoltage);
	int8_t simpleTemperature;

	if (simpleBatteryVoltage >= COARSE_BATT_ADC_3V0) {
		performPrecisionADC();
		simpleTemperature = ADC_simpleTemperature();
	}

	if (simpleBatteryVoltage >= COARSE_BATT_ADC_3V7) {
		trace_printf("Good batt\n");
		GPSCycle(simpleBatteryVoltage, simpleTemperature);
		WSPRCycle(simpleTemperature);

		// Do the GPS acq and calibration
		// Check and tx APRS
		// Check and tx WSPR
		// Reschedule normal
	} else if (simpleBatteryVoltage >= COARSE_BATT_ADC_3V3) {
		trace_printf("OK batt\n");
		GPSCycle(simpleBatteryVoltage, simpleTemperature);
		// Do the GPS acq and calibration
		// Check and tx APRS
		// Reschedule slow
	} else if (simpleBatteryVoltage >= COARSE_BATT_ADC_3V0) {
		trace_printf("Low batt\n");
		// Transmit APRS without GPS (using old position to decide, and assuming no core)
	} else {
		trace_printf("VERY low batt %d, gng back to sleep.\n",
				simpleBatteryVoltage);
	}
}

int main() {
	/*!< At this stage the microcontroller clock setting is already configured,
	 this is done through SystemInit() function which is called from startup
	 file (startup_stm32l1xx_xx.s) before to branch to application main.
	 To reconfigure the default setting of SystemInit() function, refer to
	 system_stm32l1xx.c file
	 */

	initGeneralIOPorts();

	// Fire up systick
	timer_start();
	RTC_init();

	timer_sleep(4000);

	if (DEFEAT_VOLTAGE_CHECKS) {
		invalidateStartupLog();
	}

	onWakeup();

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
