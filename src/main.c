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
#include "RTC.h"
#include "GPS.h"
#include "StabilizedOscillator.h"

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

	/* Configure PD0 and PD1 or PD3 and PD7 in output pushpull mode */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_0;

	// The 1 LED port
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	// LED on (portB.6) .. the library way to do IO (As opposed to above direct way)
	// GPIO_WriteBit(GPIOB, GPIO_Pin_6, Bit_SET);
}

int main() {
	/*!< At this stage the microcontroller clock setting is already configured,
	 this is done through SystemInit() function which is called from startup
	 file (startup_stm32l1xx_xx.s) before to branch to application main.
	 To reconfigure the default setting of SystemInit() function, refer to
	 system_stm32l1xx.c file
	 */

	//uint32_t bkup = RTC_ReadBackupRegister(RTC_BKP_DR0);
	//trace_printf("BKUP was %u\n", bkup);
	//RTC_WriteBackupRegister(RTC_BKP_DR0, bkup + 1);
	initGeneralIOPorts();

	// Fire up systick
	timer_start();
	RTC_init();

	timer_sleep(3000);
	//uint16_t simpleBatteryVoltage = ADC_cheaplyMeasureBatteryVoltage();
	//trace_printf("Start. Battery was %d\n", simpleBatteryVoltage);

	ADC_DMA_init(ADCUnloadedValues);
	while (!ADC_DMA_Complete)
		;

	trace_printf("ADCValue0: %d\n", ADCUnloadedValues[0]);
	trace_printf("ADCValue1: %d\n", ADCUnloadedValues[1]);
	trace_printf("ADCValue2: %d\n", ADCUnloadedValues[2]);

	// This should NOT be needed!
	// ADC_DMA_shutdown();

	//ADC_DMA_init(ADCLoadedValues);
	//ADC_DMA_start(ADCLoadedValues);
	//while (!ADC_DMA_Complete)
	//	;

	 /*
	 trace_printf("ADCValue0: %d\n", ADCLoadedValues[0]);
	 trace_printf("ADCValue1: %d\n", ADCLoadedValues[1]);
	 trace_printf("ADCValue2: %d\n", ADCLoadedValues[2]);
	 ADC_DMA_shutdown();

	 myBuffer[0] = myBuffer[0] + 1;
	 trace_printf("My funny var is now: %u\n", myBuffer[0]);
	 */
	 trace_printf("Batt voltage: %d\n",
	 (int) (1000 * batteryVoltage(ADCUnloadedValues[0])));
	 trace_printf("Temperature mC: %d\n",
	 (int) (1000 * temperature(ADCUnloadedValues[2])));

	 /*
	GPS_init();

	if (GPS_waitForTimelock(300000)) {
		trace_printf("GPS timelock okay\n");
		setRTC(&nmeaTimeInfo.date, &nmeaTimeInfo.time);
	}
	*/

	// uint8_t success = oscillatorCalibration(300000, &fsys);
	// if (!success || fsys < 16E6 - 1600 || fsys > 16E6 + 1600) {
	// Disregard, it's too absurd a claim of imprecision
	//	trace_printf("Unrealistic fsys: status %u, ignore (%u, %u, %u, %u).\n",
	//h(int) fsys, success, 16E6 - 1600, 16E6 + 1600, SystemCoreClock);
//		fsys = 16E6;
//	} else {
//		trace_printf("fsys %u\n", fsys);
//	}

	//uint32_t nRTC = 0;
	//success = RTCCalibration(300000, &nRTC);
	//trace_printf("RTC cal: status %u, period %u\n", success, nRTC);

	//double RTCSpeedFactor = (double)fsys / (double)nRTC;
	//RTC_setCalibration(RTCSpeedFactor);

	// trace_printf("Waiting for position\n");
	// GPS_waitForPosition(300000);

	// A very good position is a luxury that we don't care too much about.
	// trace_printf("Waiting for HP position\n");
	// GPS_waitForPrecisionPosition(300000);

	// boolean test[8];
	// APRS_determineFrequencyFromPosition(&nmeaPositionInfo, test);
	// APRS_debugFrequency(test);
	// GPS_shutdown();

	// selfCalibrateForWSPR(fsys, 0.00003);

	WSPRSynthesisExperiment(25998440);

	// APRS_debugWorldMap();

	// Finito with GPS. Well we might want to have a position too.
	// GPS_shutdown();

	//aprs_compressedMessage(123, 28);

	/*
	 * 	double modulation[2] = { 0, 0 };
	 uint16_t count = 0;
	 *
	 */
	// RTC_TimeTypeDef rtcTime;
	// RTC_GetTime(RTC_Format_BIN, &rtcTime);
	// scheduleASAPAlarmInSlot(1);
	// setWakeup(10);
	while (1) {
		timer_sleep(10000);
		debugGPSTime();
		debugRTCTime();
		trace_printf("\n");
		/*
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
}

/**
 * @brief  Delay Function.
 * @param  nCount:specifies the Delay time length.
 * @retval None
 */
void Delay(__IO uint32_t nCount) {
	while (nCount--) {
	}
}

#ifdef  USE_FULL_ASSERT

/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t* file, uint32_t line)
{
	/* User can add his own implementation to report the file name and line number,
	 ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

	/* Infinite loop */
	while (1)
	{
	}
}
#endif

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
