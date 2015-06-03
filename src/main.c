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
#include <math.h>
#include "stm32l1xx_conf.h"
#include "CDCEL913.h"
#include "Si4463_SPI.h"
#include "DAC_DMA_SignalGeneration.h"
#include "systick.h"
#include "WSPR.h"
#include "stm32l1xx_pwr.h"
#include "ADC_DMA.h"
#include "GPS.h"
#include "RTC.h"
#include "SelfCalibration.h"
#include "DataTypes.h"

uint8_t band = 0;
uint8_t wsprMessageType = 3;

extern void WSPR_TransmitCycle(
		uint8_t band,
		const CDCEL913_PLL_Setting_t* setting,
		int staticCorrection, int stepModulation);

float temperature(uint16_t ADCvalue) {
	float vTemp_mV = ADCvalue * 2200 / 4096;
	float rootexp = 5.506 * 5.506 + 4 * 0.00176 * (870.6 - vTemp_mV);
	float temp = (5.506 - sqrt(rootexp)) / (2 * -0.00176) + 30.0;
	return temp;
}

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

int main() {
	/*!< At this stage the microcontroller clock setting is already configured,
	 this is done through SystemInit() function which is called from startup
	 file (startup_stm32l1xx_xx.s) before to branch to application main.
	 To reconfigure the default setting of SystemInit() function, refer to
	 system_stm32l1xx.c file
	 */

	uint32_t bkup = RTC_ReadBackupRegister(RTC_BKP_DR0);
	trace_printf("BKUP was %u\n", bkup);
	RTC_WriteBackupRegister(RTC_BKP_DR0, bkup + 1);

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
	GPIO_WriteBit(GPIOB, GPIO_Pin_6, Bit_SET);

	// Fire up systick
	timer_start();

	timer_sleep(2000);
	trace_printf("Start\r\n");

	GPS_init();
	RTC_init();

	if (GPS_waitForTimelock(300000)) {
		trace_printf("GPS timelock okay\n");
		setRTC(&nmeaTimeInfo.date, &nmeaTimeInfo.time);
	}

	uint32_t fsys = 0;
	uint8_t success = getOscillatorCalibration(300000, &fsys);
	if (!success || fsys < 16E6 - 1600 || fsys > 16E6 + 1600) {
		// Disregard, it's too absurd a claim of imprecision
		trace_printf("Unrealistic fsys: status %u, ignore (%u, %u, %u, %u).\n",
				(int) fsys, success, 16E6 - 1600, 16E6 + 1600, SystemCoreClock);
		fsys = 16E6;
	} else {
		trace_printf("fsys %u\n", fsys);
	}

	trace_printf("Waiting for position\n");
	GPS_waitForPosition(10000);

	// A very good position is a luxury that we don't care too much about.
	trace_printf("Waiting for HP position\n");
	GPS_waitForPrecisionPosition(10000);

	// Finito with GPS. Well we might want to have a position too.
	GPS_shutdown();

	/*
	 * 	double modulation[2] = { 0, 0 };
	 uint16_t count = 0;
	 *
	 */
	RTC_TimeTypeDef rtcTime;
	double deviationMeasured;
	double oscillatorFrequencyMeasured;

//	scheduleASAPAlarmInSlot(2);
	while (1) {
		CDCEL913_init();
		CDCEL913_setDirectModeWithDivision(CDCEL_TRIM_PF);

		timer_sleep(100);

		RTC_GetTime(RTC_Format_BIN, &rtcTime);

		trace_printf("Self-cal at %02u:%02u:%02u\n",
				rtcTime.RTC_Hours, rtcTime.RTC_Minutes, rtcTime.RTC_Seconds);

		uint8_t selfCalSuccess = selfCalibrateModulation(
				20000,
				fsys,
				&deviationMeasured,
				&oscillatorFrequencyMeasured);

		trace_printf("Self-cal success : %u\n", selfCalSuccess);
		trace_printf("Self-cal deviation PPB : %d\n",
				(int) (deviationMeasured * 1.0E9));
		trace_printf("Self-cal osc freq: %d\n",
				(int) oscillatorFrequencyMeasured);

		double targetFrequency = WSPRFrequencies[band];

		double DACStepsPerHz = SELF_CALIBRATION_MODULATION_AMPL_PP
				/ (deviationMeasured * targetFrequency);

		double stepModulation = 12000.0 * DACStepsPerHz / 8192.0;
		trace_printf("%d DAC steps per Hz and %d per WSPR step\n", (int)DACStepsPerHz, (int)stepModulation);

		double desiredMultiplication = targetFrequency / oscillatorFrequencyMeasured;
		uint8_t bestSettingIndex = CDCEL913_BestSettingIndex(band, desiredMultiplication);
		const CDCEL913_PLL_Setting_t* bestSetting = &PLL_SETTINGS[band][bestSettingIndex];

		double bestFrequency = bestSetting->mul*oscillatorFrequencyMeasured;

		trace_printf("Best setting found: n=%u N=%u, f=%u\n", bestSettingIndex, bestSetting->N,
				(uint32_t)bestFrequency);

		int32_t lastCorrection = (targetFrequency - bestFrequency) * DACStepsPerHz;

		trace_printf("Final correction on modulation: %d\n", lastCorrection);

		if (lastCorrection > 1500) lastCorrection = 1500;
		else if (lastCorrection < -1500) lastCorrection = -1500;

		do {
			RTC_GetTime(RTC_HourFormat_24, &rtcTime);
		}
		while((rtcTime.RTC_Minutes %2 != 0) ||
				rtcTime.RTC_Seconds != 1);

		trace_printf("WSPR at %02u:%02u:%02u\n",
				rtcTime.RTC_Hours, rtcTime.RTC_Minutes, rtcTime.RTC_Seconds);

		prepareWSPRMessage(wsprMessageType);
		WSPR_TransmitCycle( band, bestSetting, lastCorrection, stepModulation);

		trace_printf("End WSPR!\n");

		// Now we can turn on that stupid thing again.
		// GPS_init();

		// band = 1-band;
		wsprMessageType = 4 - wsprMessageType; // Toggle between 1 and 3
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
