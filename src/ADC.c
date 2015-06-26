/*
 * ADC_DMA.c
 *
 *  Created on: May 14, 2015
 *      Author: dongfang
 */

/* Includes ------------------------------------------------------------------*/

#include "../inc/ADC.h"

#include <math.h>

#include "../Libraries/CMSIS/Device/ST/STM32L1xx/Include/stm32l1xx.h"
#include "../Libraries/STM32L1xx_StdPeriph_Driver/inc/misc.h"
#include "../Libraries/STM32L1xx_StdPeriph_Driver/inc/stm32l1xx_adc.h"
#include "../Libraries/STM32L1xx_StdPeriph_Driver/inc/stm32l1xx_dma.h"
#include "../Libraries/STM32L1xx_StdPeriph_Driver/inc/stm32l1xx_gpio.h"
#include "../Libraries/STM32L1xx_StdPeriph_Driver/inc/stm32l1xx_rcc.h"

/* Private define ------------------------------------------------------------*/
#define ADC1_DR_ADDRESS    ((uint32_t)0x40012458)

/* Private variables ---------------------------------------------------------*/
volatile uint16_t ADCUnloadedValues[NUM_ADC_VALUES];
volatile uint16_t ADCLoadedValues[NUM_ADC_VALUES];

volatile boolean ADC_DMA_Complete;

uint16_t ADC_cheaplyMeasureBatteryVoltage() {
	RCC_HSICmd(ENABLE);

	/* Enable GPIOA clock (is that really needed?? Apparently NOT.) */
	// RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, DISABLE);
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* ADC1 configuration */
	ADC_InitTypeDef ADC_InitStructure;
	ADC_InitStructure.ADC_Resolution = ADC_Resolution_8b; // just pretty coarse.
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfConversion = 1;

	/* Enable ADC1 clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

	/* Check that HSI oscillator is ready */
	while (RCC_GetFlagStatus(RCC_FLAG_HSIRDY) == RESET)
		;
	/* Enable ADC1 */
	ADC_Init(ADC1, &ADC_InitStructure);
	ADC_Cmd(ADC1, ENABLE);

	/* Wait until the ADC1 is ready */
	while (ADC_GetFlagStatus(ADC1, ADC_FLAG_ADONS) == RESET) {
	}

	ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 1, ADC_SampleTime_4Cycles);

	// Start the conversion
	ADC_SoftwareStartConv(ADC1);

	// Wait until conversion completion
	while (ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET)
		;
	// Get the conversion value

	uint16_t result = ADC_GetConversionValue(ADC1);

	/* Enable ADC1 */
	ADC_Cmd(ADC1, DISABLE);

	// RCC_HSICmd(DISABLE);
	// We still haven't disabled HSI again... need do that when we shut down
	return result;
}

/*
 void ADC_DMA_start(volatile uint16_t* conversionTargetArray) {
 ADC_DMA_Complete = false;
 DMA1_Channel1->CMAR = (uint32_t) conversionTargetArray;

 / * Start ADC1 Software Conversion * /
 ADC_SoftwareStartConv(ADC1);
 }

 void ADC_DMA_restart(volatile uint16_t* conversionTargetArray) {
 ADC_DMA_Complete = false;
 DMA1_Channel1->CMAR = (uint32_t) conversionTargetArray;

 ADC_Cmd(ADC1, DISABLE);
 DMA_Cmd(DMA1_Channel1, DISABLE);

 // Wait until DMA is disabled
 while (DMA_GetCmdStatus(DMA1_Channel1))
 ;

 // Clear flag(s) - if any interrupt is enabled, clear the IT flags as well!
 DMA_ClearFlag(DMA1_Channel1, DMA_FLAG_TCIF0);

 DMA_Cmd(DMA1_Channel1, ENABLE);
 ADC_Cmd(ADC1, ENABLE);

 / * Start ADC1 Software Conversion * /
 ADC_SoftwareStartConv(ADC1);
 }
 */

/* @brief  Configure the ADC1 channel18 using DMA channel1.
 * @param  None
 * @retval None
 */
void ADC_DMA_init(volatile uint16_t* conversionTargetArray) {
	/*----------------- ADC1 configuration with DMA enabled --------------------*/
	/* Enable the HSI oscillator. ADC runs on that (only). */
	RCC_HSICmd(ENABLE);

	/* Disable ADC1 (experimental) */
	ADC_Cmd(ADC1, DISABLE);

	/* Disable ADC1 DMA (experimental) */
	ADC_DMACmd(ADC1, DISABLE);

	/*------------------------ DMA1 configuration ------------------------------*/
	/* Enable DMA1 clock */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	/* DMA1 channel1 configuration */
	DMA_InitTypeDef DMA_InitStructure;
	DMA_DeInit(DMA1_Channel1);
	DMA_InitStructure.DMA_PeripheralBaseAddr = ADC1_DR_ADDRESS;
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t) conversionTargetArray;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
	DMA_InitStructure.DMA_BufferSize = NUM_ADC_VALUES;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable; // Read from same address every time.
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA1_Channel1, &DMA_InitStructure);

	/* Enable DMA1 channel1 */
	DMA_Cmd(DMA1_Channel1, ENABLE);

	/* Enable GPIOA clock (is that really needed?? Apparently NOT.) */
	// RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, DISABLE);
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	// RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, DISABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15 /*| GPIO_Pin_12*/;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/* Check that HSI oscillator is ready */
	while (RCC_GetFlagStatus(RCC_FLAG_HSIRDY) == RESET)
		;

	/* Enable ADC1 clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
	/* ADC1 configuration */
	ADC_InitTypeDef ADC_InitStructure;
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;
	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE; //ENABLE;
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfConversion = NUM_ADC_VALUES;
	ADC_Init(ADC1, &ADC_InitStructure);

	/* ADC1 regular channel1 configuration. Ch21 is supposedly the PB15 thermometer. */
	ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 1, ADC_SampleTime_24Cycles); // Batt
	ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 2, ADC_SampleTime_24Cycles); // Main solar
	ADC_RegularChannelConfig(ADC1, ADC_Channel_21, 3, ADC_SampleTime_24Cycles); // Temperature
	// ADC_RegularChannelConfig(ADC1, ADC_Channel_18, 4, ADC_SampleTime_24Cycles); // Tx solar

	/* Enable the request after last transfer for DMA Circular mode */
	ADC_DMARequestAfterLastTransferCmd(ADC1, ENABLE);

	/* Enable ADC1 DMA */
	ADC_DMACmd(ADC1, ENABLE);

	/* Enable ADC1 */
	ADC_Cmd(ADC1, ENABLE);

	/* Wait until the ADC1 is ready */
	while (ADC_GetFlagStatus(ADC1, ADC_FLAG_ADONS) == RESET) {
	}

	/* Set up an interrupt for DMA complete */
	// Enable DMA1 Channel Transfer Complete interrupt
	DMA_ITConfig(DMA1_Channel1, DMA_IT_TC, ENABLE);

	DMA_ClearITPendingBit(DMA1_IT_TC1);
	ADC_DMA_Complete = false;

	DMA_Cmd(DMA1_Channel1, ENABLE); //Enable the DMA1 - Channel1
	NVIC_InitTypeDef NVIC_InitStructure;
	//Enable DMA1 channel IRQ Channel */
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* Start ADC1 Software Conversion */
	ADC_SoftwareStartConv(ADC1);
}

void ADC_DMA_shutdown() {
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;
	NVIC_Init(&NVIC_InitStructure);

	DMA_Cmd(DMA1_Channel1, DISABLE);
	DMA_ITConfig(DMA1_Channel1, DMA_IT_TC, DISABLE);

	/* Enable ADC1 */
	ADC_Cmd(ADC1, ENABLE);

	/* Enable ADC1 DMA */
	ADC_DMACmd(ADC1, ENABLE);

	/* Enable the request after last transfer for DMA Circular mode */
	ADC_DMARequestAfterLastTransferCmd(ADC1, ENABLE);

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, DISABLE);

	DMA_Cmd(DMA1_Channel1, DISABLE);

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, DISABLE);
}

float temperature(uint16_t ADCvalue) {
	float vTemp_mV = ADCvalue * ADC_FACTOR * 1000.0f;
	float rootexp = 5.506 * 5.506 + 4 * 0.00176 * (870.6 - vTemp_mV);//30.43
	float temp = (5.506 - sqrt(rootexp)) / (2 * -0.00176) + 30.0;
	return temp;
}

float batteryVoltage(uint16_t ADCvalue12) {
	return ADCvalue12 * BATT_ADC_VDIV_FACTOR;
}

float solarVoltage(uint16_t ADCvalue12) {
	return ADCvalue12 * SOLAR_ADC_VDIV_FACTOR;
}

void DMA1_Channel1_IRQHandler(void) {
	/* Test on DMA Transfer Complete interrupt */
	if (DMA_GetITStatus(DMA1_IT_TC1)) {
		DMA_ClearITPendingBit(DMA1_IT_TC1);
		ADC_DMA_Complete = true;
		// trace_printf("ACD-DMA xfer done.\n");
	}
}

// 0dB ~ 26mV
// 3V ~ 15.46
// 3.7V ~ 17
// 4.2V ~ 18
// No good!!
// Instead, do: 3V->12, 3.33V->15, 3.66V->18, 4.0V->21, 4.33V->24
uint8_t fake_dBm(float voltage) {
	/*
	voltage = voltage * 3 / (4 * sqrt(2));
	float power_mW = voltage*voltage*1000/72.0;// Z is 72 Ohms
	float dB = log10(power_mW) * 10;
	return dB;
	*/
	float result = (voltage - 3) * 9;
	if (result < 9) result = 9;
	return (uint8_t) result;
}

