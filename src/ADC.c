/*
 * ADC_DMA.c
 *
 *  Created on: May 14, 2015
 *      Author: dongfang
 */

/* Includes ------------------------------------------------------------------*/

#include <ADC.h>
#include <misc.h>
#include <stm32l1xx.h>
#include <stm32l1xx_adc.h>
#include <stm32l1xx_dma.h>
#include <stm32l1xx_gpio.h>
#include <stm32l1xx_pwr.h>
#include <stm32l1xx_rcc.h>
#include <diag/Trace.h>
#include "LED.h"

/* Private define ------------------------------------------------------------*/
#define ADC1_DR_ADDRESS    ((uint32_t)0x40012458)

/* Private variables ---------------------------------------------------------*/
volatile uint16_t ADCBeforeLoadPowerValues[NUM_ADC_POWER_VALUES];
volatile uint16_t ADCAfterGPSPowerValues[NUM_ADC_POWER_VALUES];
volatile uint16_t ADCAfterHFPowerValues[NUM_ADC_POWER_VALUES];
volatile uint16_t ADCTemperatureValues[NUM_ADC_TEMPERATURE_VALUES];
volatile uint16_t ADC_VDDValues[NUM_ADC_VDD_VALUES];

volatile boolean ADC_DMA_Complete;
/* @brief  Configure the ADC1 using DMA channel1.
 * @param  None
 * @retval None
 */
void ADC_DMA_init(ADC_MEASUREMENT_t measurement) {
	uint8_t numConversions;
	volatile uint16_t* targetBuffer;

	switch (measurement) {
	case ADC_MEASUREMENT_POWER_BEFORELOAD:
		numConversions = NUM_ADC_POWER_VALUES;
		targetBuffer = ADCBeforeLoadPowerValues;
		break;
	case ADC_MEASUREMENT_POWER_AFTERGPS:
		numConversions = NUM_ADC_POWER_VALUES;
		targetBuffer = ADCAfterGPSPowerValues;
		break;
	case ADC_MEASUREMENT_POWER_AFTERHF:
		numConversions = NUM_ADC_POWER_VALUES;
		targetBuffer = ADCAfterHFPowerValues;
		break;
	case ADC_MEASUREMENT_TEMPERATURE:
		numConversions = NUM_ADC_TEMPERATURE_VALUES;
		targetBuffer = ADCTemperatureValues;
		break;
	case ADC_MEASUREMENT_VDD:
		numConversions = NUM_ADC_VDD_VALUES;
		targetBuffer = ADC_VDDValues;
		break;
	}

	/*----------------- ADC1 configuration with DMA enabled --------------------*/
	ADC_TempSensorVrefintCmd(ENABLE);

	/* Enable the HSI oscillator. ADC runs on that (only) on the L151. */
	RCC_HSICmd(ENABLE);

	GPIO_InitTypeDef GPIO_InitStructure;
	// 6: Solar volt, 7: Batt volt, 2: Temperature
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* Check that HSI oscillator is ready */
	while (RCC_GetFlagStatus(RCC_FLAG_HSIRDY) == RESET) {
		trace_printf("HSI dead?\n");
	}

	/* Enable ADC1 clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
	/*------------------------ DMA1 configuration ------------------------------*/
	/* Enable DMA1 clock */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

	/* DMA1 channel1 configuration */
	DMA_InitTypeDef DMA_InitStructure;
	DMA_DeInit(DMA1_Channel1);
	DMA_InitStructure.DMA_PeripheralBaseAddr = ADC1_DR_ADDRESS;
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t) targetBuffer;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
	DMA_InitStructure.DMA_BufferSize = numConversions;
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

	/* Enable the request after last transfer for DMA Circular mode */
	ADC_DMARequestAfterLastTransferCmd(ADC1, ENABLE);

	/* Enable ADC1 DMA */
	ADC_DMACmd(ADC1, ENABLE);

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

	/* ADC1 configuration */
	ADC_InitTypeDef ADC_InitStructure;
	ADC_StructInit(&ADC_InitStructure);
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;
	ADC_InitStructure.ADC_NbrOfConversion = numConversions;

    ADC_Cmd(ADC1, DISABLE);
	ADC_Init(ADC1, &ADC_InitStructure);

	switch (measurement) {
	case ADC_MEASUREMENT_POWER_BEFORELOAD:
	case ADC_MEASUREMENT_POWER_AFTERGPS:
	case ADC_MEASUREMENT_POWER_AFTERHF:
		ADC_RegularChannelConfig(ADC1, ADC_Channel_7, 1,
		ADC_SampleTime_48Cycles); // Batt
		ADC_RegularChannelConfig(ADC1, ADC_Channel_6, 2,
		ADC_SampleTime_48Cycles); // Main solar

		break;
	case ADC_MEASUREMENT_TEMPERATURE:
		for (int i = 1; i <= NUM_ADC_TEMPERATURE_VALUES; i++) {
			ADC_RegularChannelConfig(ADC1, ADC_Channel_TempSensor, i,
			ADC_SampleTime_384Cycles); // Internal T
		}
		break;

	case ADC_MEASUREMENT_VDD:
		for (int i = 1; i <= NUM_ADC_VDD_VALUES; i++) {
			ADC_RegularChannelConfig(ADC1, ADC_Channel_Vrefint, i,
			ADC_SampleTime_192Cycles); // Voltage ref.
		}
		break;
	}

	/* Enable ADC1 */
	ADC_Cmd(ADC1, ENABLE);

	/* Wait until the ADC1 is ready */
	while (ADC_GetFlagStatus(ADC1, ADC_FLAG_ADONS) == RESET) {
	}

	/* Start ADC1 Software Conversion */
	ADC_SoftwareStartConv(ADC1);
}

void ADC_measurement_blocking(ADC_MEASUREMENT_t measurement) {
// Danger, this requires that there is an interrupt to have us recover from the sleep!
	//while (!ADC_DMA_Complete) {
	//	PWR_EnterSleepMode(PWR_Regulator_ON, PWR_SLEEPEntry_WFI);
	//}
	ADC_DMA_init(measurement);

	while (!ADC_DMA_Complete) {
		PWR_EnterSleepMode(PWR_Regulator_ON, PWR_SLEEPEntry_WFI);
	}

	ADC_DMA_shutdown();
}

void ADC_DMA_shutdown() {
	ADC_TempSensorVrefintCmd(DISABLE);

	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;
	NVIC_Init(&NVIC_InitStructure);

	DMA_ITConfig(DMA1_Channel1, DMA_IT_TC, DISABLE);

	/* Disable ADC1 */
	ADC_Cmd(ADC1, DISABLE);

	/* Disable ADC1 DMA */
	ADC_DMACmd(ADC1, DISABLE);

	/* Disable the request after last transfer for DMA Circular mode */
	ADC_DMARequestAfterLastTransferCmd(ADC1, DISABLE);

	DMA_Cmd(DMA1_Channel1, DISABLE);

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, DISABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, DISABLE);

	RCC_HSICmd(DISABLE);
}

void DMA1_Channel1_IRQHandler(void) {
	/* Test on DMA Transfer Complete interrupt */
	if (DMA_GetITStatus(DMA1_IT_TC1)) {
		DMA_ClearITPendingBit(DMA1_IT_TC1);
		ADC_DMA_Complete = true;
	}
}

