/*
 * ADC_DAC.c
 *
 *  Created on: Mar 9, 2015
 *      Author: dongfang
 */
//#include <stm32l1xx_conf.h>
#include "stm32l1xx_conf.h"
#include <diag/Trace.h>
#include "systick.h"

	// PA1, PA2 and PB15 are analog inputs
/*
void initADC() {
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

	// Correct??
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

	GPIO_InitTypeDef GPIO_InitStructureIn;
	GPIO_InitStructureIn.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2;
	GPIO_InitStructureIn.GPIO_Mode = GPIO_Mode_AN;
	GPIO_Init(GPIOA, &GPIO_InitStructureIn);

	/ * basic setup
	 ADC_InitTypeDef ADC_init;
	 ADC_StructInit(&ADC_init);
	 ADC_Init(ADC1, &ADC_Init);
	 * /

	/ * Fancier setup * /
	ADC_InitTypeDef ADC_InitStructure;
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
	//We will convert multiple channels
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;
	//select continuous conversion mode
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE; //!
	//select no external triggering
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	//right 12-bit data alignment in ADC data register
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	//8 channels conversion
	ADC_InitStructure.ADC_NbrOfConversion = 2;
	//load structure values to control and status registers
	ADC_Init(ADC1, &ADC_InitStructure);
}
*/

/*
Seems unused.
void initDAC() {
	// Init the DAC-OUT pin
	GPIO_InitTypeDef GPIO_InitStructureIn;
	GPIO_InitStructureIn.GPIO_Pin = GPIO_Pin_4;
	// GPIO_InitStructureIn.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructureIn.GPIO_Mode = GPIO_Mode_AN;
	// GPIO_InitStructureIn.GPIO_OType = GPIO_OType_OD
	GPIO_Init(GPIOA, &GPIO_InitStructureIn);
	//	DAC->CR

	DAC_InitTypeDef DAC_init;
	DAC_init.DAC_OutputBuffer = DAC_OutputBuffer_Enable;

	DAC_DHR12R1_DACC1DHR;
	DAC_Init(0, &GPIO_InitStructureIn);
}
*/

