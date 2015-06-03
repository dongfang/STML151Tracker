/**
 ******************************************************************************
 * @file    DMA/RAM_DAC/main.c
 * @author  MCD Application Team
 * @version V1.1.1
 * @date    13-April-2012
 * @brief   Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT 2012 STMicroelectronics</center></h2>
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
#include "stm32l1xx_conf.h"
#include <math.h>

extern volatile uint8_t packet[];
extern volatile uint16_t packet_size;
extern volatile uint16_t packet_cnt;

/* Private define ------------------------------------------------------------*/
#define DAC_DHR12RD_Address      0x40007420

#define NUM_SAMPLES 202
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

uint32_t DualSine12bit[NUM_SAMPLES];

/* Private function prototypes -----------------------------------------------*/
void TIM_Config(void);
void DAC_Config(void);
void DMA_Config(void);

/* Private functions ---------------------------------------------------------*/

/**
 * @brief   Main program
 * @param  None
 * @retval None
 */
void DAC_DMA_main(void) {
	/*!< At this stage the microcontroller clock setting is already configured,
	 this is done through SystemInit() function which is called from startup
	 file (startup_stm32l1xx_xx.s) before to branch to application main.
	 To reconfigure the default setting of SystemInit() function, refer to
	 system_stm32l1xx.c file
	 */

	/* Fill Sine12bit table */
	for (uint16_t Idx = 0; Idx < NUM_SAMPLES; Idx++) {
		double p = (double) Idx / NUM_SAMPLES * 3.14159265358979323846 * 2;
		double a = sin(p) * 250.0 + 2048;
		uint16_t v = (uint16_t) (a + 0.5);
		//v = ((double)Idx / NUM_SAMPLES) * 200 + 1024;
		DualSine12bit[Idx] = v + (v << 16);
	}

	/* DMA1 channel3 configuration: DualSine12bit is used as memory base address */
	DMA_Config();

	/* DAC configuration ------------------------------------------------------*/
	DAC_Config();

	/* TIM2 configuration ------------------------------------------------------*/
	TIM_Config();
}

static uint8_t tone;

/**
 * @brief  Configures the TIM2
 * @param  None
 * @retval None
 */
void TIM_Config(void) {
	/* TIM2 Periph clock enable */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	/* Time base configuration, 1200 /s for timer2 */
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.TIM_Period = 11 - 1;
	tone = 0;
	TIM_TimeBaseStructure.TIM_Prescaler = 6 - 1;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0x0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Down;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

	TIM_TimeBaseStructure.TIM_Period = 11 * NUM_SAMPLES - 1;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

	/* TIM2 TRGO selection: update event is selected as trigger for DAC */
	TIM_SelectOutputTrigger(TIM2, TIM_TRGOSource_Update);

	/* TIM3 TRGO selection: update event is selected as trigger for TIM2 control */
	// TIM_SelectOutputTrigger(TIM3, TIM_TRGOSource_Update);
	/* TIM2 enable counter */
	TIM_Cmd(TIM2, ENABLE);

	TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);

	// Set up timer interrupt.
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* TIM3 enable counter */
	TIM_Cmd(TIM3, ENABLE);
}

static const uint8_t DIVIDERS[] = { 6, 11 };

void setTone(uint8_t newTone) {
	if (tone != newTone) {
		tone = newTone;
		TIM2->ARR = DIVIDERS[tone];
	}
}

void sendNRZI(uint8_t newBit) {
	if (!newBit) {
		setTone(!tone);
	}
}

void dummyModulation(void) {
	static uint8_t cnt;
	if (cnt == 0 || cnt == 7)
		sendNRZI(0);
	else
		sendNRZI(1);
	cnt++;
	if (cnt == 8)
		cnt = 0;
}

void packetModulation(void) {
	uint8_t mask = 1<<(packet_cnt & 7);
	uint16_t index = packet_cnt>>3;
	uint8_t bit = packet[index] & mask;
	if (bit) bit = 1;
	sendNRZI(bit);
}

void TIM3_IRQHandler(void) {
	if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET) {
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
		dummyModulation();
	}
}

/**
 * @brief  Configures DAC channel 1 and channel 2
 * @param  None
 * @retval None
 */
void DAC_Config(void) {
	/* Enable GPIOA Periph clock --------------------------------------*/
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
	GPIO_InitTypeDef GPIO_InitStructure;
	/* Configure PA.04 (DAC_OUT1), PA.05 (DAC_OUT2) as analog */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* DAC Periph clock enable */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE);

	DAC_InitTypeDef DAC_InitStructure;
	/* DAC init struct configuration */
	DAC_InitStructure.DAC_Trigger = DAC_Trigger_T2_TRGO;
	DAC_InitStructure.DAC_WaveGeneration = DAC_WaveGeneration_None;
	DAC_InitStructure.DAC_OutputBuffer = DAC_OutputBuffer_Enable;

	/* DAC channel1 Configuration */
	DAC_Init(DAC_Channel_1, &DAC_InitStructure);

	/* DAC channel2 Configuration */
	DAC_Init(DAC_Channel_2, &DAC_InitStructure);

	/* Enable DAC Channel1: Once the DAC channel1 is enabled, PA.04 is
	 automatically connected to the DAC converter. */
	DAC_Cmd(DAC_Channel_1, ENABLE);

	/* Enable DAC Channel2: Once the DAC channel2 is enabled, PA.05 is
	 automatically connected to the DAC converter. */
	DAC_Cmd(DAC_Channel_2, ENABLE);

	/* Enable DMA for DAC Channel2 */
	DAC_DMACmd(DAC_Channel_2, ENABLE);
}

/**
 * @brief  Configures DMA1 channel3
 * @param  None
 * @retval None
 */
void DMA_Config(void) {
	DMA_InitTypeDef DMA_InitStructure;
	/* Enable DMA1 clock -------------------------------------------------------*/
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

	DMA_DeInit(DMA1_Channel3);
	DMA_InitStructure.DMA_PeripheralBaseAddr = DAC_DHR12RD_Address;
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t) &DualSine12bit;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
	DMA_InitStructure.DMA_BufferSize = NUM_SAMPLES;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA1_Channel3, &DMA_InitStructure);

	/* Enable DMA1 Channel3 */
	DMA_Cmd(DMA1_Channel3, ENABLE);
}

// For WSPR (no DMA)
void setupChannel2DAC() {
	// Setup DAC-out port and DAC's clock
	GPIO_InitTypeDef GPIO_InitStructure;
	/* Configure PA.04 (DAC_OUT1), PA.05 (DAC_OUT2) as analog */
	GPIO_InitStructure.GPIO_Pin = /* GPIO_Pin_4 | */ GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE);

	// Set up DAC.
	DAC_InitTypeDef DAC_InitStructure;
	DAC_DeInit();
	DAC_InitStructure.DAC_Trigger = DAC_Trigger_Software;
	DAC_InitStructure.DAC_WaveGeneration = DAC_WaveGeneration_None;
	DAC_InitStructure.DAC_OutputBuffer = DAC_OutputBuffer_Enable;
	DAC_Init(DAC_Channel_2, &DAC_InitStructure);

	DAC_Cmd(DAC_Channel_2, ENABLE);
}


void setDAC2(uint16_t value) {
	DAC_SetChannel2Data(DAC_Align_12b_R, value);
	DAC_SoftwareTriggerCmd(DAC_Channel_2, ENABLE);
}
