/* Includes ------------------------------------------------------------------*/
#include "stm32l1xx_conf.h"
#include <math.h>
#include "DataTypes.h"
#include "APRS.h"
#include "DAC.h"

extern volatile uint8_t packet[];
extern volatile uint16_t packet_size;

volatile uint16_t packet_cnt;
volatile uint8_t packetTransmissionComplete;
volatile APRSModulationMode_t currentMode;

/* Private define ------------------------------------------------------------*/
#define DAC_DHR12RD_Address      0x40007420

volatile uint16_t simpleGFSKLevels[2];

#define NUM_AFSK_SAMPLES 202
uint16_t singleSine12bit[NUM_AFSK_SAMPLES];
static const uint8_t AFSK_DIVIDERS[] = { 5, 10 };

/* Private functions ---------------------------------------------------------*/

static uint8_t symbol;

/**
 * @brief  Configures the TIM2 (tone output) and TIM3 (symbol output @ 1200Hz)
 */
void AFSK_TIM_Config(void) {
	/* TIM2 Periph clock enable */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	/* Time base configuration, 1200 /s for timer2 the sample-out timer */
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.TIM_Period = AFSK_DIVIDERS[1];
	// tone = 2;
	TIM_TimeBaseStructure.TIM_Prescaler = 6 - 1;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0x0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Down;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

	/* Timer3 is the baud rate generator and runs at 1200Hz */
	TIM_TimeBaseStructure.TIM_Period = 11 * NUM_AFSK_SAMPLES - 1;
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

void AFSK_TIM_deinit(void) {
	TIM_ITConfig(TIM3, TIM_IT_Update, DISABLE);
	TIM_Cmd(TIM2, DISABLE);

	/* TIM3 enable counter */
	TIM_Cmd(TIM3, DISABLE);

	/* TIM2 Periph clock disable */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, DISABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, DISABLE);
}

/**
 * @brief  Configures DAC channel 1 and channel 2
 * @param  None
 * @retval None
 */
void AFSK_DAC_Config(void) {
	/* Enable GPIOA Periph clock --------------------------------------*/
	// RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
	GPIO_InitTypeDef GPIO_InitStructure;
	/* Configure PA.04 (DAC_OUT1), PA.05 (DAC_OUT2) as analog */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
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

	/* DAC channel2 Configuration. Should not be necessary but it @#$# doesnt work with only DAC1 running. */
	DAC_Init(DAC_Channel_2, &DAC_InitStructure);

	/* Enable DAC Channel1: Once the DAC channel1 is enabled, PA.04 is
	 automatically connected to the DAC converter. */
	DAC_Cmd(DAC_Channel_1, ENABLE);

	/* Enable DAC Channel2: Once the DAC channel2 is enabled, PA.05 is
	 automatically connected to the DAC converter. Should not be necessary but it @#$# doesnt work with only DAC1 running. */
	DAC_Cmd(DAC_Channel_2, ENABLE);

	/* Enable DMA for DAC Channel1 */
	DAC_DMACmd(DAC_Channel_1, ENABLE);

	/* Enable DMA for DAC Channel2. Should not be necessary but it @#$# doesnt work with only DAC1 running.*/
	DAC_DMACmd(DAC_Channel_2, ENABLE);
}

void AFSK_DAC_deinit(void) {
	/* Disable DMA for DAC Channel2 */
	DAC_DMACmd(DAC_Channel_1, DISABLE);
	DAC_DMACmd(DAC_Channel_2, DISABLE);
	/* Disable DAC Channel2 */
	DAC_Cmd(DAC_Channel_2, DISABLE);
	/* Disable DAC Channel1 */
	DAC_Cmd(DAC_Channel_1, DISABLE);

	/* DAC Periph clock enable */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, DISABLE);

	/* Dubious - disable GPIOA Periph clock --------------------------------------*/
	//RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
}

/**
 * @brief  Configures DMA1 channel3
 * @param  None
 * @retval None
 */
void AFSK_DMA_Config(void) {
	DMA_InitTypeDef DMA_InitStructure;
	/* Enable DMA1 clock -------------------------------------------------------*/
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

	DMA_DeInit(DMA1_Channel3);
	DMA_InitStructure.DMA_PeripheralBaseAddr = DAC_DHR12RD_Address;
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t) & singleSine12bit;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
	DMA_InitStructure.DMA_BufferSize = NUM_AFSK_SAMPLES;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA1_Channel3, &DMA_InitStructure);

	/* Enable DMA1 Channel3 */
	DMA_Cmd(DMA1_Channel3, ENABLE);
}

void AFSK_DMA_deinit(void) {
	/* Enable DMA1 Channel3 */
	DMA_Cmd(DMA1_Channel3, DISABLE);

	/* Enable DMA1 clock -------------------------------------------------------*/
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, DISABLE);
}

/**
 * @brief  Configures TIM3 (symbol output @ 300Hz)
 */
void GFSK_TIM_Config(void) {
	/* TIM2 Periph clock enable */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);

	// 16E6 / 7619 / 7 = 300
	TIM_TimeBaseStructure.TIM_Prescaler = 7619 - 1;
	TIM_TimeBaseStructure.TIM_Period = 7 - 1;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0x0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

	// Set up timer interrupt.
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);

	/* TIM3 enable counter */
	TIM_Cmd(TIM3, ENABLE);
}

// TODO: Can share with WSPR if taking a parameter which DAC to set up.
void GFSK_DAC_Config() {
	// Setup DAC-out port and DAC's clock
	GPIO_InitTypeDef GPIO_InitStructure;
	/* Configure PA.04 (DAC_OUT1), PA.05 (DAC_OUT2) as analog */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
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
	DAC_Init(DAC_Channel_1, &DAC_InitStructure);

	DAC_Cmd(DAC_Channel_1, ENABLE);
}

void setSymbol(uint8_t _symbol) {
	symbol = _symbol;
	if (currentMode == AFSK) {
		TIM2->ARR = AFSK_DIVIDERS[symbol];
	} else {
		setDAC(DAC1, simpleGFSKLevels[symbol]);
	}
}

void FSK_Modulation() {
	uint8_t mask = 1 << (packet_cnt & 7);
	uint16_t index = packet_cnt >> 3;
	uint8_t bit = packet[index] & mask;
	if (!bit) {
		setSymbol(!symbol);
	}
}

void TIM3_IRQHandler(void) {
	if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET) {
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update);

		FSK_Modulation();

		if (!packetTransmissionComplete) {
			if (packet_cnt < packet_size) {
				packet_cnt++;
			} else {
				packetTransmissionComplete = true;
			}
		}
	}
}

void AFSK_init(float modulationAmplitude) {
	currentMode = AFSK;

	/* Fill Sine12bit table */
	for (uint16_t Idx = 0; Idx < NUM_AFSK_SAMPLES; Idx++) {
		double p = (double) Idx / NUM_AFSK_SAMPLES * 3.14159265358979323846 * 2;
		double a = sin(p) * modulationAmplitude;
		uint16_t v = (uint16_t)(a + 2048);
		//v = ((double)Idx / NUM_AFSK_SAMPLES) * 200 + 1024;
		singleSine12bit[Idx] = v; //+ (v << 16);
	}

	/* Avoid firing a transmission prematurely */
	packetTransmissionComplete = true;

	/* DMA1 channel3 configuration: DualSine12bit is used as memory base address */
	AFSK_DMA_Config();

	/* DAC configuration ------------------------------------------------------*/
	AFSK_DAC_Config();

	/* TIM2 configuration ------------------------------------------------------*/
	AFSK_TIM_Config();

	setSymbol(0);
}

void AFSK_shutdown() {
	/* TIM2 configuration ------------------------------------------------------*/
	AFSK_TIM_deinit();

	/* DMA1 channel3 configuration: DualSine12bit is used as memory base address */
	AFSK_DMA_deinit();

	/* DAC configuration ------------------------------------------------------*/
	AFSK_DAC_deinit();
}

/* We can implement GFSK as:
 * There are 2 curves in memory, ramp-up and ramp-down, made as polynomials.
 * Just play the right one.
 * Or, simplified: Just set DAC directly 300 times a second, using an interrupt.
 */
void GFSK_init(float modulationAmplitude) {
	currentMode = GFSK;

	simpleGFSKLevels[0] = 2048 - modulationAmplitude / 2;
	simpleGFSKLevels[1] = 2048 + modulationAmplitude / 2;

	// Similar to the WSPR DAC setup.
	GFSK_DAC_Config();
	GFSK_TIM_Config();

	setSymbol(0);
}

void GFSK_shutdown() {

}

// For WSPR (no DMA)
void WSPR_DAC_Init() {
	// Setup DAC-out port and DAC's clock
	GPIO_InitTypeDef GPIO_InitStructure;
	/* Configure PA.04 (DAC_OUT1), PA.05 (DAC_OUT2) as analog */
	GPIO_InitStructure.GPIO_Pin = /* GPIO_Pin_4 | */GPIO_Pin_5;
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

// SelfCalibration uses it. WSPR too.
void setDAC(DACChannel_t channel, uint16_t value) {
	switch (channel) {
	case DAC1:
		DAC_SetChannel1Data(DAC_Align_12b_R, value);
		DAC_SoftwareTriggerCmd(DAC_Channel_1, ENABLE);
		break;
		break;
	case DAC2:
		DAC_SetChannel2Data(DAC_Align_12b_R, value);
		DAC_SoftwareTriggerCmd(DAC_Channel_2, ENABLE);
		break;
	}
}
