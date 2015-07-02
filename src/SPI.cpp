/*
 * Copyright (c) 2010 by Cristian Maglie <c.maglie@bug.st>
 * SPI Master library for arduino.
 *
 * This file is free software; you can redistribute it and/or modify
 * it under the terms of either the GNU General Public License version 2
 * or the GNU Lesser General Public License version 2.1, both as
 * published by the Free Software Foundation.
 */

//#include "pins_arduino.h"
#include "SPI.h"
#include "stm32l1xx_conf.h"

#include "Systick.h"
#include <diag/trace.h>

void SPI::SPI_begin() {
	//	  SPI_Cmd(SPI1, DISABLE);           /* Disable the SPI  */

	//Enable clock of the SPI module
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);

	/* Enable SCK, MOSI and MISO GPIO clocks */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

	GPIO_PinAFConfig(GPIOA, GPIO_PinSource5, GPIO_AF_SPI1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_SPI1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_SPI1);

	GPIO_InitTypeDef GPIO_InitStructure;

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;

	/* SPI SCK pin configuration */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	// Configure the SPI module
	SPI_InitTypeDef SPI_InitStructure;
	SPI_StructInit(&SPI_InitStructure);
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16;

	SPI_Init(SPI1, &SPI_InitStructure);
	SPI_Cmd(SPI1, ENABLE); /* Enable the SPI  */

	SPI_NSSInternalSoftwareConfig(SPI1, SPI_NSSInternalSoft_Set);
}

uint8_t SPI::SPI_transfer(uint8_t _data) {
	// SPI_assertSS();

	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET)
		;

	SPI_I2S_SendData(SPI1, _data);

	// wait until RXNE = 1
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) != SET)
		;

	// SPI_releaseSS();
	// timer_sleep(1);
	// SPI_assertSS();
	// uint8_t reply;

	/*
	while(() != 0xff) {
		SPI_releaseSS();
		timer_sleep(1);
		SPI_assertSS();
	}

	SPI_releaseSS();
	timer_sleep(1);
	return reply;
	*/
	uint8_t reply = SPI_I2S_ReceiveData(SPI1);
	trace_printf("Sent %d and got %d back\n", _data, reply);
	return reply;
}

void SPI::SPI_end() {
	SPI_Cmd(SPI1, DISABLE); /* Disable the SPI  */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, DISABLE);
}

void SPI::SPI_assertSS() {
	// SPI_NSSInternalSoftwareConfig(SPI1, SPI_NSSInternalSoft_Reset);
	timer_sleep(1);
	GPIOB->ODR &= ~(1 << 0);
	timer_sleep(1);
}

void SPI::SPI_releaseSS() {
	timer_sleep(1);
	GPIOB->ODR |= (1 << 0);
	timer_sleep(1);
	// SPI_NSSInternalSoftwareConfig(SPI1, SPI_NSSInternalSoft_Set);
}
