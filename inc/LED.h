/*
 * LED.h
 *
 *  Created on: Sep 9, 2015
 *      Author: dongfang
 */

#ifndef INC_LED_H_
#define INC_LED_H_

#include <stm32l1xx_gpio.h>

typedef enum {
	GPS_ACQUISITION,
	GPS_NO_DATA_RECEIVED_ERROR,
	GPS_NO_CONFIRMATION_RECEIVED_ERROR,

	GPS_ACQUIRED,
	GPS_FAILED_TO_ACQUIRE,

	I2C_ERROR,
	SPI_ERROR,
	ADC_ERROR,
	RTC_ERROR
} LEDMessage_t;

#define LED_CODES {\
	'.', '--.', '..-', 	// dot-flash for #sats, G=GPS_NO_DATA_RECEIVED_ERROR, U=GPS_NO_CONFIRMATION_RECEIVED_ERROR\
	'.-', '-.', 		// A=GPS_ACQUIRED, N=GPS_FAILED_TO_ACQUIRE \
	'.--.', '.-.','...-','-.-.'	// P=PLL(I2C)ERROR, R=RADIO(SPI)ERROR, V=VOLTAGE(ADC)ERROR, C=CLOCK(RTC)ERROR \
}

#define LED_PORT GPIOB
#define LED_PORTBIT GPIO_Pin_0

#endif /* INC_LED_H_ */
