#include <diag/Trace.h>
#include <stm32l1xx.h>
#include <stm32l1xx_gpio.h>
#include <Types.h>

void I2C1_GPIO_Config(void) {
	// RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
	GPIO_InitTypeDef GPIO_InitStructure;
	// Configure I2C1 pins: SCL and SDA
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_OType_OD;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
}

void I2C_delay(void) {
	volatile int i = 5;
	while (i) {
		i--;
	}
}

#define SDAH (GPIOB->ODR |= GPIO_Pin_9)
#define SDAL (GPIOB->ODR &= ~GPIO_Pin_9)
#define SCLH (GPIOB->ODR |= GPIO_Pin_8)
#define SCLL (GPIOB->ODR &= ~GPIO_Pin_8)

#define SDAread (GPIOB->IDR & GPIO_Pin_9)
#define SCLread (GPIOB->IDR & GPIO_Pin_8)

boolean I2C1_start(void) {
	SDAH;
	SCLH;
	I2C_delay();
	if (!SDAread)
		return false;
	SDAL;
	I2C_delay();
	if (SDAread)
		return false;
	SDAL;
	I2C_delay();
	return true;
}

void I2C1_stop(void) {
	SCLL;
	I2C_delay();
	SDAL;
	I2C_delay();
	SCLH;
	I2C_delay();
	SDAH;
	I2C_delay();
}

void I2C1_ack(void) {
	SCLL;
	I2C_delay();
	SDAL;
	I2C_delay();
	SCLH;
	I2C_delay();
	SCLL;
	I2C_delay();
}

void I2C1_noAck(void) {
	SCLL;
	I2C_delay();
	SDAH;
	I2C_delay();
	SCLH;
	I2C_delay();
	SCLL;
	I2C_delay();
}

boolean I2C1_waitAck(void) {
	SCLL;
	I2C_delay();
	SDAH;
	I2C_delay();
	SCLH;
	I2C_delay();
	if (SDAread) {
		SCLL;
		return false;
	}
	SCLL;
	return true;
}

void I2C1_sendByte(uint8_t sendByte) {
	unsigned char i = 8;
	while (i--) {
		SCLL;
		I2C_delay();
		if (sendByte & 0x80)
			SDAH;
		else
			SDAL;
		sendByte <<= 1;
		I2C_delay();
		SCLH;
		I2C_delay();
	}
	SCLL;
}

uint8_t I2C1_receiveByte(void) {
	uint8_t i = 8;
	uint8_t receiveByte = 0;

	SDAH;
	while (i--) {
		receiveByte <<= 1;
		SCLL;
		I2C_delay();
		SCLH;
		I2C_delay();
		if (SDAread) {
			receiveByte |= 0x01;
		}
	}
	SCLL;
	return receiveByte;
}

uint8_t I2C1_readByte(uint8_t deviceAddress, uint8_t registerAddress) {
	uint8_t temp;
	if (!I2C1_start())
		return false;

	I2C1_sendByte((deviceAddress & 0xFE));
	if (!I2C1_waitAck()) {
		I2C1_stop();
		return false;
	}
	I2C1_sendByte(registerAddress);
	I2C1_waitAck();

	// Restart
	I2C1_start();
	// Now as read
	I2C1_sendByte((deviceAddress & 0xFE) | 0x01);
	I2C1_waitAck();

	temp = I2C1_receiveByte();

	I2C1_noAck();

	I2C1_stop();
	return temp;
}

boolean I2C1_writeByte(uint8_t DeviceAddress, uint8_t registerAddress, uint8_t data) {
	uint8_t temp;
	if (!I2C1_start()) {
		trace_printf("I2CStart fail\n");
		return false;
	}

	I2C1_sendByte(DeviceAddress & 0xFE);
	if (!I2C1_waitAck()) {
		trace_printf("I2CWriteDeviceAddress fail\n");
		I2C1_stop();
		return false;
	}
	I2C1_sendByte(registerAddress);
	if (!I2C1_waitAck()) {
			trace_printf("I2CWriteRegAddress fail\n");
			I2C1_stop();
			return false;
		}	// I2C1_start();
	// I2C1_sendByte(DeviceAddress & 0xFE);
	// I2C1_waitAck();

	I2C1_sendByte(data);
	if (!I2C1_waitAck()) {
			trace_printf("I2CWriteData fail\n");
			I2C1_stop();
			return false;
		}	I2C1_stop();
	return temp;
}
