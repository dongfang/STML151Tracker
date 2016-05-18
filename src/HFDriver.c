#include "stm32l1xx.h"
#include "HFDriver.h"
#include "Globals.h"

HF_POWER_LEVEL HF_power() {
	HF_POWER_LEVEL power;
	if (batteryVoltage >= 4.2)
		power = FOUR_DRIVERS;
	else if (batteryVoltage >= 3.9)
		power = TWO_DRIVERS;
	else
		power = ONE_DRIVER;
	return power;
}

void HF_enableDriver(HF_POWER_LEVEL power) {
	switch (power) {
	case FOUR_DRIVERS:
		GPIOB->ODR |= GPIO_Pin_6;
		// no break here!
	case TWO_DRIVERS:
		GPIOB->ODR |= GPIO_Pin_4;
		// no break here!
	case ONE_DRIVER:
		GPIOB->ODR |= GPIO_Pin_5;
		// no break here!
	}
}

void HF_shutdownDriver() {
	GPIOB->ODR &= ~(GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6);
}
