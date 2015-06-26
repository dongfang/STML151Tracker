#include "stm32l1xx_conf.h"
#include "CDCE913.h"
#include "Types.h"
#include <diag/trace.h>
#include "systick.h"

#define CDCE913_I2C_ADDR 0b1100101
#define I2C_TIMEOUT 25
/*
 const TransmitterSetting_t* bestPLLSetting(const WSPR_BandSetting_t* bandSettings, double desiredMultiplication) {
 double minError = 1000;
 uint8_t i;
 uint8_t besti = bandSettings->numPLLOptions/2;
 for(i=0; i<bandSettings->numPLLOptions; i++) {
 double error = bandSettings->PLLOptions[i].mul - desiredMultiplication;
 if (error < 0) error = -error;
 if (error < minError) {
 besti = i;
 minError = error;
 }
 }
 return &bandSettings->PLLOptions[besti];
 }
 */

static void CDCE913_initInterface() {
	GPIO_InitTypeDef GPIO_InitStructure;
	I2C_InitTypeDef I2C_InitStructure;

	/* I2C1 clock enable */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);

	/* I2C1 SDA and SCL configuration */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	// What the fuck is this for?
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_I2C1);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_I2C1);

	I2C_DeInit(I2C1);

	/*enable I2C*/
	I2C_Cmd(I2C1, ENABLE);

	/* I2C1 configuration */
	I2C_StructInit(&I2C_InitStructure);
	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
	I2C_InitStructure.I2C_OwnAddress1 = 0x00;
	I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_InitStructure.I2C_ClockSpeed = IS_I2C_CLOCK_SPEED(400000);
	I2C_Init(I2C1, &I2C_InitStructure);
	//trace_printf("I2C initialized.\n");
}

static uint8_t I2C_read(uint8_t reg_addr) {
	int status;
	timer_mark();
	/* initiate start sequence */
	I2C_GenerateSTART(I2C1, ENABLE);
	while ((status = !I2C_GetFlagStatus(I2C1, I2C_FLAG_SB))
			&& !timer_elapsed(I2C_TIMEOUT))
		;
	if (status) {
		trace_printf("I2C:Timeout sending START\n");
	}

	/*send read command to chip*/
	timer_mark();
	I2C_Send7bitAddress(I2C1, CDCE913_I2C_ADDR << 1, I2C_Direction_Transmitter);
	/*check master is now in Tx mode*/
	while ((status = !I2C_CheckEvent(I2C1,
			I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
			&& !timer_elapsed(I2C_TIMEOUT))
		;
	if (status) {
		trace_printf("I2C:Timeout sending address (1)\n");
	}

	/*who am i register address*/
	timer_mark();
	I2C_SendData(I2C1, reg_addr | (1 << 7)); // The 1<<7 is to do a byte read, not a block read.
	/*wait for byte send to complete*/
	while ((status = !I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
			&& !timer_elapsed(I2C_TIMEOUT))
		;
	if (status) {
		trace_printf("I2C:Timeout sending RA\n");
	}

	// Repeated start
	timer_mark();
	I2C_GenerateSTART(I2C1, ENABLE);
	while ((status = !I2C_GetFlagStatus(I2C1, I2C_FLAG_SB))
			&& !timer_elapsed(I2C_TIMEOUT))
		;
	if (status) {
		trace_printf("I2C:Timeout sending repeated start\n");
	}
	/*send read command to chip*/
	timer_mark();
	I2C_Send7bitAddress(I2C1, CDCE913_I2C_ADDR << 1, I2C_Direction_Receiver);
	/*check master is now in Rx mode*/
	while ((status = !I2C_CheckEvent(I2C1,
			I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED))
			&& !timer_elapsed(I2C_TIMEOUT))
		;
	if (status) {
		trace_printf("I2C:Timeout sending address (2)\n");
	}

	/*enable ACK bit */
	timer_mark();
	I2C_AcknowledgeConfig(I2C1, DISABLE);
	while ((status = !I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED))
			&& !timer_elapsed(I2C_TIMEOUT))
		;
	uint8_t data = I2C_ReceiveData(I2C1);
	if (status) {
		trace_printf("I2C:Timeout sending data\n");
	}
	/*generate stop*/
	timer_mark();
	I2C_GenerateSTOP(I2C1, ENABLE);
	/*stop bit flag*/
	while ((status = I2C_GetFlagStatus(I2C1, I2C_FLAG_STOPF))
			&& !timer_elapsed(I2C_TIMEOUT))
		;
	if (status) {
		trace_printf("I2C:Timeout sending STOP\n");
	}

	return data;
}

static void I2C_write(uint8_t reg_addr, uint8_t data) {
	int status;
	timer_mark();
	/* initiate start sequence */
	I2C_GenerateSTART(I2C1, ENABLE);
	while ((status = !I2C_GetFlagStatus(I2C1, I2C_FLAG_SB))
			&& !timer_elapsed(I2C_TIMEOUT))
		;
	if (status) {
		trace_printf("I2C:Timeout sending START\n");
	}

	timer_mark();
	/*send read command to chip*/
	I2C_Send7bitAddress(I2C1, CDCE913_I2C_ADDR << 1, I2C_Direction_Transmitter);
	/*check master is now in Tx mode*/
	while ((status = !I2C_CheckEvent(I2C1,
			I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
			&& !timer_elapsed(I2C_TIMEOUT))
		;
	if (status) {
		trace_printf("I2C:Timeout sending address\n");
	}

	/*who register address*/
	timer_mark();
	I2C_SendData(I2C1, reg_addr | (1 << 7));
	/*wait for byte send to complete*/
	while ((status = !I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
			&& !timer_elapsed(I2C_TIMEOUT))
		;
	if (status) {
		trace_printf("I2C:Timeout sending RA\n");
	}

	/*data byte*/
	timer_mark();
	I2C_SendData(I2C1, data);
	/*wait for byte send to complete*/
	while ((status = !I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
			&& !timer_elapsed(I2C_TIMEOUT))
		;
	if (status) {
		trace_printf("I2C:Timeout sending data\n");
	}

	/*generate stop*/
	timer_mark();
	I2C_GenerateSTOP(I2C1, ENABLE);
	/*stop bit flag*/
	while ((status = I2C_GetFlagStatus(I2C1, I2C_FLAG_STOPF))
			&& !timer_elapsed(I2C_TIMEOUT))
		;
	if (status) {
		trace_printf("I2C:Timeout sending STOP\n");
	}
}

static void CDCE913_setPLLValue(const CDCE913_PLLSetting_t* setting) {
	uint8_t pllRange = 0;

	// uint8_t r2 = I2C_read(0x02);
	// I2C_write(0x02, (r2 & 0b11111100) | (setting->pdiv >> 8));
	// I2C_write(0x03, (uint8_t) setting->pdiv);

	I2C_write(0x18, (uint8_t)(setting->N >> 4));
	I2C_write(0x19, (setting->N << 4) | (setting->R >> 5));
	I2C_write(0x1a, (setting->R << 3) | (setting->Q >> 3)); // Q upper bits
	I2C_write(0x1b, (setting->Q << 5) | (setting->P << 2) | pllRange);

	I2C_write(0x1c, (uint8_t)(setting->N >> 4));
	I2C_write(0x1d, (setting->N << 4) | (setting->R >> 5));
	I2C_write(0x1e, (setting->R << 3) | (setting->Q >> 3)); // Q upper bits
	I2C_write(0x1f, (setting->Q << 5) | (setting->P << 2) | pllRange);
}

// whichOutput:
// 0 : Shut down
// 1 : Use Y1 and ground the other two.
// 2 : Use Y2 and ground the other two.
// 3 : Use Y3 and ground the other two.
// 4 : Feed xtal to Y1 with division.
// The direct mode is not supported from here.
void CDCE913_enableOutput(CDCE913_OutputMode_t whichOutput, uint16_t pdiv) {
	uint8_t r1 = 0;
	uint8_t r2 = 0;
	uint8_t r0x14 = 0;
	switch (whichOutput) {

	case CDCE913_OutputMode_SHUTDOWN:
		r1 = 0b00010001;	// power down
		r2 = 0;
		r0x14 = 0;
		break;
	case CDCE913_OutputMode_OUTPUT_1:
		r1 = 0b00000101; 	// power on, VCXO and default address
		r2 = 0b10111100 | (pdiv >> 8);
		r0x14 = 0b01101010;	// Disable Y2, Y3 to low
		I2C_write(0x03, pdiv);
		break;
	case CDCE913_OutputMode_OUTPUT_2:
		r1 = 0b00000101; 	// power on, VCXO and default address
		r2 = 0b10101000;	// Disable Y1
		r0x14 = 0b01101111;	// Enable Y2, Y3
		I2C_write(0x16, (1 << 7) | pdiv);
		I2C_write(0x17, 0);	// Keep Y3 reset.
		break;
	case CDCE913_OutputMode_OUTPUT_3:
		r1 = 0b00000101; 	// power on, VCXO and default address
		r2 = 0b10101000;	// Disable Y1
		r0x14 = 0b01101111;	// Enable Y2, Y3
		I2C_write(0x16, 1 << 7);	// Keep Y2 reset.
		I2C_write(0x17, pdiv);
		break;
	case CDCE913_OutputMode_SELFCALIBRATION_DIVISION_AT_1:
		r1 = 0b00000101; 	// power on, VCXO and default address
		r2 = 0b00111100 | (pdiv >> 8);
		r0x14 = 0b11101010;	// Disable Y2, Y3 to low
		I2C_write(0x03, (uint8_t) pdiv);
		break;
	case CDCE913_OutputMode_XO_PASSTHROUGH:
		// TODO - impl
		break;
	}
	I2C_write(0x01, r1);
	I2C_write(0x02, r2);
	I2C_write(0x14, r0x14);
}

void CDCE913_shutdown() {
	CDCE913_enableOutput(0, 0);
}

// Well actually direct mode is the startup default.... haha
void CDCE913_setDirectModeWithDivision(uint8_t trim) {
	CDCE913_initInterface();
	uint16_t pdiv = CDCE913_SELFCALIBRATION_DIVISION;
	CDCE913_enableOutput(4, pdiv);
	// Enable VCXO
	// I2C_write(1, 0b0101);
	// Well! We still need set a divider.
	// uint8_t r2 = I2C_read(0x02);
	// I2C_write(0x02, (r2 & 0b01111100) | (pdiv >> 8));
	// I2C_write(0x03, (uint8_t) pdiv);
	I2C_write(5, trim << 3); 	// Cap. in pF.
	// I2C_write(0x14, 0b00001010); // Disable Y2, Y3
	// TODO: Also set feedthru mode (or trigger a reset if possible).
}

// Set up for WSPR with a 8.388608MHz xtal
void CDCE913_setPLL(CDCE913_OutputMode_t output,
		const CDCE913_PLLSetting_t* setting, uint8_t trim) {
	CDCE913_initInterface();
	// Turn on Y1 output in all cases and PLL out
	// I2C_write(1, 0b0101);
	// I2C_write(2, (1 << 7) | (0b1111 << 2));
	CDCE913_enableOutput(output, setting->pdiv);
	I2C_write(5, trim << 3); 	// Cap. in pF.

	// Turn on PLL1
	// I2C_write(0x14, 0b00000101);

	// Center mode SSC
	// I2C_write(0x16, 0b10000000);

	CDCE913_setPLLValue(setting);
}

void CECEL913_debug() {
	uint16_t r18 = I2C_read(0x18);
	uint16_t r19 = I2C_read(0x19);
	trace_printf("N=%u\n", (r18 << 4) + (r19 >> 4));

	uint16_t r1a = I2C_read(0x1a);
	trace_printf("R=%u\n", ((r19 & 0xf) << 5) + (r1a >> 3));

	uint16_t r1b = I2C_read(0x1b);
	trace_printf("Q=%u\n", ((r1a & 0x7) << 3) + (r1b >> 5));
	trace_printf("P=%u\n", (r1b & 0b1100) >> 2);
}

