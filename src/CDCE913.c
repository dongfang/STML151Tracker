#include "../inc/CDCE913.h"

#include "../inc/Bands.h"
#include "../inc/CDCE913_26MHzXtal.h"
#include "../inc/diag/Trace.h"
#include "../inc/Systick.h"
#include "../Libraries/CMSIS/Device/ST/STM32L1xx/Include/stm32l1xx.h"
#include "../Libraries/STM32L1xx_StdPeriph_Driver/inc/stm32l1xx_gpio.h"
#include "../Libraries/STM32L1xx_StdPeriph_Driver/inc/stm32l1xx_i2c.h"
#include "../Libraries/STM32L1xx_StdPeriph_Driver/inc/stm32l1xx_rcc.h"

#define CDCE913_I2C_ADDR 0b1100101
#define I2C_TIMEOUT 25

const int16_t PLL_XTAL_TRIM_PP10M[] = PLL_XTAL_TRIM_PP10M_VALUES;

// These are different options for the same frequency.
static const CDCE913_PLLSetting_t PLL_OPTIONS_WSPR_10m[] =
CDCE913_PLL_SETTINGS_10m_WSPR;

// These are different options for the same frequency.
static const CDCE913_PLLSetting_t PLL_OPTIONS_WSPR_30m[] =
CDCE913_PLL_SETTINGS_30m_WSPR
;

// These are different options for the same frequency (maybe one is enough).
// static const CDCE913_PLLSetting_t PLL_OPTIONS_APRS_30m[];

// These are different options but each for its OWN frequency.
// static const CDCE913_PLLSetting_t PLL_OPTIONS_APRS_DIRECT_2m[];
// This does not belong here at ALL but try to get the compiler convinced that they really are const
// if coming from a different source .. it's not possible.
/*
const HF_BandDef_t HF_BAND_DEFS[] = { { .hardwareChannel = 1, .numPLLOptions =
		sizeof(PLL_OPTIONS_WSPR_30m) / sizeof(CDCE913_PLLSetting_t),
		.PLLOptions = PLL_OPTIONS_WSPR_30m, .frequency = 10140200 }, {
		.hardwareChannel = 1, .numPLLOptions = sizeof(PLL_OPTIONS_WSPR_10m)
				/ sizeof(CDCE913_PLLSetting_t), .PLLOptions =
				PLL_OPTIONS_WSPR_10m, .frequency = 28126100 } };

const VHF_ChannelDef_PLL_t VHF_PLL_BAND_DEFS[] = { };
const uint8_t NUM_VHF_PLL_BAND_DEFS = sizeof(VHF_PLL_BAND_DEFS)
		/ sizeof(VHF_ChannelDef_PLL_t);

const VHF_ChannelDef_Si6643_t VHF_SI4463_BAND_DEFS[] = { 144390, 144620, 144640,
		144575, 144660, 144930, 144800, 145010, 145175, 145525, 145575 };
const uint8_t NUM_VHF_SI4463_BAND_DEFS = sizeof(VHF_SI4463_BAND_DEFS)
		/ sizeof(VHF_ChannelDef_Si6643_t);
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

static void CDCE913_initInterfaceIfNecessary() {
	if (!(I2C1->CR1 & I2C_CR1_PE)) {
		CDCE913_initInterface();
	}
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
	I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) && !timer_elapsed(I2C_TIMEOUT))
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
	I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)) && !timer_elapsed(I2C_TIMEOUT))
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
	I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) && !timer_elapsed(I2C_TIMEOUT))
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
static void CDCE913_enableOutput(CDCE913_OutputMode_t whichOutput,
		uint16_t pdiv) {
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
		r1 = 0b00000101; 	// power on, VCXO and default address
		r2 = 0b10111100 | (pdiv >> 8);
		r0x14 = 0b11101010;	// Disable Y2, Y3 to low
		I2C_write(0x03, (uint8_t) pdiv);
	}
	I2C_write(0x01, r1);
	I2C_write(0x02, r2);
	I2C_write(0x14, r0x14);
}

void PLL_shutdown() {
	CDCE913_initInterfaceIfNecessary();
	CDCE913_enableOutput(CDCE913_OutputMode_SHUTDOWN, 0);
}

// Always on output #1
void CDCE913_setDirectModeWithDivision(uint8_t trim, uint16_t pdiv) {
	CDCE913_initInterfaceIfNecessary();
	CDCE913_enableOutput(CDCE913_OutputMode_SELFCALIBRATION_DIVISION_AT_1, pdiv);
	I2C_write(5, trim << 3); 	// Cap. in pF.
}

// Always on output #1
void PLL_setXOPassthroughMode(uint8_t trim) {
	CDCE913_initInterfaceIfNecessary();
	CDCE913_enableOutput(CDCE913_OutputMode_XO_PASSTHROUGH, 1);
	I2C_write(5, trim << 3); 	// Cap. in pF.
}

void setPLL(CDCE913_OutputMode_t output,
		const CDCE913_PLLSetting_t* setting) {
	CDCE913_initInterfaceIfNecessary();
	// Turn on Y1 output in all cases and PLL out
	// I2C_write(1, 0b0101);
	// I2C_write(2, (1 << 7) | (0b1111 << 2));
	CDCE913_enableOutput(output, setting->pdiv);
	CDCE913_setPLLValue(setting);
	trace_printf("Using trim %d\n", setting->trim);
	I2C_write(5, setting->trim << 3); 	// Cap. in pF.
	// PLL_printSettings();
}

// 0-->0
// 1-->0	// 1 is wrong, should be 0!
// 2-->1
// 3-->1
// 4-->2
// etc
static uint8_t ilog2(uint16_t N) {
	uint8_t result = 0;
	N >>= 1; // 0->0, 1->0, which both will correctly return 0
	while (N) {
		N >>= 1;
		result++;
	}
	return result;
}

void setPQR(uint16_t N, uint16_t M, CDCE913_PLLSetting_t* result) {
	int8_t P = 4 - ilog2(N / M);
	if (P < 0)
		P = 0;
	result->P = P;
	uint32_t Nprime = N * (1 << P);
	result->Q = Nprime / M;
	result->R = Nprime - M * result->Q;
}

boolean findM(int16_t N, double desiredMultiplication, uint16_t* M,
		double* signedError) {
	// desiredMultiplication = Fvco/Fin = N/M
	// M = N/desiredMultiplication
	int16_t Mlo = N / desiredMultiplication;
	// yeah Mhi will be wrong in case of an exact division.
	// But in that case, the Mhi result will be discarded anyway as the Mlo
	// one is "infinitely better" so no reason to make a big deal of it.
	int16_t Mhi = Mlo + 1;

	if (Mlo > 0 && Mlo < 512) {
		// Err = (N/M) / desired
		// = N / (M * desired)
		double error = (double) N / (Mlo * desiredMultiplication)-1;
		*signedError = error;
		double unsignedError = (error < 0) ? -error : error;
		*M = Mlo;

		if (Mhi < 512) {
			//trace_printf("FindM 2\n");
		    error = (double) N / (Mhi * desiredMultiplication)-1;
			double localUnsignedError = (error < 0) ? -error : error;
			if (localUnsignedError < unsignedError) {
				*signedError = error;
				*M = Mhi;
			}
		}

		return true;
	}

	//trace_printf("FindM returns %d and error %d\n", hadResult, (int)(*signedError*1000));
	return false;
}

int8_t PLL_bestTrim(double desiredTrim) {
	int32_t desiredPP10M = desiredTrim * 1E7;
	// trace_printf("Trim desired change pp10m: %d\n", desiredPP10M);
	int16_t bestError = 1500;
	int8_t bestIndex = -1;

	for (uint8_t i = PLL_MIN_TRIM_INDEX_VALUE; i <= PLL_MAX_TRIM_INDEX_VALUE;
			i++) {
		int16_t test = desiredPP10M - PLL_XTAL_TRIM_PP10M[i];
		if (test < 0)
			test = -test;
		if (test < bestError) {
			bestError = test;
			bestIndex = i;
		}
	}
	return bestIndex;
}

boolean PLL_bestPLLSetting(
		uint32_t oscillatorFrequency,
		uint32_t desiredFrequency,
		double maxError,
		CDCE913_PLLSetting_t* result) {
	trace_printf("bestPLLSetting: %d %d\n", oscillatorFrequency,
			desiredFrequency);

	// okay the rounding is off if somebody wants 100MHz. But nobody wants that.
	int Pdivmin = 100E6 / (double) desiredFrequency + 1;
	int pdivMax = 200E6 / (double) desiredFrequency;
	double bestError = 1;
	boolean feasible = false;

	for (int Pdiv = Pdivmin; Pdiv <= pdivMax && bestError > maxError; Pdiv++) {
		double desiredMultiplication = ((double) desiredFrequency * Pdiv)
				/ oscillatorFrequency;
		if (desiredMultiplication < 1)
			continue;
		uint16_t Nmax = desiredMultiplication * 511;
		if (Nmax > 4095) {
			Nmax = 4095;
		}
		// trace_printf("Trying Pdiv : %d, N from 1 to %d\n", Pdiv, Nmax);
		uint16_t M;
		// Just cut it short as we get within decent trim range, which is about +-30ppm
		for (uint16_t N = 1; N <= Nmax && bestError > maxError; N++) {
			double signedError;
			double unsignedError;
			if (findM(N, desiredMultiplication, &M, &signedError)) {
				unsignedError = signedError < 0 ? -signedError : signedError;
				// trace_printf("N: %d, US %d S %d\n", N, (int)(unsignedError*1000), (int)(signedError*1000));
				if (unsignedError < bestError) {
					bestError = unsignedError;
					// trace_printf("Best N:%d,M:%d, error pp10M:%u\n", N, M, (int) (signedError * 1E7));
					result->pdiv = Pdiv;
					result->N = N;
					result->M = M;
					setPQR(N, M, result);

					int8_t trim = PLL_bestTrim(-signedError);
					// trace_printf("SignedError PPM: %d, trim:%d\n", (int)(signedError * 1E6), trim);

					if (trim != -1) {
						/*
						 trace_printf("desired:%d, real:%d, error:%d, trim: %d\n",
						 (int) (desiredMultiplication * 1E7),
						 (int) (1E7 * (double) N / (double) M),
						 (int) (signedError * 1E7),
						 trim);
						 */
						result->trim = trim;
						feasible = true;
					}
				}
			}
		}
	}
	return feasible;
}

int16_t PLL_oscillatorError(uint32_t measuredFrequency) {
	int32_t result = (int32_t)measuredFrequency - (int32_t)PLL_XTAL_NOMINAL_FREQUENCY;
	// trace_printf("Osc error: %d\n", result);
	if (result > 32767) result = 32767;
	else if (result < -32768) result = -32768;
	return result;
}

void PLL_printSettings() {
	uint16_t r5 = I2C_read(0x05);
	uint16_t r18 = I2C_read(0x18);
	uint16_t r19 = I2C_read(0x19);
	trace_printf("N=%u\n", (r18 << 4) + (r19 >> 4));

	uint16_t r1a = I2C_read(0x1a);
	trace_printf("R=%u\n", ((r19 & 0xf) << 5) + (r1a >> 3));

	uint16_t r1b = I2C_read(0x1b);
	trace_printf("Q=%u\n", ((r1a & 0x7) << 3) + (r1b >> 5));
	trace_printf("P=%u\n", (r1b & 0b1100) >> 2);

	trace_printf("Trim=%u\n", r5 >> 3);
}

