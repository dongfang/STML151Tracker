// RH_RF24.cpp
//
// Copyright (C) 2011 Mike McCauley
// $Id: RH_RF24.cpp,v 1.10 2014/09/17 22:41:47 mikem Exp $

#include "RH_RF24.h"
#include "SPI.h"
#include "Systick.h"
#include "stm32l1xx.h"
#include <diag/trace.h>
#include <string.h>

// Generated with Silicon Labs WDS software:
#include "radio_config_Si4460.h"

// Interrupt vectors for the 3 Arduino interrupt pins
// Each interrupt can be handled by a different instance of RH_RF24, allowing you to have
// 2 or more RF24s per Arduino
// RH_RF24* RH_RF24::_deviceForInterrupt[RH_RF24_NUM_INTERRUPTS] = { 0, 0, 0 };
// uint8_t RH_RF24::_interruptCount = 0; // Index into _deviceForInterrupt for next device

// This configuration data is defined in radio_config_Si4460.h 
// which was generated with the Silicon Labs WDS program
const uint8_t RFM26_CONFIGURATION_DATA[] =
		RADIO_CONFIGURATION_DATA_ARRAY;

// These configurations were all generated originally by the Silicon LAbs WDS configuration tool.
// The configurations were imported into RH_RF24, the complete properties set dumped to a file with printRegisters, then 
// RH_RF24_property_data/convert.pl was used to generate the entry for this table.
// Contributions of new complete and tested ModemConfigs ready to add to this list will be readily accepted.
// Casual suggestions of new schemes without working examples will probably be passed over
static const RH_RF24::ModemConfig MODEM_CONFIG_TABLE[] =
		{
				{ 0b11101000, 0x01, 0x86, 0xa0, 0x01, 0x00, 0x00, 0x00, 0x00,
						0x32, 0x20, 0x00, 0x5e, 0x05, 0x76, 0x1a, 0x02, 0xb9,
						0x02, 0x12, 0x00, 0x57, 0x02, 0xb0, 0x62, 0x11, 0x15,
						0x15, 0x00, 0x02, 0xff, 0xff, 0x00, 0x28, 0x00, 0x00,
						0x07, 0xff, 0x40, 0xa2, 0x81, 0x26, 0xaf, 0x3f, 0xee,
						0xc8, 0xc7, 0xdb, 0xf2, 0x02, 0x08, 0x07, 0x03, 0x15,
						0xfc, 0x0f, 0x00, 0xa2, 0x81, 0x26, 0xaf, 0x3f, 0xee,
						0xc8, 0xc7, 0xdb, 0xf2, 0x02, 0x08, 0x07, 0x03, 0x15,
						0xfc, 0x0f, 0x00, 0x3f, 0x2c, 0x0e, 0x04, 0x0c, 0x73, }
		};

RH_RF24::RH_RF24() {
	_mode = RHModeInitialising;
	_idleMode = RH_RF24_DEVICE_STATE_READY;
}

void RH_RF24::setIdleMode(uint8_t idleMode) {
	_idleMode = idleMode;
}

boolean RH_RF24::initWarm() {
	// cmd_clear_all_interrupts();
	// Here we use a configuration generated by the Silicon Las Wireless Development Suite
	// in radio_config_Si4460.h
	// WE override a few things later that we need to be sure of.

	configure(RFM26_CONFIGURATION_DATA);

	// Get the device type and check it
	// This also tests whether we are really connected to a device
	uint8_t buf[8];
	if (!command(RH_RF24_CMD_PART_INFO, 0, 0, buf, sizeof(buf)))
		return false; // SPI error? Not connected?
	_deviceType = (buf[1] << 8) | buf[2];
	trace_printf("deviceType: %x\n", _deviceType);
	// Check PART to be either 0x4460, 0x4461, 0x4463, 0x4464
	if (_deviceType != 0x4460 && _deviceType != 0x4461 && _deviceType != 0x4463
			&& _deviceType != 0x4464)
		return false; // Unknown radio type, or not connected

	setModemConfig(EXPERIMENTAL_CW);

	return true;
}

void RH_RF24::clearBuffer() {
	_bufLen = 0;
	_txBufSentIndex = 0;
	_rxBufValid = false;
}

// Sets registers from a canned modem configuration structure
void RH_RF24::setModemRegisters(const ModemConfig* config) {
	// This list also generated with convert.pl
	set_properties(0x2000, &config->prop_2000, 1);
	set_properties(0x2003, &config->prop_2003, 1);
	set_properties(0x2004, &config->prop_2004, 1);
	set_properties(0x2005, &config->prop_2005, 1);
	set_properties(0x2006, &config->prop_2006, 1);
	set_properties(0x200b, &config->prop_200b, 1);
	set_properties(0x200c, &config->prop_200c, 1);
	set_properties(0x2018, &config->prop_2018, 1);
	set_properties(0x201e, &config->prop_201e, 1);
	set_properties(0x201f, &config->prop_201f, 1);
	set_properties(0x2022, &config->prop_2022, 1);
	set_properties(0x2023, &config->prop_2023, 1);
	set_properties(0x2024, &config->prop_2024, 1);
	set_properties(0x2025, &config->prop_2025, 1);
	set_properties(0x2026, &config->prop_2026, 1);
	set_properties(0x2027, &config->prop_2027, 1);
	set_properties(0x2028, &config->prop_2028, 1);
	set_properties(0x2029, &config->prop_2029, 1);
	set_properties(0x202d, &config->prop_202d, 1);
	set_properties(0x202e, &config->prop_202e, 1);
	set_properties(0x202f, &config->prop_202f, 1);
	set_properties(0x2030, &config->prop_2030, 1);
	set_properties(0x2031, &config->prop_2031, 1);
	set_properties(0x2035, &config->prop_2035, 1);
	set_properties(0x2038, &config->prop_2038, 1);
	set_properties(0x2039, &config->prop_2039, 1);
	set_properties(0x203a, &config->prop_203a, 1);
	set_properties(0x203b, &config->prop_203b, 1);
	set_properties(0x203c, &config->prop_203c, 1);
	set_properties(0x203d, &config->prop_203d, 1);
	set_properties(0x203e, &config->prop_203e, 1);
	set_properties(0x203f, &config->prop_203f, 1);
	set_properties(0x2040, &config->prop_2040, 1);
	set_properties(0x2043, &config->prop_2043, 1);
	set_properties(0x2045, &config->prop_2045, 1);
	set_properties(0x2046, &config->prop_2046, 1);
	set_properties(0x2047, &config->prop_2047, 1);
	set_properties(0x204e, &config->prop_204e, 1);
	set_properties(0x2100, &config->prop_2100, 1);
	set_properties(0x2101, &config->prop_2101, 1);
	set_properties(0x2102, &config->prop_2102, 1);
	set_properties(0x2103, &config->prop_2103, 1);
	set_properties(0x2104, &config->prop_2104, 1);
	set_properties(0x2105, &config->prop_2105, 1);
	set_properties(0x2106, &config->prop_2106, 1);
	set_properties(0x2107, &config->prop_2107, 1);
	set_properties(0x2108, &config->prop_2108, 1);
	set_properties(0x2109, &config->prop_2109, 1);
	set_properties(0x210a, &config->prop_210a, 1);
	set_properties(0x210b, &config->prop_210b, 1);
	set_properties(0x210c, &config->prop_210c, 1);
	set_properties(0x210d, &config->prop_210d, 1);
	set_properties(0x210e, &config->prop_210e, 1);
	set_properties(0x210f, &config->prop_210f, 1);
	set_properties(0x2110, &config->prop_2110, 1);
	set_properties(0x2111, &config->prop_2111, 1);
	set_properties(0x2112, &config->prop_2112, 1);
	set_properties(0x2113, &config->prop_2113, 1);
	set_properties(0x2114, &config->prop_2114, 1);
	set_properties(0x2115, &config->prop_2115, 1);
	set_properties(0x2116, &config->prop_2116, 1);
	set_properties(0x2117, &config->prop_2117, 1);
	set_properties(0x2118, &config->prop_2118, 1);
	set_properties(0x2119, &config->prop_2119, 1);
	set_properties(0x211a, &config->prop_211a, 1);
	set_properties(0x211b, &config->prop_211b, 1);
	set_properties(0x211c, &config->prop_211c, 1);
	set_properties(0x211d, &config->prop_211d, 1);
	set_properties(0x211e, &config->prop_211e, 1);
	set_properties(0x211f, &config->prop_211f, 1);
	set_properties(0x2120, &config->prop_2120, 1);
	set_properties(0x2121, &config->prop_2121, 1);
	set_properties(0x2122, &config->prop_2122, 1);
	set_properties(0x2123, &config->prop_2123, 1);
	set_properties(0x2203, &config->prop_2203, 1);
	set_properties(0x2300, &config->prop_2300, 1);
	set_properties(0x2301, &config->prop_2301, 1);
	set_properties(0x2303, &config->prop_2303, 1);
	set_properties(0x2304, &config->prop_2304, 1);
	set_properties(0x2305, &config->prop_2305, 1);
}

// Set one of the canned Modem configs
// Returns true if its a valid choice
bool RH_RF24::setModemConfig(ModemConfigChoice index) {
	if (index > (signed int) (sizeof(MODEM_CONFIG_TABLE) / sizeof(ModemConfig)))
		return false;

	ModemConfig cfg;
	memcpy(&cfg, &MODEM_CONFIG_TABLE[index], sizeof(RH_RF24::ModemConfig));
	setModemRegisters(&cfg);

	return true;
}

bool RH_RF24::setFrequency(uint32_t freq_kHz) {
	// See Si446x Data Sheet section 5.3.1
	// Also the Si446x PLL Synthesizer / VCO_CNT Calculator Rev 0.4
	uint8_t outdiv;
	uint8_t band;
	if (_deviceType == 0x4460 || _deviceType == 0x4461
			|| _deviceType == 0x4463) {
		// Non-continuous frequency bands
		if (freq_kHz <= 1050000 && freq_kHz >= 850000)
			outdiv = 4, band = 0;
		else if (freq_kHz <= 525000 && freq_kHz >= 425000)
			outdiv = 8, band = 2;
		else if (freq_kHz <= 350000 && freq_kHz >= 284000)
			outdiv = 12, band = 3;
		else if (freq_kHz <= 175000 && freq_kHz >= 142000)
			outdiv = 24, band = 5;
		else
			return false;
	} else {
		// 0x4464
		// Continuous frequency bands
		if (freq_kHz <= 960000 && freq_kHz >= 675000)
			outdiv = 4, band = 1;
		else if (freq_kHz < 675000 && freq_kHz >= 450000)
			outdiv = 6, band = 2;
		else if (freq_kHz < 450000 && freq_kHz >= 338000)
			outdiv = 8, band = 3;
		else if (freq_kHz < 338000 && freq_kHz >= 225000)
			outdiv = 12, band = 4;
		else if (freq_kHz < 225000 && freq_kHz >= 160000)
			outdiv = 16, band = 4;
		else if (freq_kHz < 169000 && freq_kHz >= 119000)
			outdiv = 24, band = 5;
		else
			return false;
	}

	// Set the MODEM_CLKGEN_BAND (not documented)
	uint8_t modem_clkgen[] = { (uint8_t)( band + 8 )};
	if (!set_properties(RH_RF24_PROPERTY_MODEM_CLKGEN_BAND, modem_clkgen,
			sizeof(modem_clkgen)))
		return false;

	freq_kHz *= 1000; // Convert to Hz

	// Now generate the RF frequency properties
	// Need the Xtal/XO freq from the radio_config file:
	uint32_t xtal_frequency[1] = RADIO_CONFIGURATION_DATA_RADIO_XO_FREQ;
	unsigned long f_pfd = 2 * xtal_frequency[0] / outdiv;

	unsigned int n = ((unsigned int) (freq_kHz / f_pfd)) - 1;
	float ratio = freq_kHz / (float) f_pfd;

	float rest = ratio - (float) n;
	unsigned long m = (unsigned long) (rest * 524288UL);
	unsigned int m2 = m / 0x10000;
	unsigned int m1 = (m - m2 * 0x10000) / 0x100;
	unsigned int m0 = (m - m2 * 0x10000 - m1 * 0x100);

	// PROP_FREQ_CONTROL_GROUP
	uint8_t freq_control[] = { (uint8_t)n, (uint8_t)m2, (uint8_t)m1, (uint8_t)m0 };
	return set_properties(RH_RF24_PROPERTY_FREQ_CONTROL_INTE, freq_control,
			sizeof(freq_control));
}

void RH_RF24::setModeIdle() {
	if (_mode != RHModeIdle) {
		// Set the antenna switch pins using the GPIO, assuming we have an RFM module with antenna switch
		uint8_t config[] = { 0, 0, RH_RF24_GPIO_HIGH, RH_RF24_GPIO_HIGH };
		command(RH_RF24_CMD_GPIO_PIN_CFG, config, sizeof(config));

		uint8_t state[] = { _idleMode };
		command(RH_RF24_CMD_REQUEST_DEVICE_STATE, state, sizeof(state));
		_mode = RHModeIdle;
	}
}

bool RH_RF24::sleep() {
	if (_mode != RHModeSleep) {
		uint8_t state[] = { RH_RF24_DEVICE_STATE_SLEEP };
		command(RH_RF24_CMD_REQUEST_DEVICE_STATE, state, sizeof(state));

		_mode = RHModeSleep;
	}
	return true;
}

void RH_RF24::setModeTx() {
	if (_mode != RHModeTx) {
		// Set the antenna switch pins using the GPIO, assuming we have an RFM module with antenna switch
		uint8_t config[] = { 0, 0, RH_RF24_GPIO_LOW, RH_RF24_GPIO_HIGH }; // The ebay board has the PG2179TB switch and rx on Ch2 and tx on Ch1.
																		  // GPIO2 is to VCont1 and GPIO3 is VCont2 so for Tx we need (really active low??)
																		  // GPIO3 low and GPIO4 high.
		command(RH_RF24_CMD_GPIO_PIN_CFG, config, sizeof(config));

		uint8_t tx_params[] = { 0x00, (uint8_t)( (_idleMode << 4)
				| RH_RF24_CONDITION_RETRANSMIT_NO
				| RH_RF24_CONDITION_START_IMMEDIATE )};
		command(RH_RF24_CMD_START_TX, tx_params, sizeof(tx_params));
		_mode = RHModeTx;
	}
}

void RH_RF24::setTxPower(uint8_t power) {
	uint8_t pa_bias_clkduty = 0;
	// These calculations valid for advertised power from Si chips at Vcc = 3.3V
	// you may get lower power from RFM modules, depending on Vcc voltage, antenna etc
	if (_deviceType == 0x4460) {
		// 0x4f = 13dBm
		pa_bias_clkduty = 0xc0;
		if (power > 0x4f)
			power = 0x4f;
	} else if (_deviceType == 0x4461) {
		// 0x7f = 16dBm
		pa_bias_clkduty = 0xc0;
		if (power > 0x7f)
			power = 0x7f;
	} else if (_deviceType == 0x4463 || _deviceType == 0x4464) {
		// 0x7f = 20dBm
		pa_bias_clkduty = 0x00; // Per WDS suggestion
		if (power > 0x7f)
			power = 0x7f;
	}
	uint8_t power_properties[] = { 0x18, 0x00, 0x00 }; // PA_MODE from WDS sugggestions (why?)
	power_properties[1] = power;
	power_properties[2] = pa_bias_clkduty;
	set_properties(RH_RF24_PROPERTY_PA_MODE, power_properties,
			sizeof(power_properties));
}

// Caution: There was a bug in A1 hardware that will not handle 1 byte commands. 
bool RH_RF24::command(
		uint8_t cmd,
		const uint8_t* write_buf,
		uint8_t write_len,
		uint8_t* read_buf,
		uint8_t read_len) {

	bool done = false;

	assertSS();

	transfer(cmd);

	// Now write any write data
	if (write_buf && write_len) {
		while (write_len--) {
			// trace_printf("Sending arg %d\n", *write_buf);
			transfer(*write_buf++);
		}
	}

	// Sigh, the RFM26 at least has problems if we deselect too quickly :-(
	// Innocuous timewaster:
	timer_sleep(1);
	// And finalise the command
	// trace_printf("done with command %d\n", cmd);
	releaseSS();

	uint16_t count; // Number of times we have tried to get CTS
	for (count = 0; !done && count < RH_RF24_CTS_RETRIES; count++) {
		// Wait for the CTS
		timer_sleep(1);
		assertSS();

		transfer(RH_RF24_CMD_READ_BUF);
		if (transfer(0) == RH_RF24_REPLY_CTS) {
			// Now read any expected reply data
			if (read_buf && read_len) {
				while (read_len--) {
					*read_buf++ = transfer(0);
				}
			}
			done = true;
		}
		// Sigh, the RFM26 at least has problems if we deselect too quickly :-(
		// Innocuous timewaster:
		// digitalWrite(_slaveSelectPin, LOW);
		// Finalise the read
		releaseSS();
	}
	// ATOMIC_BLOCK_END;
	return done; // False if too many attempts at CTS
}

bool RH_RF24::configure(const uint8_t* commands) {
	// Command strings are constructed in radio_config_Si4460.h
	// Each command starts with a count of the bytes in that command:
	// <bytecount> <command> <bytecount-2 bytes of args/data>
	uint8_t next_cmd_len;

	while (memcpy(&next_cmd_len, commands, 1), next_cmd_len > 0) {
		// trace_printf("Next cmd length is %d\n", next_cmd_len);
		uint8_t buf[20]; // As least big as the biggest permitted command/property list of 15
		memcpy(buf, commands + 1, next_cmd_len);

		/*
		trace_printf("Next cmd is %d with args : ", buf[0]);
		for (uint8_t i=0; i<next_cmd_len; i++) {
			trace_printf("%d,", buf[i]);
		}
		trace_printf("\n");
*/
		command(buf[0], buf + 1, next_cmd_len - 1);
		commands += (next_cmd_len + 1);
	}
	return true;
}

void RH_RF24::shutdown_HW() {
	GPIOA->ODR |= (1 << RF24_SDN_BIT);
	end();
}

extern "C" boolean timer_sleep(uint32_t);

void RH_RF24::init_HW() {
	// Sigh: its necessary to control the SDN pin to reset this chip.
	// Tying it to GND does not produce reliable startups
	// Per Si44f64 Data Sheet 3.3.2
	// _delay_ms(10);
	begin();
	GPIOA->ODR &= ~(1 << RF24_SDN_BIT);
	timer_sleep(6);
}

bool RH_RF24::set_properties(uint16_t firstProperty, const uint8_t* values,
		uint8_t count) {
	uint8_t buf[15];

	buf[0] = firstProperty >> 8;   // GROUP
	buf[1] = count;                // NUM_PROPS
	buf[2] = firstProperty & 0xff; // START_PROP
	uint8_t i;
	for (i = 0; i < 12 && i < count; i++)
		buf[3 + i] = values[i]; // DATAn
	return command(RH_RF24_CMD_SET_PROPERTY, buf, count + 3);
}

bool RH_RF24::get_properties(uint16_t firstProperty, uint8_t* values,
		uint8_t count) {
	if (count > 16)
		count = 16;
	uint8_t buf[3];
	buf[0] = firstProperty >> 8;   // GROUP
	buf[1] = count;                // NUM_PROPS
	buf[2] = firstProperty & 0xff; // START_PROP
	return command(RH_RF24_CMD_GET_PROPERTY, buf, sizeof(buf), values, count);
}

float RH_RF24::get_temperature() {
	uint8_t write_buf[] = { 0x10 };
	uint8_t read_buf[8];
	// Takes nearly 4ms
	command(RH_RF24_CMD_GET_ADC_READING, write_buf, sizeof(write_buf), read_buf,
			sizeof(read_buf));
	uint16_t temp_adc = (read_buf[4] << 8) | read_buf[5];
	return ((800 + read_buf[6]) / 4096.0) * temp_adc - ((read_buf[7] / 2) + 256);
}

float RH_RF24::get_battery_voltage() {
	uint8_t write_buf[] = { 0x08 };
	uint8_t read_buf[8];
	// Takes nearly 4ms
	command(RH_RF24_CMD_GET_ADC_READING, write_buf, sizeof(write_buf), read_buf,
			sizeof(read_buf));
	uint16_t battery_adc = (read_buf[2] << 8) | read_buf[3];
	return 3.0 * battery_adc / 1280;
}

float RH_RF24::get_gpio_voltage(uint8_t gpio) {
	uint8_t write_buf[] = { 0x04 };
	uint8_t read_buf[8];
	write_buf[0] |= (gpio & 0x3);
	// Takes nearly 4ms
	command(RH_RF24_CMD_GET_ADC_READING, write_buf, sizeof(write_buf), read_buf,
			sizeof(read_buf));
	uint16_t gpio_adc = (read_buf[0] << 8) | read_buf[1];
	return 3.0 * gpio_adc / 1280;
}

// Defines the commands we can interrogate in printRegisters
typedef struct {
	uint8_t cmd;       ///< The command number
	uint8_t replyLen;  ///< Number of bytes in the reply stream (after the CTS)
} CommandInfo;

