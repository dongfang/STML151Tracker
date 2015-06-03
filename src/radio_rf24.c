/* trackuino copyright (C) 2010  EA5HAV Javi
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */
#include "radio_rf24.h"
#include "radio_config_Si4460.h"

#ifdef DEBUG
#include <stdio.h>
#include "SerialStream.h"
extern UARTSerial serial0;
#endif

void si4463_configure();
void si4463_setModemConfig();
void si4463_setProperties(uint8_t address, uint8_t* data, uint8_t length);

const uint8_t RFM26_CONFIGURATION_DATA[] =
		RADIO_CONFIGURATION_DATA_ARRAY;

uint8_t si4463_init() {
	float frequency;

	// Initialise the radio
	power_on_reset();

	si4463_configure(RFM26_CONFIGURATION_DATA);

	// Get the device type and check it
	// This also tests whether we are really connected to a device
	uint8_t buf[8];
	if (!command(RH_RF24_CMD_PART_INFO, 0, 0, buf, sizeof(buf)))
		return 0; // SPI error? Not connected?

	uint16_t deviceType = (buf[1] << 8) | buf[2];
	// Check PART to be either 0x4460, 0x4461, 0x4463, 0x4464
	if (deviceType != 0x4460 && deviceType != 0x4461 && deviceType != 0x4463
			&& deviceType != 0x4464)
		return 0; // Unknown radio type, or not connected

	si4463_setModemConfig(EXPERIMENTAL_CW);

	uint8_t trim = OSC_TRIM;
	si4463_setProperties(RH_RF24_PROPERTY_GLOBAL_XO_TUNE, &trim, 1);

	frequency = 144.8;

	rf24.setFrequency(frequency + FREQ_CORRECTION);
	rf24.setTxPower(_4463_POWER);
	rf24.setIdleMode(RH_RF24_DEVICE_STATE_SLEEP);
	return 1;
}

void RadioRF24::ptt_on() {
	// Experimental
	if (initWarm()) {
#ifdef DEBUG
		printf_P(PSTR("RF24 OK\r\n"));
		serial0.flush();
#endif
	} else {
#ifdef DEBUG
		printf_P(PSTR("Error @ warm init RF24!\r\n"));
		serial0.flush();
#endif
	}
	rf24.setModeTx();
}

void RadioRF24::ptt_off() {
	rf24.setModeIdle();
}

void si4463_shutdown() {
	// Anyway here we go:
	// Set the shutdown line high (we might also just use the sleep feature of the radio)
	// Disable SPI (maybe).
}

