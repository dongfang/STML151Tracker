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
#include "RF24Wrapper.h"
#include "RH_RF24.h"

#ifdef DEBUG
#include <stdio.h>
#include "SerialStream.h"
extern UARTSerial serial0;
#endif

static RH_RF24 rf24;

extern "C" {

boolean RF24_initWarm(uint32_t frequency, uint8_t txPower) {
	if (!rf24.initWarm()) {
		return false;
	}
//	uint8_t trim = OSC_TRIM;
//	rf24.set_properties(RH_RF24_PROPERTY_GLOBAL_XO_TUNE, &trim, 1);
	rf24.setFrequency(frequency/1000);
	rf24.setTxPower(txPower);
	rf24.setIdleMode(RH_RF24_DEVICE_STATE_SLEEP);
	return true;
}

void RF24_initHW() {
	rf24.init_HW();
}

void RF24_transmit(uint32_t frequency, uint8_t txPower) {
	// Experimental
	if (RF24_initWarm(frequency, txPower)) {
	} else {
	}
	rf24.setModeTx();
}

void RF24_stopTransmitting() {
	rf24.setModeIdle();
}

void RF24_shutdownHW() {
	rf24.shutdown_HW();
}
}
