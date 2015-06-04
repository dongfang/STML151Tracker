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

/* Credit to:
 *
 * Michael Smith for his Example of Audio generation with two timers and PWM:
 * http://www.arduino.cc/playground/Code/PCMAudio
 *
 * Ken Shirriff for his Great article on PWM:
 * http://arcfn.com/2009/07/secrets-of-arduino-pwm.html 
 *
 * The large group of people who created the free AVR tools.
 * Documentation on interrupts:
 * http://www.nongnu.org/avr-libc/user-manual/group__avr__interrupts.html
 */

#include "config.h"
#include "afsk_avr.h"
#include "radio_rf24.h"
#include <avr/interrupt.h>
#include <stdint.h>

#ifdef DEBUG
#include <stdio.h>
#include "SerialStream.h"
extern UARTSerial serial0;
#endif

#define TONE_1200 1200UL
#define TONE_2200 2200UL

// Module consts

// The actual baudrate after rounding errors will be:
// PLAYBACK_RATE / (integer_part_of((PLAYBACK_RATE * 256) / BAUD_RATE) / 256)
static const uint16_t BAUD_RATE = 1200;
static const uint16_t SAMPLES_PER_BAUD = ((uint32_t) PLAYBACK_RATE << 8)
		/ BAUD_RATE;  // Fixed point 8.8 . Num PWM cycles per baud * 256.
static const uint16_t PHASE_DELTA_1200 = (((TABLE_SIZE * TONE_1200) << 7)
		/ PLAYBACK_RATE); // Fixed point 9.7
static const uint16_t PHASE_DELTA_2200 = (((TABLE_SIZE * TONE_2200) << 7)
		/ PLAYBACK_RATE);
static const uint8_t SAMPLE_FIFO_SIZE = 32;

// Module globals
volatile static uint8_t current_byte;
volatile static uint16_t current_sample_in_baud; // 1 bit = SAMPLES_PER_BAUD samples
volatile static bool go = false;                         // Modem is on
volatile static uint16_t phase_delta;            // 1200/2200 for standard AX.25
volatile static uint16_t phase;            // Fixed point 9.7 (2PI = TABLE_SIZE)
volatile static uint16_t packet_pos;                  // Next bit to be sent out
volatile static uint8_t sample_fifo[SAMPLE_FIFO_SIZE];   // queue of samples
volatile static uint8_t sample_fifo_head = 0;         // empty when head == tail
volatile static uint8_t sample_fifo_tail = 0;
volatile static uint32_t sample_overruns = 0;

// The radio (class defined in config.h)
static RadioRF24 radio;

volatile static uint16_t afsk_packet_size = 0;
volatile static const uint8_t *afsk_packet;

extern void measureLoadedVoltage();

// Module functions

inline static bool afsk_is_fifo_full() {
	return (((sample_fifo_head + 1) % SAMPLE_FIFO_SIZE) == sample_fifo_tail);
}

inline static bool afsk_is_fifo_full_safe() {
	cli();
	bool b = afsk_is_fifo_full();
	sei();
	return b;
}

inline static bool afsk_is_fifo_empty() {
	return (sample_fifo_head == sample_fifo_tail);
}

inline static bool afsk_is_fifo_empty_safe() {
	cli();
	bool b = afsk_is_fifo_empty();
	sei();
	return b;
}

inline static void afsk_fifo_in(uint8_t s) {
	sample_fifo[sample_fifo_head] = s;
	sample_fifo_head = (sample_fifo_head + 1) % SAMPLE_FIFO_SIZE;
}

inline static void afsk_fifo_in_safe(uint8_t s) {
	cli();
	afsk_fifo_in(s);
	sei();
}

inline static uint8_t afsk_fifo_out() {
	uint8_t s = sample_fifo[sample_fifo_tail];
	sample_fifo_tail = (sample_fifo_tail + 1) % SAMPLE_FIFO_SIZE;
	return s;
}

// Exported functions
void afsk_setup() {
	// Start radio
	radio.setup();
}

void afsk_shutdown() {
	radio.shutdown();
}

void afsk_send(const uint8_t *buffer, int len) {
	afsk_packet_size = len;
	afsk_packet = buffer;
}

void afsk_start() {
	phase_delta = PHASE_DELTA_1200;
	phase = 0;
	packet_pos = 0;
	current_sample_in_baud = 0;
	go = true;

	// Prime the fifo
	afsk_flush();

	// Start timer (CPU-specific)
	afsk_timer2_setup();

	// Key the radio
	radio.ptt_on();

	// Start transmission
	afsk_timer2_start();
}

// Apparently returns true as long as the transmission it not completed.
bool afsk_flush() {
	while (!afsk_is_fifo_full_safe()) {
		// If done sending packet

		if (packet_pos == afsk_packet_size) {
			go = false;         // End of transmission
		}

		if (!go) {
			if (afsk_is_fifo_empty_safe()) {
				afsk_timer2_stop();  // Disable modem
				measureLoadedVoltage(); // for next time.
				//_delay_ms(10);
				radio.ptt_off();    // Release PTT
				return false;       // Done
			} else {
				return true;
			}
		}

		// If sent SAMPLES_PER_BAUD already, go to the next bit
		if (current_sample_in_baud < (1 << 8)) {    // Load up next bit
			if ((packet_pos & 7) == 0) {         // Load up next byte
				current_byte = afsk_packet[packet_pos >> 3];
			} else {
				current_byte = current_byte / 2;  // ">>1" forces int conversion
			}
			if ((current_byte & 1) == 0) {
				// Toggle tone (1200 <> 2200), NRZI
				phase_delta ^= (PHASE_DELTA_1200 ^ PHASE_DELTA_2200); // Flip all bits which are different :)
			}
		}

#ifdef MODULATION_TEST
#ifdef MODULATION_TEST_SILENCE
		phase = 0;
#endif
#ifdef MODULATION_TEST_LOWTONE
		phase += PHASE_DELTA_1200 + PHASE_DELTA_2200 * 1/3;
#endif
#ifndef MODULATION_TEST_LOWTONE
		phase += PHASE_DELTA_1200 + PHASE_DELTA_2200 * 2/3;
#endif
#else
		phase += phase_delta;
#endif
		uint8_t s = afsk_read_sample((phase >> 7) & (TABLE_SIZE - 1));

		afsk_fifo_in_safe(s);

		current_sample_in_baud += (1 << 8);
		if (current_sample_in_baud >= SAMPLES_PER_BAUD) {
			packet_pos++;
			current_sample_in_baud -= SAMPLES_PER_BAUD;
		}
	}

	return true;  // still working
}

// This is called at PLAYBACK_RATE Hz to load the next sample.
AFSK_ISR {
	if (afsk_is_fifo_empty()) {
		if (go) {
			sample_overruns++;
		}
	} else {
		afsk_output_sample(afsk_fifo_out());
	}
}

