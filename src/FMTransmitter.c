/*
 * FMTransmitter.c
 *
 *  Created on: Jun 5, 2015
 *      Author: dongfang
 */

#include "FMTransmitter.h"
#include "DAC.h"

void AFSK_startTransmission() {
	AFSK_init();
	packet_cnt = 0;
  packetTransmissionComplete = 0;
}

uint8_t AFSK_ended() {
	return packetTransmissionComplete;
}

void AFSK_endTransmission() {
	AFSK_shutdown();
}
