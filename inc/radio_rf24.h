#ifndef __RADIO_RF24_H__
#define __RADIO_RF24_H__

#include "RH_RF24.h"

/*
class RadioRF24 {
private:
	RH_RF24 rf24;
	bool initWarm();

public:
	void setup();
	void ptt_on();
	void ptt_off();
	void shutdown();
};
*/

uint8_t si4463_init();
void si4463_PTTOn();
void si4463_PTTOff();
void si4463_shutdown();

#endif
