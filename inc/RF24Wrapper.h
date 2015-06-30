#ifndef __RADIO_RF24_H__
#define __RADIO_RF24_H__

#include "RH_RF24.h"
#include "Types.h"

//class RadioRF24 {
//private:
RH_RF24 rf24;
boolean RF24_initWarm(uint32_t frequency, uint8_t txPower);

// public:
void RF24_setup();
void RF24_transmit(uint32_t frequency, uint8_t txPower);
void RF24_stopTransmitting();
void RF24_shutdown();
// };

#endif
