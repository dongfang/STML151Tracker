#ifndef __RADIO_RF24_H__
#define __RADIO_RF24_H__

#include "Types.h"

#ifdef __cplusplus
extern "C" {
#endif
//class RadioRF24 {
//private:
boolean RF24_initWarm(uint32_t frequency, uint8_t txPower);

// public:
void RF24_initHW();
void RF24_transmit(uint32_t frequency, uint8_t txPower);
void RF24_stopTransmitting();
void RF24_shutdown();
// };

#ifdef __cplusplus
}
#endif
#endif
