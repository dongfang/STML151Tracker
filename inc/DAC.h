/*
 * DAC.h
 *
 *  Created on: Jun 3, 2015
 *      Author: dongfang
 */

#ifndef DAC_H_
#define DAC_H_

// WSPR stuff - no DMA.
// Setup DAC ch 2 but leave Ch1 floating
void setupChannel2DACForWSPR();
void setDAC2(uint16_t value);

// AFSK stuff - uses DMA
void AFSK_init();
void AFSK_shutdown();
void AFSK_startTransmission();

extern volatile uint16_t packet_cnt;
extern volatile uint8_t packetTransmissionComplete;

#endif /* DAC_H_ */
