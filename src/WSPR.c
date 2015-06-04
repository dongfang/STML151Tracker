#include <diag/Trace.h>
#include <stdint.h>
#include <string.h>
#include "stm32l1xx_conf.h"
#include "DataTypes.h"
#include "GPS.h"

static uint8_t losslessCompressedBuf[11];
uint8_t convolutionalBuf[21];
uint8_t interleavedbuf[21];
uint8_t symbolList[162 / 4 + 1];

extern uint16_t hash(const char* call, uint16_t length);

// Must be length 6. Must have a number at 3rd position.
// Must have no numbers at last 3 positions.
// Valid symbols are [A..Z] and [0..9] and space.

static inline uint8_t encode(char c);
static uint8_t encode(char c) {
	if (c >= '0' && c <= '9')
		return c - '0';
	if (c >= 'A' && c <= 'Z')
		return 10 + c - 'A';
	if (c >= 'a' && c <= 'z')
		return 10 + c - 'a';
	if (c == ' ')
		return 36;
	trace_printf("Error! Unhandled char in callsign '%c'\n", c);
	return -1;
}

static inline uint8_t encodeCharOnly(char c);
static uint8_t encodeCharOnly(char c) {
	if (c >= 'A' && c <= 'Z')
		return c - 'A';
	if (c >= 'a' && c <= 'z')
		return c - 'a';
	if (c == ' ')
		return 26;
	trace_printf("Error! Unhandled char in callsign '%c'\n", c);
	return -1;
}

static uint32_t encodeCallsign(const char* callsign) {
	uint32_t result = encode(callsign[0]);
	result = result * 36 + encode(callsign[1]);
	result = result * 10 + encode(callsign[2]);
	result = result * 27 + encodeCharOnly(callsign[3]);
	result = result * 27 + encodeCharOnly(callsign[4]);
	result = result * 27 + encodeCharOnly(callsign[5]);
	return result;
}

static uint32_t encodeCallsignFaster(char* callsign) {
	uint32_t result = encode(callsign[0]) * ((uint32_t) 36 * 10 * 27 * 27 * 27);
	result += encode(callsign[1]) * ((uint32_t) 10 * 27 * 27 * 27);
	result += encode(callsign[2]) * ((uint32_t) 27 * 27 * 27);
	result += encode(callsign[3]) * ((uint32_t) 27 * 27);
	result += encode(callsign[4]) * ((uint32_t) 27);
	result += encode(callsign[5]);
	return result;
}

static uint32_t encode4DigitMaidenhead(const char* mh) {
	uint32_t result = 10 * (mh[0] - 'A') + (mh[2] - '0');
	result = 179 - result;
	result *= 180;
	result += 10 * (mh[1] - 'A') + (mh[3] - '0');
	return result;
}

static uint8_t encodePower(uint8_t power) {
	return power + 64;
}

static void encodeType1Message(const char* maidenhead4, uint8_t power) {

	char myPaddedCall[7];
	strcpy(myPaddedCall, MY_CALLSIGN);
	for (uint8_t i = strlen(MY_CALLSIGN); i < 6; i++)
		myPaddedCall[i] = ' ';

	uint32_t N = encodeCallsign(myPaddedCall) << 4;
	uint32_t maiden = encode4DigitMaidenhead(maidenhead4);
	uint32_t M = (maiden * 128 + encodePower(power)) << (32 - 22 - 4);

	uint8_t* Nsplit = (uint8_t*) &N;
	uint8_t* Msplit = (uint8_t*) &M;

	losslessCompressedBuf[0] = (Nsplit[3]);
	losslessCompressedBuf[1] = (Nsplit[2]);
	losslessCompressedBuf[2] = (Nsplit[1]);
	losslessCompressedBuf[3] = (Nsplit[0]);

	losslessCompressedBuf[3] |= Msplit[3];  // 4 used  and 0 free in buffer
	losslessCompressedBuf[4] = Msplit[2];   // 12 used and 0 free in buffer
	losslessCompressedBuf[5] = Msplit[1];   // 20 used and 0 free in buffer
	losslessCompressedBuf[6] = Msplit[0];   // 22 used and 6 free in buffer
	losslessCompressedBuf[7] = 0;   // 14 free in buffer
	losslessCompressedBuf[8] = 0;   // 22 free in buffer
	losslessCompressedBuf[9] = 0;   // 30 free in buffer
	losslessCompressedBuf[10] = 0;  // 31 free in buffer
}

static const int8_t powerSomething[] = { 0, -1, 1, 0, -1, 2, 1, 0, -1, 1 };
static void encodeType3Message(const char* maidenhead6, int8_t power) {

  	char rotatedMaidenhead[7];
	for (uint8_t i = 1; i < 6; i++)
		rotatedMaidenhead[i - 1] = maidenhead6[i];
	rotatedMaidenhead[5] = maidenhead6[0];
	rotatedMaidenhead[6] = 0;

	uint32_t n1 = encodeCallsign(rotatedMaidenhead);
	// trace_printf("n1 aka the num value of extended locator is %u\n", n1);
	n1 = n1<<4;

	uint32_t n2 = hash(MY_CALLSIGN, strlen(MY_CALLSIGN));

	//trace_printf("Power initially %d\n",power);
	if (power > 60)
		power = 60;

	power += powerSomething[power % 10];
	power = -(power + 1);
	// trace_printf("Power ultimately %d\n", power);

	// This is in arithmetic operations, not bitwise!
	n2 = n2 * 128 + power + 64;
	// trace_printf("n2 aka shifted callsign plus power is %u\n", n2);

	// n2 is in bits 0..21. We move that to 6..27
	n2 = n2 << (32 - 22 - 4);

	uint8_t* n1split = (uint8_t*) &n1;
        uint8_t* n2split = (uint8_t*) &n2;

	losslessCompressedBuf[0] = (n1split[3]);
        losslessCompressedBuf[1] = (n1split[2]);
        losslessCompressedBuf[2] = (n1split[1]);
        losslessCompressedBuf[3] = (n1split[0]);

        losslessCompressedBuf[3] |= n2split[3];  // 4 used  and 0 free in buffer
        losslessCompressedBuf[4] = n2split[2];   // 12 used and 0 free in buffer
        losslessCompressedBuf[5] = n2split[1];   // 20 used and 0 free in buffer
        losslessCompressedBuf[6] = n2split[0];   // 22 used and 6 free in buffer
        losslessCompressedBuf[7] = 0;   // 14 free in buffer
        losslessCompressedBuf[8] = 0;   // 22 free in buffer                                                        
        losslessCompressedBuf[9] = 0;   // 30 free in buffer                                                         
        losslessCompressedBuf[10] = 0;  // 31 free in buffer                  	
}

static inline uint8_t getBit(uint8_t* buf, uint8_t index);
static uint8_t getBit(uint8_t* buf, uint8_t index) {
	uint8_t b = buf[index >> 3];
	uint8_t mask = (1 << (7 - (index & 7)));
	uint8_t value = b & mask;
	return value == 0 ? 0 : 1;
}

static inline void setBit(uint8_t* buf, uint8_t index, uint8_t value01);
static void setBit(uint8_t* buf, uint8_t index, uint8_t value) {
	uint8_t idx = index >> 3;
	uint8_t mask = 1 << (7 - (index & 7));
	if (value)
		buf[idx] |= mask;
	else
		buf[idx] &= ~mask;
}

/*
 void printBuf(const char* name, uint8_t* buf, uint8_t len) {
 printf("%s:\n", name);
 for (uint8_t i=0; i<len; i++) {
 printf("%d ", getBit(buf, i));
 if (i % 16 == 15) printf("\n");
 }
 printf("\n\n");
 }
 */

static inline uint8_t bitParity(uint32_t v);
static uint8_t bitParity(uint32_t v) {
	v ^= v >> 16;
	v ^= v >> 8;
	v ^= v >> 4;
	v &= 0xf;
	return (0x6996 >> v) & 1;
}

static inline uint8_t stupidBitParity(uint32_t v);
static uint8_t stupidBitParity(uint32_t v) {
	uint32_t mask = 1;
	uint32_t cnt = 0;
	for (uint8_t i = 0; i < 32; i++) {
		if ((v & mask) != 0)
			cnt++;
		mask <<= 1;
	}
	return cnt & 1;
}

static void convolutionalEncoding() {
	uint32_t shift1 = 0;
	uint32_t shift2 = 0;
	uint8_t pos = 0;

	uint8_t idx;
	for (idx = 0; idx < 81; idx++) {
		uint8_t bit = getBit(losslessCompressedBuf, idx);
		shift1 = (shift1 << 1) | bit;
		shift2 = (shift2 << 1) | bit;

		uint32_t x = shift1 & 0xF2D05351UL;
		uint8_t bitp = stupidBitParity(x);
		uint8_t bitp2 = bitParity(x);

		if (bitp != bitp2)
			trace_printf("ERROR! bitp\n");

		setBit(convolutionalBuf, pos++, bitp);

		x = shift2 & 0xE4613C47UL;
		bitp = stupidBitParity(x);
		setBit(convolutionalBuf, pos++, bitp);
	}
}

static uint8_t bitReverse(uint8_t v) {
	uint8_t result = 0;
	uint8_t maskin = 1;
	uint8_t maskout = 1 << 7;
	int8_t bit;
	for (bit = 7; bit >= 0; bit--) {
		if (v & maskin)
			result |= maskout;
		maskin <<= 1;
		maskout >>= 1;
	}
	return result;
}

static uint8_t fasterBitReverse(uint8_t v) {
  uint8_t r = v; // r will be reversed bits of v; first get LSB of v
  int s = 7; // extra shift needed at end
  
  for (v >>= 1; v; v >>= 1)
    {   
      r <<= 1;
      r |= v & 1;
      s--;
    }
  return r << s; // shift when v's highest bits are zero
}

static void interleave() {
  uint8_t p = 0;
  for (uint16_t i = 0; i < 256; i++) {
    uint8_t j = fasterBitReverse(i);
    if (j < 162) {
      uint8_t value = getBit(convolutionalBuf, p);
      setBit(interleavedbuf, j, value);
      p++;
    }
  }
}

static const uint8_t SYNC_VECTOR[162] = { 1, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1,
		1, 1, 0, 0, 0, 1, 0, 0, 1, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
		1, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 0, 1, 1,
		0, 1, 0, 0, 0, 0, 1, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 0, 1, 0, 0, 1, 0, 1,
		1, 0, 0, 0, 1, 1, 0, 1, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0,
		0, 1, 1, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0,
		1, 0, 1, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 1, 0, 1, 1, 0, 0, 0,
		1, 1, 0, 0, 0 };

static inline uint8_t readSyncVectorSym(uint8_t idx);
static uint8_t readSyncVectorSym(uint8_t idx) {
	return SYNC_VECTOR[idx];
}

static void makeSymbolList() {
	for (uint8_t i = 0; i < 162; i++) {
		uint8_t idx = i / 4;
		uint8_t shift = i & 3;
		if (shift == 0)
			symbolList[idx] = 0;
		uint8_t value = getBit(interleavedbuf, i);
		uint8_t sym = readSyncVectorSym(i) + (value ? 2 : 0);
		symbolList[idx] |= sym << (6 - shift * 2);
	}
}

void completeMessage() {
	convolutionalEncoding();
	interleave();
	makeSymbolList();
}

void prepareType1Transmission(const char* maidenhead4, uint8_t power) {
	encodeType1Message(maidenhead4, power);
	completeMessage();
}

void prepareType3Transmission(const char* maidenhead6, uint8_t power) {
	encodeType3Message(maidenhead6, power);
	completeMessage();
}

uint8_t getWSPRSymbol(uint8_t i) {
	uint8_t idx = i / 4;
	uint8_t shift = i & 3;
	uint8_t sym = symbolList[idx] >> (6 - shift * 2);
	sym &= 3;
	return sym;
}

void prepareWSPRMessage(uint8_t type) {
  char maidenhead[7];
  switch (type) {
  case 1:
    currentPositionAs4DigitMaidenhead(maidenhead);
    prepareType1Transmission(maidenhead, WSPR_POWER_LEVEL);
    break;
  case 3:
    currentPositionAs6DigitMaidenhead(maidenhead);
    prepareType3Transmission(maidenhead, WSPR_POWER_LEVEL);
    break;
  }
  trace_printf("Maidenhead: %s\n", maidenhead);
}
