#include "Setup.h"

// For ground test, see the DummyGPS source.
#if MODE==FLIGHT

#include "GPS.h"
#include "stm32l1xx.h"
#include <math.h>
#include <string.h>
#include "diag/trace.h"
#include "systick.h"
#include "Globals.h"

enum {
	STATE_IDLE,
	STATE_READ_ID,
	STATE_DATA,
	STATE_CHECKSUM1,
	STATE_CHECKSUM2,
	STATE_READ_BINARY_UBX_SYNC_2,
	STATE_READ_BINARY_UBX_CLASS_ID,
	STATE_READ_BINARY_UBX_MSG_ID,
	STATE_READ_BINARY_UBX_MSG_LEN1,
	STATE_READ_BINARY_UBX_MSG_LEN2,
	STATE_READ_BINARY_UBX_MSG_BODY,
	STATE_READ_BINARY_UBX_MSG_CHECKA,
	STATE_READ_BINARY_UBX_MSG_CHECKB,
} NMEA_PARSER_STATES;

enum {
	GPVTG,	// Track made good and ground speed
	GPGGA,	// Global Positioning System Fix Data
	GPRMC,	// Recommended minimum specific GPS/Transit data
	GPGSA,	// GPS DOP and active satellites
	GPGSV,	// Detailed Satellite data
	GPGLL	// Lat/Lon data
} messagesParsed;

static uint8_t dataindex;
static uint8_t commaindex;
static uint8_t state;
static uint8_t commitCheck;

// static uint8_t _UBX_POLL_NAV5_MESSAGE[] = UBX_POLL_NAV5_MESSAGE;
static uint8_t _UBX_INIT_NAV5_MESSAGE[] = UBX_INIT_NAV5_MESSAGE;

//static UBX_MESSAGE POLL_NAV_MESSAGE = { sizeof(_UBX_POLL_NAV5_MESSAGE),
//		_UBX_POLL_NAV5_MESSAGE };

static UBX_MESSAGE INIT_NAV_MESSAGE = { sizeof(_UBX_INIT_NAV5_MESSAGE),
		_UBX_INIT_NAV5_MESSAGE };

// Unsafes are always valid (no parser construction site) but are written to from within
// interrupt handler.
// Interested consumers should disable interrupts, copy the unsafes into a their locally
// managed safe copies and re-enable interrupts.
NMEA_TimeInfo_t nmeaTimeInfo_unsafe;
NMEA_CRS_SPD_Info_t nmeaCRSSPDInfo_unsafe;
Location_t nmeaPositionInfo_unsafe;
NMEA_StatusInfo_t nmeaStatusInfo_unsafe;

uint8_t nmea_parse(char c);

static UBX_MESSAGE* currentSendingMessage;
static int8_t currentSendingIndex;
static uint8_t currentChecksumA;
static uint8_t currentChecksumB;
static boolean busySendingMessage;

static uint8_t readClassId;
static uint8_t readMessageId;
static uint16_t readBodyLength;
static uint16_t readBodyCnt;

static uint8_t ackedClassId;
static uint8_t ackedMessageId;
static uint32_t lastSendConfigurationTime;

static boolean navSettingsConfirmed;

static void continueSendingUBXMessage() {
	if (currentSendingIndex == -2) {
		USART_SendData(USART1, 0xB5);
		currentSendingIndex++;
	} else if (currentSendingIndex == -1) {
		USART_SendData(USART1, 0x62);
		currentSendingIndex++;
	} else if (currentSendingIndex < currentSendingMessage->length) {
		uint8_t data = currentSendingMessage->message[currentSendingIndex];
		USART_SendData(USART1, data);
		currentChecksumA += data;
		currentChecksumB += currentChecksumA;
		currentSendingIndex++;
	} else if (currentSendingIndex == currentSendingMessage->length) {
		USART_SendData(USART1, currentChecksumA);
		currentSendingIndex++;
	} else if (currentSendingIndex == currentSendingMessage->length + 1) {
		USART_SendData(USART1, currentChecksumB);
		currentSendingIndex++;
	} else if (currentSendingIndex == currentSendingMessage->length + 2) {
		// we want to delay the clearing till the usart tx buffer is empty.. hence this extra case.
		busySendingMessage = false;
		USART_ITConfig(USART1, USART_IT_TXE, DISABLE);
	}
}

static void beginSendUBXMessage(UBX_MESSAGE* message) {
	if (!busySendingMessage) {
		currentSendingMessage = message;
		currentSendingIndex = -2;
		currentChecksumA = 0;
		currentChecksumB = 0;
		busySendingMessage = true;
		continueSendingUBXMessage();
	}
	USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
}

void setupUSART1() {
	USART_InitTypeDef USART_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	// RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

	/* Configure USART1 pins:  Rx and Tx ----------------------------*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF; 		// Alternate func
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	// We use A9,10 for USART1
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);

	/* Enable USART1 IRQ (on the NVIC) */
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* Set USART parameters */
	USART_InitStructure.USART_BaudRate = BAUDRATE;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl =
	USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART1, &USART_InitStructure);

	USART_Cmd(USART1, ENABLE);

	/* Enable USART1 IRQ (on the USART hw) */
	// USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
}

void USART1_IRQHandler(void) {
	if (USART_GetITStatus(USART1, USART_IT_TXE) != RESET) // Transmit the string in a loop
			{
		continueSendingUBXMessage();
	}

	if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) // Received characters modify string
			{
		uint16_t rxd = USART_ReceiveData(USART1);
		if (DEBUG_GPS_DATA)
			trace_putchar(rxd);
		nmea_parse(rxd);
	}
}


/*
 void printNMEA_TimeInfo() {
 uint32_t itod = nmeaTimeInfo_unsafe.itod;
 int millis = itod % 1000;
 itod = itod / 1000;
 int seconds = itod % 60;
 itod = itod / 60;
 int minutes = itod % 60;
 itod = itod / 60;
 int hours = itod % 60;

 trace_printf("NMEA_TimeInfo_t\r\n");
 trace_printf("itod: %u (%0d:%0d:%0d.%000d)\r\n", nmeaTimeInfo_unsafe.itod,
 hours, minutes, seconds, millis);
 }

 void printNMEA_CRS_SPD_Info() {
 printf("NMEA_CRS_SPD_Info\r\n");
 printf("crs: %f spd %f\r\n", nmeaCRSSPDInfo_unsafe.course, nmeaCRSSPDInfo_unsafe.groundSpeed);
 }

 void printNMEA_PositionInfo() {
 uint32_t itod = nmeaPositionInfo_unsafe.fixTimeUTC;
 int millis = itod % 1000;
 itod = itod / 1000;
 int seconds = itod % 60;
 itod = itod / 60;
 int minutes = itod % 60;
 itod = itod / 60;
 int hours = itod % 60;
 printf("FixTime: %u (%0d:%0d:%0d.%000d) ", nmeaPositionInfo_unsafe.fixTimeUTC, hours, minutes, seconds, millis);
 printf("lat %f, lon %f, alt %f, valid %c\r\n", nmeaPositionInfo_unsafe.lat,nmeaPositionInfo_unsafe.lon,nmeaPositionInfo_unsafe.alt, nmeaPositionInfo_unsafe.validity);
 }

 void printNMEA_StatusInfo() {
 printf("NMEA_StatusInfo\r\n");
 printf("FixMode %d, NumSat %d, horizAcc %f\r\n", nmeaStatusInfo_unsafe.fixMode, nmeaStatusInfo_unsafe.numberOfSatellites, nmeaStatusInfo_unsafe.horizontalAccuracy);
 }
 */
MessageState latestGPSState = CONSUMED;

// Parse one character of a NMEA time.
// apparently writes into a millis value at end.
static void parseTime(char c, uint8_t* state, Time_t* value) {
	switch (++(*state)) {
	case 1:
		value->hours = (c - '0') * 10;
		break;
	case 2:
		value->hours += (c - '0');
		break;
	case 3:
		value->minutes = (c - '0') * 10;
		break;
	case 4:
		value->minutes += (c - '0');
		break;
	case 5:
		value->seconds = (c - '0') * 10;
		break;
	case 6:
		value->seconds += (c - '0');
		break;
	default:
		break;
	}
}

static void parseDegrees(char c, uint8_t* state, double* value) {
	static uint32_t ivalue;
	uint8_t i;
	double temp;
	switch (++(*state)) {
	case 1:
		ivalue = (c - '0') * 1000;
		break;
	case 2:
		ivalue += (c - '0') * 100;
		break;
	case 3:
		ivalue += (c - '0') * 10;
		break;
	case 4:
		ivalue += (c - '0') * 1;
		break;
	case 5: {
		if (c == '.') {
			*state = 9;
		} else {
			ivalue *= 10;
			ivalue += (c - '0') * 1;
		}
		break;
	}
	case 6: {
		*state = 9;
		break;
	}
	case 10:
		temp = ivalue % 100;
		*value = ivalue / 100 + temp / 60.0;
		// no break;
	default:
		temp = (c - '0') / 60.0;
		for (i = *state - 9; i > 0; i--) {
			temp /= 10;
		}
		*value += temp;
		break;
	}
}

static void parseFloat(char c, uint8_t *state, float* value) {
	uint8_t i;
	if (c == '.') {
		*state = 1;
	} else if (!*state) {
		*value *= 10;
		*value += (c - '0');
	} else {
		float digit = (c - '0');
		for (i = *state; i > 0; i--) {
			digit /= 10;
		}
		*value += digit;
		(*state)++;
	}
}

static void parseDate(char c, uint8_t* state, Date_t* value) {
	switch (++(*state)) {
	case 1:
		value->date = (c - '0') * 10;
		break;
	case 2:
		value->date += (c - '0');
		break;
	case 3:
		value->month = (c - '0') * 10;
		break;
	case 4:
		value->month += (c - '0');
		break;
	case 5:
		value->year100 = (c - '0') * 10;
		break;
	case 6:
		value->year100 += (c - '0');
		break;
	default:
		break;
	}
}

static void parseInt8(char c, uint8_t* value) {
	*value *= 10;
	uint8_t digit = (c - '0');
	*value += digit;
}

static double tempLat;
static double tempLon;
static float tempFloat;
static float tempFloat2;
static Time_t tempTime;
static Date_t tempDate;
static char tempChar;
static uint8_t tempU8;
static uint8_t tempU82;

static void parseGPVTG(char c) {
	static uint8_t state;
	if (c == ',') {
		state = 0;
		if (commaindex == 0) {
			tempFloat = 0;
			tempFloat2 = 0;
		} else if (commaindex == 5) {
			nmeaCRSSPDInfo_unsafe.course = tempFloat;
			nmeaCRSSPDInfo_unsafe.groundSpeed = tempFloat2 * 0.514444; // knots to m/s
			commitCheck |= 1;
		}
		commaindex++;
	} else {
		switch (commaindex) {
		case 1:
			parseFloat(c, &state, &tempFloat);
			break;
		case 5:
			parseFloat(c, &state, &tempFloat2);
			break;
		default:
			break;
		}
	}
}

void parseGPGGA(char c) {
	static uint8_t state;
	if (c == ',') {
		state = 0;
		if (commaindex == 0) {
			tempLat = 0;
			tempLon = 0;
			tempU8 = 0;
			tempU82 = 0;
			tempFloat = 0;
			tempFloat2 = 0;
		} else if (commaindex == 9) {
			//debugTime("GGA", &tempTime);
			nmeaTimeInfo_unsafe.time = tempTime;
			nmeaTimeInfo_unsafe.time.valid = 1;
			nmeaPositionInfo_unsafe.lat = tempLat;
			nmeaPositionInfo_unsafe.lon = tempLon;
			nmeaStatusInfo_unsafe.fixMode = tempU8;
			nmeaStatusInfo_unsafe.numberOfSatellites = tempU82;
			nmeaStatusInfo_unsafe.horizontalAccuracy = tempFloat;
			nmeaPositionInfo_unsafe.alt = tempFloat2;
			commitCheck |= 2;
		}
		commaindex++;
	} else {
		switch (commaindex) {
		case 1:
			parseTime(c, &state, &tempTime);
			break;
		case 2:
			parseDegrees(c, &state, &tempLat);
			break;
		case 3:
			if (c == 'S')
				tempLat = -tempLat;
			break;
		case 4:
			parseDegrees(c, &state, &tempLon);
			break;
		case 5:
			if (c == 'W')
				tempLon = -tempLon;
			break;
		case 6:
			parseInt8(c, &tempU8);
			break;
		case 7:
			parseInt8(c, &tempU82);
			break;
		case 8:
			parseFloat(c, &state, &tempFloat);
			break;
		case 9:
			parseFloat(c, &state, &tempFloat2);
			break;
		default:
			break;
		}
	}
}

void parseGPRMC(char c) {
	static uint8_t state;
	if (c == ',') {
		state = 0;
		if (commaindex == 0) {
			tempFloat = 0;
			tempFloat2 = 0;
			tempLat = 0;
			tempLon = 0;
		} else if (commaindex == 9) {
			// debugTime("RMC", &tempTime);
			nmeaPositionInfo_unsafe.fixTime = tempTime;
			nmeaPositionInfo_unsafe.fixTime.valid = 1;
			nmeaPositionInfo_unsafe.valid = tempChar;
			nmeaPositionInfo_unsafe.lat = tempLat;
			nmeaPositionInfo_unsafe.lon = tempLon;
			nmeaCRSSPDInfo_unsafe.groundSpeed = tempFloat * 0.514444; // knots to m/s
			nmeaCRSSPDInfo_unsafe.course = tempFloat2;
			nmeaTimeInfo_unsafe.date = tempDate;
			nmeaTimeInfo_unsafe.date.valid = 1;
			commitCheck |= 4;
		}
		commaindex++;
	} else {
		switch (commaindex) {
		case 1:
			parseTime(c, &state, &tempTime);
			break;
		case 2:
			tempChar = c;
			break;
		case 3:
			parseDegrees(c, &state, &tempLat);
			break;
		case 4:
			if (c == 'S')
				tempLat = -tempLat;
			break;
		case 5:
			parseDegrees(c, &state, &tempLon);
			break;
		case 6:
			if (c == 'W')
				tempLon = -tempLon;
			break;
		case 7:
			parseFloat(c, &state, &tempFloat);
			break;
		case 8:
			parseFloat(c, &state, &tempFloat2);
			break;
		case 9:
			parseDate(c, &state, &tempDate);
			break;
		default:
			break;
		}
	}
}

void parseGPGSA(char c) {
// Ignore. Not important.
	commitCheck |= 16;
}

void parseGPGSV(char c) {
// Ignore GSV
	if (c == ',') {
		commaindex++;
		if (DEBUG_GPS_SNR && commaindex >= 5 && (commaindex-5) % 4 == 0) {
			// between SV number and elevation data
				trace_putchar(':');
		}
		if (DEBUG_GPS_SNR && commaindex >= 4 && commaindex % 4 == 0) {
			// Before SV number
			trace_putchar('\n');
		}
	} else {
		switch (commaindex) {
		case 1:
			break;
		case 2:
			break;
		case 3:
			break;
		case 4:
		case 12:
		case 16:
		case 20:
		// PRN number;
			if (DEBUG_GPS_SNR) {
				trace_putchar(c);
			}
			break;
		case 5:
		case 13:
		case 17:
		case 21:
		// elevation
			break;
		case 6:
		case 14:
		case 18:
		case 22:
		// azimuth
			break;
		case 7:
		case 15:
		case 19:
		case 23:
		// SNR
			if (DEBUG_GPS_SNR) {
				trace_putchar(c);
			}
			break;
		}
	}
	commitCheck |= 32;
}

void parseGPGLL(char c) {
	static uint8_t state;
	if (c == ',') {
		state = 0;
		if (commaindex++ == 0) {
			tempLat = 0;
			tempLon = 0;
		}
	} else {
		switch (commaindex) {
		case 1:
			parseDegrees(c, &state, &tempLat);
			break;
		case 2:
			if (c == 'S')
				tempLat = -tempLat;
			break;
		case 3:
			parseDegrees(c, &state, &tempLon);
			break;
		case 4:
			if (c == 'W')
				tempLon = -tempLon;
			break;
		case 5:
			parseTime(c, &state, &tempTime);
			break;
		case 6:
			tempChar = c;
			break;
		case 7:
			//debugTime("GLL", &tempTime);
			nmeaPositionInfo_unsafe.lat = tempLat;
			nmeaPositionInfo_unsafe.lon = tempLon;
			nmeaPositionInfo_unsafe.fixTime = tempTime;
			nmeaTimeInfo_unsafe.time = tempTime;
			nmeaTimeInfo_unsafe.time.valid = 1;
			nmeaPositionInfo_unsafe.valid = tempChar;
			commitCheck |= 8;
			break;
		default:
			break;
		}
	}
}

uint8_t char2hexdigit(uint8_t c) {
	if (c >= 'A' && c <= 'F')
		return c - 'A' + 10;
	if (c >= 'a' && c <= 'f')
		return c - 'a' + 10;
	return c - '0';
}

static char id[5];
uint8_t nmea_parse(char c) {
	static char sentence;
	static uint8_t checksum;
	//trace_printf("GPS state %d, in %d\n", state, c);
	switch (state) {
	case STATE_IDLE:
		if (c == '$') {
			state = STATE_READ_ID;
			dataindex = 0;
			checksum = 0;
		} else if (c == 0xb5) {
			state = STATE_READ_BINARY_UBX_SYNC_2;
		}

		// trigger sending a message, until confirmed.
		if (!navSettingsConfirmed && !busySendingMessage
				&& systemTimeMillis >= lastSendConfigurationTime + 1000) {
			ackedClassId = 0;
			ackedMessageId = 0; // clear any prior acknowledge.
			beginSendUBXMessage(&INIT_NAV_MESSAGE);
			lastSendConfigurationTime = systemTimeMillis;
			trace_printf("Sending GPS conf. message\n");
		}
		break;
	case STATE_READ_ID:
		id[dataindex++] = c;
		checksum ^= c;
		if (dataindex == 5) {
			dataindex = 0;
			commaindex = 0;
			state = STATE_DATA;
			if (strncmp("GPVTG", id, 5) == 0) {
				sentence = GPVTG;
				//trace_printf("GPVTG\n");
			} else if (strncmp("GPGGA", id, 5) == 0) {
				sentence = GPGGA;
				//trace_printf("GPGGA\n");
			} else if (strncmp("GPRMC", id, 5) == 0) {
				sentence = GPRMC;
				//trace_printf("GPRMC\n");
			} else if (strncmp("GPGSA", id, 5) == 0) {
				sentence = GPGSA;
				//trace_printf("GPGSA\n");
			} else if (strncmp("GPGSV", id, 5) == 0) {
				sentence = GPGSV;
				//trace_printf("GPGSV\n");
			} else if (strncmp("GPGLL", id, 5) == 0) {
				sentence = GPGLL;
				//trace_printf("GPGLL\n");
			} else {
				// printf("Got an unknown message %s\r\n", id);
				state = STATE_IDLE; // We are not listening to other than these messages.
			}
		}
		break;
	case STATE_DATA:
		if (c == '*') {
			state = STATE_CHECKSUM1;
		} else {
			checksum ^= c;
			switch (sentence) {
			case GPVTG:
				parseGPVTG(c); // looks ok
				break;
			case GPGGA:
				parseGPGGA(c);
				break;
			case GPRMC:
				parseGPRMC(c); // looks ok
				break;
			case GPGSA:
				parseGPGSA(c);
				break;
			case GPGSV:
				parseGPGSV(c);
				break;
			case GPGLL:
				parseGPGLL(c);
				break;
			}
			dataindex++;
		}
		break;
	case STATE_CHECKSUM1:
		checksum -= char2hexdigit(c) << 4;
		state = STATE_CHECKSUM2;
		break;
	case STATE_CHECKSUM2:
		checksum -= char2hexdigit(c);
		state = STATE_IDLE;
		if (checksum) {
			latestGPSState = INVALID;
			// trace_printf("*** Bad GPS checksum!\n");
		} else {
			latestGPSState = NEWDATA;
			// trace_printf("parse check %d\n", commitCheck);
			return 1;
		}
		break;
	case STATE_READ_BINARY_UBX_SYNC_2:
		if (c == 0x62)
			state = STATE_READ_BINARY_UBX_CLASS_ID;
		else
			state = STATE_IDLE;
		break;
	case STATE_READ_BINARY_UBX_CLASS_ID:
		readClassId = c;
		state = STATE_READ_BINARY_UBX_MSG_ID;
		break;
	case STATE_READ_BINARY_UBX_MSG_ID:
		readMessageId = c;
		state = STATE_READ_BINARY_UBX_MSG_LEN1;
		break;
	case STATE_READ_BINARY_UBX_MSG_LEN1:
		readBodyLength = c;
		state = STATE_READ_BINARY_UBX_MSG_LEN2;
		break;
	case STATE_READ_BINARY_UBX_MSG_LEN2:
		readBodyLength += (c << 8);
		readBodyCnt = 0;
		state = STATE_READ_BINARY_UBX_MSG_BODY;
		break;
	case STATE_READ_BINARY_UBX_MSG_BODY:
		if (readBodyCnt == 0 && readClassId == 5 && readMessageId == 1) {
			ackedClassId = c;
		} else if (readBodyCnt == 1 && readClassId == 5 && readMessageId == 1) {
			ackedMessageId = c;
		}
		readBodyCnt++;
		if (readBodyCnt == readBodyLength)
			state = STATE_READ_BINARY_UBX_MSG_CHECKA;
		break;
	case STATE_READ_BINARY_UBX_MSG_CHECKA:
		// don't bother to check.
		state = STATE_READ_BINARY_UBX_MSG_CHECKB;
		break;
	case STATE_READ_BINARY_UBX_MSG_CHECKB:
		if (ackedClassId == 6 && ackedMessageId == 0x24) {
			trace_printf("NAV5 Settings confirmed.\n");
			navSettingsConfirmed = true;
		}
		state = STATE_IDLE;
		break;
	}
	return 0;
}

extern void onNewGPSData();

void GPS_getData() {
	NVIC_DisableIRQ(USART1_IRQn);
	__DSB();
	__ISB();

	// These are committed by the IRQ handler once messages are complete.
	GPSTime = nmeaTimeInfo_unsafe;
	GPSCourseSpeed = nmeaCRSSPDInfo_unsafe;
	GPSPosition = nmeaPositionInfo_unsafe;
	GPSStatus = nmeaStatusInfo_unsafe;

	onNewGPSData();
	NVIC_EnableIRQ(USART1_IRQn);
}

void GPS_powerOn() {
	// Reset the message sending
	lastSendConfigurationTime = systemTimeMillis;
	currentSendingIndex = 0;
	busySendingMessage = false;
	navSettingsConfirmed = false;
	setupUSART1();
	GPIOA->ODR &= ~ GPIO_Pin_0;
	state = STATE_IDLE;
}

void GPS_stopListening() {
	USART_Cmd(USART1, DISABLE);

	/* Disable USART1 IRQ (on the USART hw) */
	USART_ITConfig(USART1, USART_IT_TXE, DISABLE);
	USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);

	latestGPSState = CONSUMED;
}

void GPS_powerOff() {
	GPIOA->ODR |= GPIO_Pin_0;
}

boolean GPS_isGPSRunning() {
	uint32_t result = GPIOA->ODR & GPIO_Pin_0;
	return !result;
}

void GPS_invalidateDateTime() {
	nmeaTimeInfo_unsafe.time.valid = 0;
	nmeaTimeInfo_unsafe.date.valid = 0;
}

boolean GPS_isDateTimeValid() {
	return nmeaTimeInfo_unsafe.time.valid && nmeaTimeInfo_unsafe.date.valid;
}

void GPS_invalidatePosition() {
	nmeaPositionInfo_unsafe.valid = 'V';
}

boolean GPS_isPositionValid() {
	return nmeaPositionInfo_unsafe.valid == 'A';
}

void GPS_invalidateNumSatellites() {
	nmeaStatusInfo_unsafe.numberOfSatellites = 0;
}

uint8_t GPS_numberOfSatellites() {
	return nmeaStatusInfo_unsafe.numberOfSatellites;
}

void GPS_powerUpInit() {
	// do nothing
}

#endif
