#include <ADC.h>
#include <APRS.h>
#include <diag/Trace.h>
#include <Globals.h>
#include <GPS.h>
#include <IndividualBoard.h>
#include <misc.h>
#include <Physics.h>
#include <PLL.h>
#include <Power.h>
#include "LED.h"
#include <RecordStorage.h>
#include <RTC.h>
#include <stdint.h>
#include <stm32l1xx.h>
#include <stm32l1xx_gpio.h>
#include <stm32l1xx_pwr.h>
#include <stm32l1xx_rcc.h>
#include <stm32l1xx_rtc.h>
#include <SelfCalibration.h>
#include <Setup.h>
#include <StabilizedOscillator.h>
#include <Systick.h>
#include <Types.h>
#include <WSPR.h>

// Some globals.
float temperature;
int8_t simpleTemperature;
uint8_t numRestarts __attribute__((section (".noinit")));
boolean radioOrGPS; // false for radio, true for GPS.
float batteryVoltage;
float solarVoltage;
uint16_t wakeupPeriod;

boolean clockWasSet;

boolean latestAPRSRegions[12]; 	 // 12 is sufficently large for the world map...
boolean latestAPRSCores[12];	 // 12 is sufficently large for the world map...

uint8_t nextWSPRMessageTypeIndex;
uint16_t lastWSPRWindowWaitTime;

CoreZoneStatus_t coreZoneStatus = UNKNOWN_CORE_ZONE;

boolean notFreshlyBooted = false;

const CalibrationRecord_t* currentCalibration = &defaultCalibration;

void diagsWhyReset() {
	int whyReset = (RCC->CSR >> 24);
	RCC_ClearFlag();
	if (whyReset & (1 << 7))
		trace_printf("Reset low power\n");
	if (whyReset & (1 << 6))
		trace_printf("Reset WWD\n");
	if (whyReset & (1 << 5))
		trace_printf("Reset IWD\n");
	if (whyReset & (1 << 4))
		trace_printf("Reset software\n");
	if (whyReset & (1 << 3))
		trace_printf("Reset POR/PDR\n");
	if (whyReset & (1 << 2))
		trace_printf("Reset NRSTpin\n");
	if (whyReset & (1 << 1))
		trace_printf("Reset OBL\n");
}

void initGeneralIOPorts() {
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);

	GPIOA->ODR = GPIO_Pin_0;

	/* GPIOA and B Periph clock were enabled in main */
	GPIOB->ODR = GPIO_Pin_1 | GPIO_Pin_14; // #3V3 enable and HF driver transistor disable.

	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;

	// GPS-power
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	// Botch transistor on PB14 OD high.
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;

	// 0: LED, 1: 3V3 enable, 4,5,6: HF driver enables, 14: ballast
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_4
			| GPIO_Pin_5 | GPIO_Pin_6;

	// GPIOB stuff: Slow
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/* Configure GPS power pin and Si4463 SDN pin */
	// GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	// GPIO_Init(GPIOA, &GPIO_InitStructure);
}

void testPowerLeak() {
	trace_printf("B1h\n");
	GPIOB->ODR |= GPIO_Pin_1;
	timer_sleep(1000);

	trace_printf("B1h\n");
	GPIOB->ODR &= ~GPIO_Pin_1;
	timer_sleep(1000);

	trace_printf("A0h\n");
	GPIOA->ODR |= GPIO_Pin_0;
	timer_sleep(1000);

	trace_printf("A0l\n");
	GPIOA->ODR &= ~GPIO_Pin_0;
	timer_sleep(1000);

	trace_printf("A1h\n");
	GPIOA->ODR |= GPIO_Pin_1;
	timer_sleep(1000);

	trace_printf("A1l\n");
	GPIOA->ODR &= ~GPIO_Pin_1;
	timer_sleep(1000);

	trace_printf("A4h\n");
	GPIOA->ODR |= GPIO_Pin_4;
	timer_sleep(1000);

	trace_printf("A4l\n");
	GPIOA->ODR &= ~GPIO_Pin_4;
	timer_sleep(1000);

	trace_printf("A5h\n");
	GPIOA->ODR |= GPIO_Pin_5;
	timer_sleep(1000);

	trace_printf("A5l\n");
	GPIOA->ODR &= ~GPIO_Pin_5;
	timer_sleep(1000);

	trace_printf("A6h\n");
	GPIOA->ODR |= GPIO_Pin_6;
	timer_sleep(1000);

	trace_printf("A6l\n");
	GPIOA->ODR &= ~GPIO_Pin_6;
	timer_sleep(1000);

	trace_printf("A14h\n");
	GPIOA->ODR |= GPIO_Pin_14;
	timer_sleep(1000);

	trace_printf("A14l\n");
	GPIOA->ODR &= ~GPIO_Pin_14;
	timer_sleep(1000);
}

void performBeforeLoadPowerADC() {
	ADC_measurement_blocking(ADC_MEASUREMENT_POWER_BEFORELOAD);
	batteryVoltage = PHY_batteryBeforeLoadVoltage();
	solarVoltage = PHY_solarVoltage();
}

void performAfterGPSPowerADC() {
	ADC_measurement_blocking(ADC_MEASUREMENT_POWER_AFTERGPS);
}

void performAfterHFPowerADC() {
	ADC_measurement_blocking(ADC_MEASUREMENT_POWER_AFTERHF);
}

void performTemperatureADC() {
	temperature = PHY_internalTemperature();
	if (temperature > 30)
		simpleTemperature = 30;
	else if (temperature < -65)
		simpleTemperature = -65;
	else
		simpleTemperature = (int8_t) temperature;
}

void calibrationCycle() {
	// This may (be warned) kill the GPS.
	// Reason is that the GPS is so noisy on the supply that the PLL runs rough.
	// WUT interrupt will also be disabled if RTC is calibrated (currently disabled).
	calibrate(simpleTemperature);

	// As a side effect, fix the RTC.
	// RTC_setCalibration(currentCalibration->RTCNeededCorrectionPP10M);
}

/*
 * Squeeze all relevant info out of GPS, including:
 * - Time
 * - Position
 * - Calibration if not already having one for current temperature
 * The GPS is left running (if it was ever started)
 */
void GPSCycle() {
	// Just in case. it bothers the GPS pretty bad. And it should never be running now anyway.
	// Done in main anyway. We should either have been shut down normally already, or have been
	// reset. Either way it should have been shut down.
	// PLL_shutdown();

	// timer_sleep(10); // we have a startup from sleep prob. but
	// only when GPS switch is present. This helps?
	GPS_start();
//	trace_printf("GPS started\n");
	uint32_t gpsStartTime = systemTimeMillis;

	if (GPS_waitForTimelock(MAX_GPS_TIMELOCK_TIME * 1000)) {
		clockWasSet = RTC_setRTC(&GPSTime.date, &GPSTime.time);
	}

	GPS_waitForPosition(POSITION_MAX_TIME_S * 1000);
	GPS_waitForPrecisionPosition(REQUIRE_HIGH_PRECISION_MAX_TIME_S * 1000);
	/*
	 if (REQUIRE_HIGH_PRECISION_POSITIONS
	 && batteryVoltage >= BATTERY_FAILSAFE_ALWAYS_SAFE_VOLTAGE) {
	 trace_printf("Waiting for HP position\n");
	 GPS_waitForPrecisionPosition(
	 REQUIRE_HIGH_PRECISION_MAX_TIME_S * 1000);
	 } else {
	 GPS_waitForPosition(POSITION_MAX_TIME_S * 1000);
	 }
	 */

	lastGPSFixTime = (systemTimeMillis - gpsStartTime + 500) / 1000;

	// Update APRS map
	APRS_frequenciesFromPosition(&lastNonzeroPosition, latestAPRSRegions,
			latestAPRSCores);

	// This will trigger a new calibration if necessary.
	calibrationCycle();

	performAfterGPSPowerADC();

	// end of it. Else the PLL can't run because of supply noise from GPS :(
	GPS_shutdown();
}

void doWSPR() {
	currentCalibration = getCalibration(simpleTemperature);

	PLL_Setting_t pllSetting;

	double maxError = 10E-6;
	// boolean foundSetting = false;

	while (maxError < 100E-6) {
		if (PLL_bestPLLSetting(
				currentCalibration->transmitterOscillatorFrequencyAtDefaultTrim,
				WSPR_FREQUENCIES[THIRTY_M], maxError, &pllSetting)) {

			trace_printf("Using fOsc=%d, N=%d, M=%d, trim=%d error=%d\n",
					currentCalibration->transmitterOscillatorFrequencyAtDefaultTrim,
					pllSetting.N, pllSetting.M, pllSetting.trim,
					(int) (maxError * 10E6));

			// foundSetting = true;
			break;
		} else {
			trace_printf(
					"NO feasible PLL setting in range! Should really not happen. Anyway, we try again with more tolerance.\n");
			maxError += 25E-6;
		}
	}

	uint8_t nextWSPRMessageType;

	if (lastNonzero3DPosition.alt < LOWALT_THRESHOLD) {
		if (nextWSPRMessageTypeIndex >= WSPR_LOWALT_SCHEDULE_LENGTH) {
			nextWSPRMessageTypeIndex = 0;
		}
		nextWSPRMessageType = WSPR_LOWALT_SCHEDULE[nextWSPRMessageTypeIndex];
	} else {
		if (nextWSPRMessageTypeIndex >= WSPR_SCHEDULE_LENGTH) {
			nextWSPRMessageTypeIndex = 0;
		}
		nextWSPRMessageType = WSPR_SCHEDULE[nextWSPRMessageTypeIndex];
	}

	prepareWSPRMessage(nextWSPRMessageType, batteryVoltage);
	nextWSPRMessageTypeIndex++;

	PWR_startDevice(E_DEVICE_HF_TX);
	WSPR_Transmit(THIRTY_M, &pllSetting, WSPR_DEVIATION_DAC_30m);
	PWR_stopDevice(E_DEVICE_HF_TX);

	// Recurse, do it again if we sent the rather uninformative TYPE1.
	// if (nextWSPRMessageType == TYPE1) {
	// 	doWSPR();
	// }
}

void HF_APRSCycle() {
	if (!PWR_isSafeToUseDevice(E_DEVICE_HF_TX)) {
		trace_printf("Unsafe to HF tx\n");
		return;
	}

	APRS_transmitMessage(HF, COMPRESSED_POSITION_MESSAGE, 10149300,
			currentCalibration->transmitterOscillatorFrequencyAtDefaultTrim);
}

void VHF_APRSCycle() {
	CoreZoneStatus_t newCoreZoneStatus = OUTSIDE_CORE_ZONE;

	// Send position on all relevant frequencies, without a break in between.
	for (uint8_t i = 0;
			i < APRS_WORLD_MAP_LENGTH && PWR_isSafeToUseDevice(E_DEVICE_VHF_TX);
			i++) {
		if (latestAPRSRegions[i]) {
			trace_printf("Sending a COMPRESSED_POSITION_MESSAGE on %d\n",
					APRS_WORLD_MAP[i].frequency * 1000);
			APRS_transmitMessage(VHF, COMPRESSED_POSITION_MESSAGE,
					APRS_WORLD_MAP[i].frequency * 1000,
					PLL_XTAL_DEFAULT_FREQUENCY);
		}

		if (latestAPRSCores[i])
			newCoreZoneStatus = IN_CORE_ZONE;
	}

// Send status on all relevant frequencies, without a break in between but with a break before.
// If we are not in network coverage, don't bother to try or to delay first.
	timer_sleep(2000);
	for (uint8_t i = 0; i < APRS_WORLD_MAP_LENGTH; i++) {
		if (latestAPRSRegions[i]) {
			trace_printf("Sending a STATUS_MESSAGE on %d\n",
					APRS_WORLD_MAP[i].frequency * 1000);
			APRS_transmitMessage(VHF, STATUS_MESSAGE,
					APRS_WORLD_MAP[i].frequency * 1000,
					PLL_XTAL_DEFAULT_FREQUENCY);

		}
	}

	if (newCoreZoneStatus == IN_CORE_ZONE) {
		timer_sleep(2000);

		// Get rid of some storage backlog, first take that scheduled for a 2nd transmission
		int maxStored = 5;

		StoredPathRecord_t* storedRecord;
		while (hasRecordOutForLastTransmission() && maxStored-- > 0) {
			trace_printf("Dumping a record 2nd time\n");
			timer_sleep(1000);
			storedRecord = nextRecordOutForLastTransmission();
			for (uint8_t i = 0; i < APRS_WORLD_MAP_LENGTH; i++) {
				if (latestAPRSCores[i])
					APRS_transmitStoredMessage(VHF, storedRecord,
							APRS_WORLD_MAP[i].frequency * 1000,
							PLL_XTAL_DEFAULT_FREQUENCY);
			}
		}

		maxStored = 5;

		// Get rid of some storage backlog, 1st transmission
		while (hasRecordOutForFirstTransmission() && maxStored-- > 0) {
			trace_printf("Dumping a record 1st time\n");
			timer_sleep(1000);
			storedRecord = nextRecordOutForFirstTransmission();
			for (uint8_t i = 0; i < APRS_WORLD_MAP_LENGTH; i++) {
				if (latestAPRSCores[i])
					APRS_transmitStoredMessage(VHF, storedRecord,
							APRS_WORLD_MAP[i].frequency * 1000,
							PLL_XTAL_DEFAULT_FREQUENCY);
			}
		}
	} else if (lastNonzeroPosition.valid == 'A') { // Not core. Either we are outside zones or the transmitter can't be run.
		trace_printf("Outside APRS core or VHF unsafe to run\n");
		if (newCoreZoneStatus != coreZoneStatus) {
			// we have just left core, since last transmit. Or, we never were in core.
			resetRecordStorageTimer();
		}

		// Store a message.
		if (timeToStoreRecord()) {
			trace_printf("Storing a record\n");
			StoredPathRecord_t* recordToStore = nextRecordIn();
			storeToRecord(recordToStore);
		} else {
			trace_printf("Not storing now\n");
		}
	} else { // no gps fix at all... just shoot at random.
		APRS_transmitMessage(VHF, STATUS_MESSAGE, 144800 * 1000,
		PLL_XTAL_DEFAULT_FREQUENCY);
	}

	coreZoneStatus = newCoreZoneStatus;
}

void HFRadioCycle() {
	HF_APRSCycle();
	doWSPR();
	performAfterHFPowerADC();
}

extern int SetSysClock(void);

void reschedule() {
	double simulatedVoltage = batteryVoltage + solarVoltage; // just to give it a boost when there is sunlight.
	// This polynomial has the values
	// 3600 for simulatedVoltage = 3.3
	// 300 for simulatedVoltage = 4.2
	// 100 for simulatedVoltage = 4.8
	int32_t time;
	/*
	 if (simulatedVoltage <= 4.2) {
	 time = (int32_t) (2222.222 * simulatedVoltage * simulatedVoltage
	 - 20333.333 * simulatedVoltage + 46500);
	 } else {
	 // 2..10 min.
	 time = 600 - (600 - 120) / (4.8 - 4.2) * (simulatedVoltage - 4.2);
	 }
	 */
	time = (int32_t) (2168.98 / (simulatedVoltage - 2.82632) - 978.947);
	trace_printf("Batt:%u, Sol: %u, mV:%u->s:%d\n",
			(int) (batteryVoltage * 1000), (int) (solarVoltage * 1000),
			(int) (simulatedVoltage * 1000), time);
	if (time < 120)
		time = 120;
	else if (time > 3600)
		time = 3600;
	RTC_setWakeup(time);
	wakeupPeriod = time;
}

void wakeupCycle() {
	systick_start(); // TODO - check if this can be moved to inside below if.

	ADC_ensureVDDMeasured();
	performBeforeLoadPowerADC();

	trace_printf("Batt %d mV\n", (int) (1000 * batteryVoltage));
	trace_printf("Solar %d mV\n", (int) (1000 * solarVoltage));

	// Are we alive at all?
	if (batteryVoltage >= ABSOLUTE_MIN_VBATT) {
		if (PWR_isSafeToUseDevice(E_DEVICE_VHF_TX)) {
			// start the real HSE clock.
			if (!SetSysClock()) {
				trace_printf("SetSysClock() failed\n");
			}

			// And fire up periphs
			RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
			RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);

			if (PWR_isSafeToUseDevice(E_DEVICE_GPS)) {
				performTemperatureADC();

				// do some radio work :)
				if (radioOrGPS && clockWasSet) {
					trace_printf("HF\n");
					VHF_APRSCycle();
					HFRadioCycle();
				} else {
					trace_printf("GPS\n");
					GPSCycle();
					VHF_APRSCycle();
				}
				radioOrGPS = !radioOrGPS;
			}

			RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, DISABLE);
			RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, DISABLE);
		} else {
			trace_printf("Not enough volt to do anything interesting\n");
		}
	} else { // we are below lowest safe battery voltage
		trace_printf("Battery too low for main-stuff\n");
		GPIOB->ODR &= ~GPIO_Pin_1; // cut off even backup power to GPS :(
	}

	systick_end();
}

void HFGroundCalibration() {
	//performUnloadedPowerADC();

	if (!SetSysClock()) {
		trace_printf("SetSysClock() failed\n");
	}

	systick_start();

	selfCalibrateForWSPR(16E6);
	printTrimmingCalibrationTable();

	while (1) {
		GPSCycle();
		VHF_APRSCycle();
		doWSPR();
	}
}

void PLLTest() {
	while (1) {
		//eepromExperiment(0);
		trace_printf("144700\n");
		APRS_makeDirectTransmissionFrequency(144700000,
		PLL_XTAL_DEFAULT_FREQUENCY,
		DIRECT_2m_HARDWARE_OUTPUT);
		timer_sleep(1000);
		PLL_shutdown();
		timer_sleep(1000);
	}
}

int main() {
	initGeneralIOPorts();

	// SetSysClock();
	/* Configure one bit for preemption priority */
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
	/* check that CPU is alive at all, just in case it seems not. */

	// Get rid of CDCEL913's default startup active.
	PLL_shutdown();

	if (++numRestarts > 99)
		numRestarts = 0;

	// for debug only.
#if defined(TRACE)
	systick_start();

	if (!SetSysClock()) {
//		trace_printf("SetSysClock() failed\n");

		//while (1) {
			LED_PORT->ODR |= LED_PORTBIT;
			timer_sleep(100);
			LED_PORT->ODR &= ~LED_PORTBIT;
			timer_sleep(100);
		//}
	}

	timer_sleep(3000);

	trace_printf("start.\n");
#endif

	RTC_init();

#if defined (TRACE) && MODE == GROUNDTEST
	// PLLTest();
	HFGroundCalibration();
#endif

	// Wake up every day at same time, just in case the interval timer failed.
	RTC_scheduleDailyEvent();

	// RTC_setWakeup(RTC_WAKEUP_PERIOD_S);
	wakeupPeriod = SIMPLE_BROWNOUT_SECONDS;
	RTC_setWakeup(SIMPLE_BROWNOUT_SECONDS);

	if (DEFEAT_VOLTAGE_CHECKS) {
		invalidateStartupLogs();
	}

	while (true != false) {
		// Prepare sleep. Clear any used-up RTC event out's.
		// Really, both of these 2 steps were found to be needed. No monkeying.
		PWR_RTCAccessCmd(ENABLE);
		RTC_ClearITPendingBit(RTC_IT_WUT | RTC_IT_ALRA);
		PWR_RTCAccessCmd(DISABLE);

		trace_printf("Sleeping...\n");
		// RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, DISABLE);
		// RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, DISABLE);
		// PWR_UltraLowPowerCmd()
		// RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);
		// PWR_VoltageScalingConfig(PWR_VoltageScaling_Range2);
		PWR_EnterSTOPMode(PWR_Regulator_LowPower, PWR_STOPEntry_WFE);

		RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, DISABLE);
		LED_PORT->ODR |= LED_PORTBIT;

		wakeupCycle();
		/*
		 if (!notFreshlyBooted) {
		 notFreshlyBooted = true;
		 trace_printf("booted\n");
		 RTC_setWakeup(SIMPLE_BROWNOUT_SECONDS);
		 wakeupPeriod = SIMPLE_BROWNOUT_SECONDS;
		 } else {
		 trace_printf("cycled\n");
		 wakeupCycle();
		 }
		 */
	}
}
