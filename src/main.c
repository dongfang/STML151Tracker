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
#include <LED.h>

// Some globals.
float temperature;
int8_t simpleTemperature;
uint8_t numRestarts __attribute__((section (".noinit")));
float batteryVoltage;
float solarVoltage;

char scheduleName = '-';
uint16_t mainPeriodWakeupCycles;
uint16_t mainPeriodCounter;
uint16_t wsprCounter;
uint16_t hfPacketCounter;

boolean latestAPRSRegions[12]; 	 // 12 is sufficently large for the world map...
boolean latestAPRSCores[12];	 // 12 is sufficently large for the world map...

uint8_t nextWSPRMessageTypeIndex;
uint16_t lastWSPRWindowWaitTime;

CoreZoneStatus_t coreZoneStatus = UNKNOWN_CORE_ZONE;

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

	GPIO_InitTypeDef GPIO_InitStructure;

	/* GPIOA and B Periph clock were enabled in main */

	GPIOB->ODR = GPIO_Pin_1; // LED off, disarm WSPR, disable Si4463 SS.

	// 0: LED, 1: 3V3 enable, 4,5,6: HF driver enables, 14: ballast
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_4
			| GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_14;

	// GPIOB stuff: Slow
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/* Configure GPS power pin and Si4463 SDN pin */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIOA->ODR = GPIO_Pin_0;	 // | GPIO_Pin_8;
}

void performUnloadedPowerADC() {
	ADC_measurement_blocking(ADC_MEASUREMENT_POWER_UNLOADED);
	batteryVoltage = PHY_batteryUnloadedVoltage();
	solarVoltage = PHY_solarVoltage();
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
	currentCalibration = getCalibration(simpleTemperature, true);

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

	if (PWR_isSafeToUseDevice(E_DEVICE_GPS)) {
		GPS_start();
		uint32_t gpsStartTime = systemTimeMillis;

		if (GPS_waitForTimelock(MAX_GPS_TIMELOCK_TIME * 1000)) {
			boolean success = RTC_setRTC(&GPSTime.date, &GPSTime.time);
		}

		if (REQUIRE_HIGH_PRECISION_POSITIONS && batteryVoltage >= 3.3) {
			trace_printf("Waiting for HP position\n");
			GPS_waitForPrecisionPosition(
			REQUIRE_HIGH_PRECISION_MAX_TIME_S * 1000);
		} else {
			GPS_waitForPosition(POSITION_MAX_TIME_S * 1000);
		}

		lastGPSFixTime = (systemTimeMillis - gpsStartTime + 500) / 1000;

		// Update APRS map
		APRS_frequenciesFromPosition(&lastNonzeroPosition, latestAPRSRegions,
				latestAPRSCores);

		// This will trigger a new calibration if necessary.
		calibrationCycle();

		// end of it. Else the PLL can't run because of supply noise from GPS :(
		GPS_shutdown();
	}
}

void doWSPR() {
	if (!PWR_isSafeToUseDevice(E_DEVICE_HF_TX))
		return;

	currentCalibration = getCalibration(simpleTemperature, false);

	PLL_Setting_t pllSetting;

	double maxError = 10E-6;
	while (maxError < 100E-6) {
		if (PLL_bestPLLSetting(
				currentCalibration->transmitterOscillatorFrequencyAtDefaultTrim,
				WSPR_FREQUENCIES[THIRTY_M], maxError, &pllSetting)) {

			trace_printf("Using fOsc=%d, N=%d, M=%d, trim=%d\n",
					currentCalibration->transmitterOscillatorFrequencyAtDefaultTrim,
					pllSetting.N, pllSetting.M, pllSetting.trim);

			trace_printf("Waiting for WSPR window\n");
			lastWSPRWindowWaitTime = RTC_waitTillModuloMinutes(2, 0);

			uint8_t nextWSPRMessageType;

			if (lastNonzero3DPosition.alt < LOWALT_THRESHOLD) {
				if (nextWSPRMessageTypeIndex >= WSPR_LOWALT_SCHEDULE_LENGTH) {
					nextWSPRMessageTypeIndex = 0;
				}
				nextWSPRMessageType =
						WSPR_LOWALT_SCHEDULE[nextWSPRMessageTypeIndex];
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

			break;
		} else {
			trace_printf(
					"NO feasible PLL setting in range! Should really not happen. Anyway, we try again with more tolerance.\n");
			maxError += 25E-6;
		}
	}
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
	if (PWR_isSafeToUseDevice(E_DEVICE_VHF_TX)) {
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
	} else { // Not core. Either we are outside zones or the transmitter can't be run.
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
	}

	coreZoneStatus = newCoreZoneStatus;
}

void radioCycle() {
	// Just do VHF in any case.
	VHF_APRSCycle();

	hfPacketCounter++;
	if (hfPacketCounter >= HF_PACKET_DIVIDER) {
		hfPacketCounter = 0;
		HF_APRSCycle();
	}

	wsprCounter++;
	trace_printf("WSPRCounter %d\n", wsprCounter);
	if (wsprCounter >= WSPR_DIVIDER) {
		wsprCounter = 0;
		doWSPR();
	}
}

void reschedule(char _scheduleName, uint8_t _mainPeriodWakeupCycles) {
	if (_scheduleName != scheduleName) {
		trace_printf("Rescheduling : %c\n", _scheduleName);

		// If changing to a faster schedule, cut short the delay.
		if (_mainPeriodWakeupCycles < mainPeriodCounter) {
			mainPeriodCounter = _mainPeriodWakeupCycles;
		}
	}
	scheduleName = _scheduleName;
	mainPeriodWakeupCycles = _mainPeriodWakeupCycles;
}

extern int SetSysClock(void);

void wakeupCycle() {
	systick_start(); // TODO - check if this can be moved to inside below if.

	ADC_ensureVDDMeasured();
	performUnloadedPowerADC();

//	trace_printf("ADCValue0 (Ch7, PA7, Batt) %d\t", ADCUnloadedPowerValues[0]);
//	trace_printf("ADCValue1 (Ch6, PA6, Solar) %d\t", ADCUnloadedPowerValues[1]);
	trace_printf("Batt %d mV\n", (int) (1000 * batteryVoltage));
	trace_printf("Solar %d mV\n", (int) (1000 * solarVoltage));

	// Are we alive at all?
	if (batteryVoltage >= ABSOLUTE_MIN_VBATT) {

		// TODO maybe this needs not be done soooooo often.
		performTemperatureADC();
		trace_printf("Temp C: %d\n", (int) temperature);

		// Are we in a very good shape?
		if (batteryVoltage >= BATTERY_FAILSAFE_ALWAYS_SAFE_VOLTAGE
				&& solarVoltage >= BATTERY_FAILSAFE_ALWAYS_SAFE_SOLAR_VOLTAGE) {
			if (lastNonzero3DPosition.alt != 0 &&
					lastNonzero3DPosition.alt < LOWALT_THRESHOLD) {
				reschedule(LOWALT_SCHEDULE_TIME);
			} else {
				reschedule(DAY_SCHEDULE_TIME);
			}
		} else if (batteryVoltage >= BATTERY_FAILSAFE_ALWAYS_SAFE_VOLTAGE) {
			// battery was okay, so it has to be the solar insufficient...
			reschedule(NIGHT_SCHEDULE_TIME);
		} else {
			reschedule(LOWBATT_SCHEDULE_TIME);
		}

		// Time to do main cycle? (the variables should have been set somewhere above, all of them)
		trace_printf("To go until main %d\n", mainPeriodCounter);

		if (mainPeriodCounter == 0) {
			mainPeriodCounter = mainPeriodWakeupCycles;

			// start the real HSE clock.
			if (!SetSysClock()) {
				trace_printf("SetSysClock() failed\n");
			}

			// And fire up periphs
			RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
			RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);

			// do some radio work :)
			GPSCycle();
			radioCycle();
		} else {
			mainPeriodCounter--;
		}
	} else { // we are below 3.0
		trace_printf("Battery too low for main-stuff (<3)\n");
		reschedule(CRISIS_SCHEDULE_TIME);
	}
	systick_end();
}

void HFGroundCalibration() {
#if defined (TRACE) && MODE == GROUNDTEST
	//performUnloadedPowerADC();

// just to be sure we didn't enable this for a flight build.
	if (!SetSysClock()) {
		trace_printf("SetSysClock() failed\n");
	}
	systick_start();
	selfCalibrateForWSPR(16E6);
	printTrimmingCalibrationTable();
	while (1)
	doWSPR();
#endif
}

void PLLTest() {
	while (1) {
		APRS_makeDirectTransmissionFrequency(144700000,
//		APRS_makeDirectTransmissionFrequency(10000000,
				PLL_XTAL_DEFAULT_FREQUENCY,
				DIRECT_2m_HARDWARE_OUTPUT);
		timer_sleep(1000);
		//PLL_shutdown();
		//timer_sleep(1000);
	}
}

int main() {
	// SetSysClock();
	/* Configure one bit for preemption priority */
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
	initGeneralIOPorts();
	/* check that CPU is alive at all, just in case it seems not.
	 while(true) {
	 volatile int i;
	 LED_PORT->ODR |= LED_PORTBIT;
	 for (i=0; i<100000; i++);
	 LED_PORT->ODR &= ~LED_PORTBIT;
	 for (i=0; i<100000; i++);
	 trace_printf("hi there!");
	 }
	 */

#if defined(SIMPLE_BROWNOUT_MODE)
	mainPeriodCounter = SIMPLE_BROWNOUT_PERIODS;
#endif

	if (++numRestarts > 99)
		numRestarts = 0;

	// for debug only.
#if defined(TRACE)
	if (!SetSysClock()) {
		trace_printf("SetSysClock() failed\n");
	}
	systick_start();
	timer_sleep(3000);
	trace_printf("start\n");
#endif

	RTC_init();

	// PLLTest();
	// HFGroundCalibration();

	// Wake up every day at same time, just in case the interval timer failed.
	RTC_scheduleDailyEvent();
	RTC_setWakeup(RTC_WAKEUP_PERIOD_S);

	if (DEFEAT_VOLTAGE_CHECKS) {
		invalidateStartupLogs();
	}

	// TODO something in PLL_shutdown() depends on systick running (or else gets stuck), what is it?
	// Something with timer_sleep() or similar.
	systick_start();

	// Get rid of CDCEL913's default startup active.
	PLL_shutdown();

	while (1) {
		wakeupCycle();

		// Prepare sleep. Clear any used-up RTC event out's.
		// Really, both of these 2 steps were found to be needed. No monkeying.
		PWR_RTCAccessCmd(ENABLE);
		RTC_ClearITPendingBit(RTC_IT_WUT | RTC_IT_ALRA);
		PWR_RTCAccessCmd(DISABLE);

		trace_printf("Sleeping...\n");
		PWR_EnterSTOPMode(PWR_Regulator_ON, PWR_STOPEntry_WFE);
	}
}
