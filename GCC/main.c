/*This file has been prepared for Doxygen automatic documentation generation.*/
/*! \file *********************************************************************
 *
 * \brief
 *     Application for Smart Battery with ATmega32HVB (two, thre or four cell Li-Ion)
 *     SBS commands management with SMbus.
 *
 * \par Application note:
 *      AVR474: SB202 Firmware user's guide.
 *
 * \par Documentation
 *      For comprehensive code documentation, supported compilers, compiler
 *      settings and supported devices see readme.html
 *
 * \author
 *      Atmel Corporation: http://www.atmel.com \n
 *      Support email: avr@atmel.com
 *
 * $Name$
 * $Revision: 6090 $
 * $Date: 2010-08-03 21:40:12 +0800 (Tue, 03 Aug. 2010) $  \n
 *
 * Copyright (c) 2010, Atmel Corporation All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. The name of ATMEL may not be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE EXPRESSLY AND
 * SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "common.h"
#include "ATmega32HVB_signature.h"
#include "bandgap.h"
#include "vadc.h"
#include "ccadc.h"
#include "smbus.h"
#include "authentication.h"
#include "battery_protection.h"
#include "battery_pack_parameters.h"
#include "battery_voltage_monitoring.h"
#include "battery_current_monitoring.h"
#include "fet_control.h"
#include "watchdog.h"
#include "rc_calibration.h"
#include "pwr_mgmnt.h"
#include "ntc_rh163h103f.h"
#include "rtc.h"
#include "cc_gas_gauging.h"
#include "gas_gauging.h"
#include "vreg_mon.h"
#include "voltage_based_SoC.h"
#include "timer.h"
#include <avr/io.h>

/******************************************************************************
 File scope variables
*******************************************************************************/
//! The internal chip temperature in 0.1 Kelvin
static uint16_t coreTemperature = DEGREES_C_TO_K( 70 dC);

//! State of Charge
static uint8_t SoC_State = 50;

#if CELLTEMPERATURE_INPUTS != 0
//! Array for storing NTC (thermistor) temperatures
static uint16_t cellTemperature[CELLTEMPERATURE_INPUTS];
#endif

//! Variable for storing the latest calculated slow RC period
static volatile uint16_t slowRcOscillatorPeriod = SLOW_RC_NOMINAL_PERIOD;

//! Struct that contains incoming messages from the communication
SBS_command_t sbs;
SBS_command_t *sbsReply;

//! Error flags
struct {
	uint8_t criticalConditionDetected : 1 ;
	uint8_t cellTemperatureTooLow : 1;
	uint8_t cellTemperatureTooHigh : 1;
	uint8_t voltageTooHigh : 1;
	uint8_t voltageTooLow : 1;
	uint8_t reoccuringChargeProtection : 1;
	uint8_t checksumFailure : 1;
	
} errorFlags = {0,0,0,0,0,0,0};

//! State flags
struct {
	uint8_t systemIsInStandby : 1; //!< Set when current is lower than defined in battParams.activeCurrentThreshold
	uint8_t inDUVR : 1;	//!< Set at startup and cleared when DUVR mode is disabled
	uint8_t chargingProhibited : 1; //!< Charging is prohibited until a discharge current is detected
	uint8_t dischargingProhibited : 1; //!< Discharging is prohibited until a charge current is detected
	uint8_t simulatePowerOff : 1; //!< Gets set when trying to power-off, but it's disabled (can be used when debugging)
	uint8_t remainingCapacityInaccurate : 1; //!< Set to 1 if remainingCapacity is based on an estimated SoC from the voltage based SoC
	
	//! If cleared: Gets set when a poweroff condition occurs.
	//! If set: If a new poweroff condition occurs with this flag set, the device will enter poweroff.
	//!         If not a poweroff is detected on a cycle, this flag will be cleared
	//! Is used to avoid entering poweroff on short-circuit and short discharge bursts
	uint8_t poweroffCondition : 1;
} stateFlags = {0,0,0,0,0,0,0};


struct {
	//! Set when the remaining capacity is lower than the set alarm value in battParams.remainingCapacityAlarm (updated once every minute)
	uint8_t remainingCapacityAlarm : 1;
	//! Set when the remaining time at the average current is lower then the alarm value in battParams.remainingTimeAlarm (updated once every minute)
	uint8_t remainingTimeAlarm : 1;
	//! Set when discharge should be terminated as soon as possible. Cleared when a charge current is detected
	uint8_t terminateDischarge : 1;
	//! Set when fully charged and cleared when a discharge current is noticed
	uint8_t fullyCharged : 1;
	//! Set when remaining capacity is at or below 0 and cleared when SoC is above 20%
	//! Also set then the discharge FET is closed because of low voltage.
	uint8_t fullyDischarged : 1;
	
	//! Force charge FET disabled
	uint8_t forceChargeFETDisabled : 1;
	//! Force discharge FET disabled
	uint8_t forceDischargeFETDisabled : 1;
} sbsFlags = {0,0,0,0,0,0,0};


//! Task flags
struct {
	//! A new CCADC ACC result is ready to be processed
	uint8_t newCurrentAvailable : 1;
	//! Temperature has changed so much that a recalibration of the fast RC is needed if this is set
	uint8_t rcCalibrationRequired : 1;
	//! The communication interface has received a complete message if this is set
	uint8_t newSbsCommandAvailable : 1;
	//! The communication interface has received an authentication command if this is set
	uint8_t doAuthentication : 1;
} taskFlags = {0,0,0,0};

/******************************************************************************
 Prototypes for private functions
******************************************************************************/
err_t Reset_Initialization();

void runEverySecond();
void runEveryMinute();

static void handleNewCoreTemperature();
static void handleNewCellTemperature();
static int8_t checkBatteryTemperature(int16_t temperature, bool discharge);
static void handleNewCellVoltage(vadcIndex_t TempCellVoltage);
static void handleNewCCADCResult();
static void configureVADC();

static void disableDUVRIfPossible();
static bool anyTemperatureError();

static void enterPoweroff();

static void handleSBSCommand();


/******************************************************************************/
/*! \brief The main function.
 *
 * This contains initialization and the main loop as described in the
 * application note. Interrupt service routines (ISR) supply all data to be
 * processed in the main loop.
 *
 */
int main( void )
{
	/*
	Initialisation of peripheral modules:
	- Watchdog (timeout must be more than longer than the longest conversion period used for the CC-ADC)
	- Turn of unneeded modules (only SPI atm)
	- Bandgap
	- Set Current protection levels/configuration
	- V-ADC
		- Get cell voltages (requires that interrupts are enabled)
		- Get temperature
		- Estimate capacity ("gas gauging")
	- Check battery voltage and release the device from DUVR mode
	- CC-ADC
		- Get initial current reading (momentary and average).
	- Communication
	*/
	// Initialization.
	wdt_reset();
	
	WDT_SetTimeOut( WDTO_2Kms );  //Set Watchdog timeout to 2 sec

#ifdef DEBUG
	if( FAILURE == Reset_Initialization() ){
		// TODO: The Watchdog has triggered several times - might want to shut down the battery?!
		// POWMAN_Shutdown();
	}
#endif
	// Enable power reduction for SPI
	PRR_DISABLE_SPI();
	// Disable digital input for port a
	DIDR0 = (1<<PA0DID) | (1<<PA1DID);
	// Enable pull-ups for port b to avoid floating pins
	PORTB = (1<<PORTB0) | (1<<PORTB1) | (1<<PORTB2) | (1<<PORTB3) | (1<<PORTB4) | (1<<PORTB5) | (1<<PORTB6) | (1<<PORTB7);
	DDRB = (1<<PORTB0) | (1<<PORTB1) | (1<<PORTB2) | (1<<PORTB3) | (1<<PORTB4);
	
	// Check the CRC checksum of the battery parameters struct
	errorFlags.checksumFailure = (! BATTPARAM_CheckCRC()) | (! CheckSignatureCRC());

	// Initialize Bandgap reference.
	if( FAILURE == BANDGAP_Initialization() ){
		errorFlags.criticalConditionDetected = true;
	}
	
	// Initialize the battery protection module (enabled by default - if default settings are good
	// this is not required).
	BATTPROT_SetShortCircuitDetection( battParams.shortcircuitCurrent, battParams.shortcircuitReactionTime );
	BATTPROT_SetOverCurrentDetection( battParams.overcurrentDischarge, battParams.overcurrentCharge, battParams.overcurrentReactionTime );
	BATTPROT_SetHighCurrentDetection( battParams.highcurrentDischarge, battParams.highcurrentCharge, battParams.highcurrentReactionTime );
	
	BATTPROT_DisableDischargeHighCurrentProtection(); // Don't want a high discharging current to disable the FETs (in this example)
	BATTPROT_EnableAllInterrupts();

	//Enable DUVR from the start, so we are always in DUVR mode.
	FETCTRL_EnableDeepUnderVoltageMode();
	
	// On reset, the device is in DUVR mode depending on the fuse setting DUVRINIT
	stateFlags.inDUVR = true;
	
	// Enable the voltage regulator monitor interrupt
	VREGMON_InterruptEnable();
	
	// Initialize V-ADC and configure a full scan
	VADC_InitializeVadcCoefficients();
	VADC_ScanConfig( (vadcIndex_t)HIGHEST_CELL, (vadcIndex_t)HIGHEST_VADC, (vadcIndex_t)VTEMP );

	// This can only fail if the VADC is set-up with an invalid ScanConfig
	if( FAILURE == VADC_StartScan() ){
		errorFlags.criticalConditionDetected = true;
	} else {
		__enable_interrupt();
		while( VADC_RUNNING == VADC_ScanState() );
		__disable_interrupt();
		
		VADC_ClearReadyFlags();
		
		disableDUVRIfPossible();
		VBSoC_Init();
	}
	
	// Calculate the coefficient numbers for ccadc_ticks to mA and gas_gauging to mAh functions
	BATTCUR_CalculateShuntCoeffient(battParams.shuntResistance, CCADC_VOLTREF);
        uint16_t temp = RCCAL_CalculateUlpRCperiod(coreTemperature/10);
	CCGASG_CalculateShuntCoefficients(temp, battParams.shuntResistance, CCADC_VOLTREF);
	
	// Calculate values for the sram battery parameters struct
	BATTPARAM_InitSramParameters();
	
	// Copy key from eeprom to sram if using AES
#if defined(AUTH_USE_AES)
	AUTH_CopyKeyToSram();
#endif
		
	
	// If a critical condition occured, (either bandgap didn't work or VADC was configured wrong)
	// don't do any of the battery related initialization and disable DUVR mode (so charging is impossible)
	if( errorFlags.criticalConditionDetected ) {
		FETCTRL_DisableDeepUnderVoltageMode();
		FETCTRL_DisableFets();
		
	} else {
		// Initialize the CC-ADC (sample every second and set regular current level)
		CCADC_Init( ACCT_1024, RCCI_1024, 10 mA );
		CCADC_SetMode( CCADC_ACC );
		
		// Set initial remaining capacity in the CCGASG module, it is marked as in-
		// accurate and updated the first time the VBSoC have an accurate reading.
		CCGASG_SetStateofCharge(VBSoC_EstimatedSoC());
		stateFlags.remainingCapacityInaccurate = 1;
		
		__enable_interrupt();
		// Needs to run a conversion before the value is correct
		while( !CCADC_isAccResultReady() );
		wdt_reset();
		__disable_interrupt();
		
		
		// Calculate the CCADC Accumulating conversion offset if it is invalid and
		// DUVR mode is disabled
		if(CCADC_GetRawAccOffset() == 32767 && ! stateFlags.inDUVR) {
			FETCTRL_DisableFets(); // Disable FETs when measuring offset as changes in current not is good
			__enable_interrupt();
			CCADC_CalculateAccOffset();
			__disable_interrupt();
		}
		
		// If two or more cells, init the misbalance corrector
#if BATTPARAM_CELLS_IN_SERIES >= 2
		BATTVMON_InitializeBalancing();
#endif
		
		//Store information about the current (to be able to reply if host asks...)
		BATTCUR_StoreCurrent( CCADC_GetAccResult() );
		BATTCUR_InitializeAverageCurrent( BATTCUR_GetOffsetCalibratedCurrent() );
	}
	
	//init communication bus
	T1init();	// Used for SmBus timeout and LED

	Comm_Init();

	// Ready to run - enable interrupts and enter main loop.
	__enable_interrupt();
	
	
	/*******************************************************************
	Main loop:
	If any of the task flags are set the corresponding task should be run.
	
	The following things are done:
	- Reset watchdog
	- If a new CCADC result is ready, start a VADC conversion
	- If the communication interface have a new byte, handle it
	- and if a whole new command has arrived, handle that
	- Handle new CCADC result if it's ready
	- Check new core temperature if ready, and run a fast RC calibration if needed
	- Check new cell temperature if ready
	- Check new cell voltage if ready
	- Set different sleep modes depending on what's running
	  - If in communication or RC calibration (any timer running) can only enter idle mode
	  - If VADC is running, can't go deeper than ADC noise reduction mode
	  - If "nothing" is running, enter power-save
	- Disable/enable fets depeding on various things
	- Go to sleep
	*/
	while(1) {

		//Tickle the watchdog every cycle
		wdt_reset();
		
		if( CCADC_isAccResultReady() )
		{
			// Start a new VADC scan as fast as possible so that all scans hopefully are
			// ready before the main loop ends, to avoid having to cycle again.
			
			
#if BATTPARAM_CELLS_IN_SERIES >= 2
			// Disable cell balancing if more than one cell
			BATTVMON_DisableCellBalancing();
#endif
			
			// Start a new VADC scan
			configureVADC();
			VADC_StartScan();
			
			taskFlags.newCurrentAvailable = true;
		}
		
		//See if there were any received commands.
		taskFlags.newSbsCommandAvailable = Comm_Handle( &sbs );

		
		//If complete sbs command has been received: process it
		if( taskFlags.newSbsCommandAvailable )
		{
			handleSBSCommand();
			// If authentication is needed, run. It will use and change the values in sbs,
			// make sure the communication interrupt does not write directly to it.
			// Note that this can take long time and communication will not work
			// during that time.
			if(taskFlags.doAuthentication) {
				AUTH_Execute(&sbs);
				taskFlags.doAuthentication = false;
			}
		}
		
		
		/*
		A new CC-ADC conversion is available. Update momentary current, average current,
		and calculate new battery capacity. Update discharge_cycle_accumulator (and increment
		cycle_count if required).
		*/
		if( taskFlags.newCurrentAvailable )
		{
			taskFlags.newCurrentAvailable = false;
			
			// We use the CCADC interrupt as counter for the RTC
			runEverySecond();
			BATTCUR_StoreCurrent( CCADC_GetAccResult() );
			handleNewCCADCResult();
			CCGASG_AccumulateCCADCMeasurements( BATTCUR_GetCurrent(), slowRcOscillatorPeriod );
			
			//Set ledstatus
			SoC_State = GASG_StateOfCharge(BATTCUR_GetAverageCurrent(), battParams_sram.capacityInCCAccumulated);
			SetLEDs((uint8_t)SoC_State);
		}

		/*
		New core temperature reading is available and should be processed. If
		temperature has changed set RC_calibration_required flag.
		*/
		if( VADC_VTempReady() )
		{
			handleNewCoreTemperature();
		}

		/*
		RC oscillator requires calibration (new CC_ADC conversion period time should
		be calculated and stored.
		*/
		if( taskFlags.rcCalibrationRequired )
		{
			taskFlags.rcCalibrationRequired = false;
			RCCAL_StartCalibrateFastRC();
			
			// Calculating slow RC period works on whole kelvins
			slowRcOscillatorPeriod = RCCAL_CalculateSlowRCperiod( coreTemperature/10 );
		}
		
		if( RCCAL_GetState() == RCCAL_CALIB_INIT || RCCAL_GetState() == RCCAL_CALIB_WORKING ) {
			RCCAL_CalibrateFastRCruntime(slowRcOscillatorPeriod);
		}
		
		/*
		Battery cell temperature is available. Check it and disable FETs if too high/low
		*/
		if( VADC_CellTempReady() )
		{
			handleNewCellTemperature();
		}
		
		
		/*
		Check battery voltage cell . Check if critical (high/low voltage)
		*/
		if( VADC_Cell1VoltageReady() )
		{
			handleNewCellVoltage(CELL1);
		}
			
		/*
		Check battery voltage cell . Check if critical (high/low voltage) and misbalance
		*/
#if BATTPARAM_CELLS_IN_SERIES >= 2
		if( VADC_Cell2VoltageReady() )
		{
			handleNewCellVoltage(CELL2);
		}

#if BATTPARAM_CELLS_IN_SERIES >= 3
		if( VADC_Cell3VoltageReady() )
		{
			handleNewCellVoltage(CELL3);
		}

#if BATTPARAM_CELLS_IN_SERIES >= 4
		if( VADC_Cell4VoltageReady() )
		{
			handleNewCellVoltage(CELL4);
		}
#endif
#endif
#endif	
		
		// Check what sleep mode we can enter. Needs disable interrupt because otherwise it might
		// overwrite what interrupts write to SMCR (for example, if external interrupt is triggered and
		// wants the sleep mode to be no higher than idle, it should be run after all those checks)
		uint8_t interruptState = __save_interrupt();
		__disable_interrupt();
		if( Comm_IsIdle() && ! RCCAL_IS_RUNNING() ) {
			if( VADC_RUNNING == VADC_ScanState() ) {
				SLEEP_SET_ADC_NR_MODE();
			} else {
				SLEEP_SET_POWER_SAVE_MODE();
			}
		} else {
			SLEEP_SET_IDLE_MODE();
		}
		__restore_interrupt(interruptState);
		
		/*
		Disable FETs if critical condition have been detected.
		Enable them if that is okey, depending om various factors like
		temperature and voltage.
		*/
		if(   errorFlags.criticalConditionDetected
			 || stateFlags.simulatePowerOff)
		{
			FETCTRL_DisableFets();
		}
		else if( ! errorFlags.cellTemperatureTooHigh &&
		         ! errorFlags.cellTemperatureTooLow &&
		         ! stateFlags.inDUVR)
		{
			if(   ! errorFlags.voltageTooHigh
			   && ! errorFlags.reoccuringChargeProtection
			   && ! stateFlags.chargingProhibited
			   && ! sbsFlags.forceChargeFETDisabled)
			{
				FETCTRL_EnableChargeFet();
			}
			
			if(   ! errorFlags.voltageTooLow
			   && ! stateFlags.dischargingProhibited
			   && ! sbsFlags.forceDischargeFETDisabled)
			{
				FETCTRL_EnableDischargeFet();
			}
		}
		
		// The reason to do this is that on short-circuit, the cell voltage reading can
		// be wrong and if that causes both chargingProhibited and dischargingProhibited
		// to get set, they would never be cleared unless we do this.
		// If they should be set, they will be set again before the FETs are enabled again
		// on next cycle.
		if( stateFlags.chargingProhibited && stateFlags.dischargingProhibited ) {
			stateFlags.chargingProhibited = false;
			stateFlags.dischargingProhibited = false;
		}
		
		// Enter sleep if no new byte just have been received on the communication
		// If a byte is received between the check for Comm_Flag and __sleep is
		// executed, the sleep function will have no effect as the software communication
		// interrupt clears sleep mode enable.
		if( (Comm_Flag() == 0 ) && (ShowLedBusy()==false) ){
			__sleep();
		}
	}
}

/******************************************************************************/
/*! \brief Initialization of the Watchdog.
 *
 * Check if Watchdog System Reset has occured, if so a counter is incremented.
 * If the WSR counter reached WDR_LIMIT a failure is returned to indicate that
 * the system has failed several times and that it might be an idea to not resume
 * operation. The WSR counter is reset if any POR or BOD resets occurs.
 *
 *  \retval err_t Returns true if WSR counter has not passed limit, otherwise false.
 */
err_t Reset_Initialization()
{
	uint8_t reset_flags;
	
	/* Read and clear reset flags */
	reset_flags = MCUSR;                // Save reset flags.
	MCUSR       = 0;                    // Clear all flags previously set.
	
	/* If no reset flags are set, runaway code wrapped back to address 0 */
	if( reset_flags == 0 )
	{
		__disable_interrupt();
		// If desired, code for handling this error can be added here.
		for(;;);                        // Let the Watchdog time out and cause a Watchdog System Reset.
	}
	
	/* Check for Watchdog System Reset */
	if( reset_flags & (1<<WDRF) )
	{
		wdr_count++;                    // Increase Watchdog System Reset counter.
		
		/* Has the number of subsequent Watchdog System Resets exceeded its max limits? */
		if( wdr_count >= WDR_LIMIT )
		{
			return FAILURE;             // Report back an error message.
		}
	}
	
	/* Clear the Watchdog System Reset Counter on power-up or external reset */
	if( reset_flags & (1<<PORF) || reset_flags & (1<<EXTRF) )
	{
		wdr_count = 0;
	}
	
	return SUCCESS;
}


/******************************************/
/*! \brief Run this function every second
 *
 * - Update RTC and runEveryMinute function
 * - Check if overcharge has occured multiple times
 *
 */
#pragma inline=forced
void runEverySecond() {	
	// Update the RTC
	// Returns true if a new minute has occured, if it has, run that function
	if( RTC_AddSecond() ) {
		runEveryMinute();
	}
	
	/*
	If no battery protection interrupt has occured for BATTPROT_REOCCURING_CHARGE_PROTECTION_GAP
	seconds, clear chargeProtectionCounter ( the flag will be set to false when a discharge
	current is detected)
	It it has occured, and the counter is above the limit, set the reoccuringChargeProtection
	flag
	*/
	static uint8_t reoccuringChargeProtectionGapCounter = 0;
	if( BATTPROT_IsProtectionTriggered() ) {
		if( BATTPROT_GetChargeProtectionCounter() >= BATTPROT_REOCCURING_CHARGE_PROTECTION_LIMIT ) {
			errorFlags.reoccuringChargeProtection = true;
		}
	} else {
		if(BATTPROT_GetChargeProtectionCounter() != 0) {
			++reoccuringChargeProtectionGapCounter;
			if(reoccuringChargeProtectionGapCounter >= BATTPROT_REOCCURING_CHARGE_PROTECTION_GAP) {
				BATTPROT_ClearChargeProtectionCounter();
				reoccuringChargeProtectionGapCounter = 0;
			}
		}
	}
}


/*! \brief Run this function every minute
 *
 * Update the remaining capacity and time alarms
 * Update terminateDischarge and fullyDischarge if needed
 *
 */
#pragma inline=forced
void runEveryMinute() {
	// Update remaining capacity and time alarm
	// If they are set to 0, the alarms are disabled
	if(   battParams.remainingCapacityAlarm != 0
		 && CCGASG_Acc2mAh(CCGASG_ReadAccumulatedCC()) < battParams.remainingCapacityAlarm)
	{
		sbsFlags.remainingCapacityAlarm = true;
	} else {
		sbsFlags.remainingCapacityAlarm = false;
	}
	
	if(   battParams.remainingTimeAlarm != 0
		 && GASG_TimeToEmpty( BATTCUR_GetAverageCurrent() ) < battParams.remainingTimeAlarm)
	{
		sbsFlags.remainingTimeAlarm = true;
	} else {
		sbsFlags.remainingTimeAlarm = false;
	}
	
	// Check if the SBS terminate flag and/or fully discharge flag should be set.
	int32_t remCap = GASG_RemainingCapacity(BATTCUR_GetAverageCurrent());
	if( remCap < battParams_sram.terminateDischargeLimit ) {
		sbsFlags.terminateDischarge = true;
		
		if( remCap <= 0 ) {
			sbsFlags.fullyDischarged = true;
		}
	}
}


/***********************************/
/*! \brief Handle a new core temperature
 *
 * If temperature has changed too much, a rc recalibration is needed
 *
 * If the temperature is too high/low for the chip to operate, enter poweroff
 *
 * If there is no thermistors for the battery temperature, use the core temperature
 * for checking battery temperature.
 *
 */
static void handleNewCoreTemperature() {
	int16_t newCoreTemperature = VADC_readSystemTemperature( );
	
	//If the temperature has changed with more than CORE_TEMPERATURE_CHANGE_THRESHOLD the
	// RC oscillator needs calibration to ensure that communication works correctly and
	// that the CC-ADC readings are accurate.
	if( (coreTemperature + CORE_TEMPERATURE_CHANGE_THRESHOLD) <= newCoreTemperature ||
		 (coreTemperature - CORE_TEMPERATURE_CHANGE_THRESHOLD) >= newCoreTemperature )
	{
		coreTemperature = newCoreTemperature;
		taskFlags.rcCalibrationRequired = true;
	}
	
	// If the temperature is outside the operating range for the chip, enter power-off
	if( coreTemperature >= CORE_MAX_TEMPERATURE || coreTemperature <= CORE_MIN_TEMPERATURE ) {
		// Warning: Maybe check that it happens a couple of times in a row before powering off, to
		//          avoid faulty values.
		enterPoweroff();
	}
	
	// Assume that the core temperature is the same as the battery temperature when
	// no thermistor are used
#if HIGHEST_VADC == 0
	if( ! stateFlags.systemIsInStandby || anyTemperatureError() ) {
		if (checkBatteryTemperature(newCoreTemperature, BATTCUR_IsDischargeOngoing() ) ) {
			errorFlags.cellTemperatureTooHigh = false;
			errorFlags.cellTemperatureTooLow = false;
		}
	}
#endif
}

/***********************************/
/*! \brief Handle a new cell temperature
 *
 * If temperature is too high or too low, disable FETs.
 *
 * If in standby (very low current flowing), allow the temperature
 * to be too high/low (but read the temp anyway, so that the user can
 * read it)
 *
 */
static void handleNewCellTemperature() {
	
	// If no thermistor are configured, this function should never be called
	// , but to be safe, do a check anyway
#if HIGHEST_VADC == 0
		return;
	
#else
		uint16_t battCellTempVoltage = VADC_readVadc( VADC0 );
		cellTemperature[0] = NTC_ReadTemperatureKelvin(battCellTempVoltage);
		
#if HIGHEST_VADC == DEF_VADC1
		battCellTempVoltage = VADC_readVadc( VADC1 );
		cellTemperature[1] = NTC_ReadTemperatureKelvin(battCellTempVoltage);
#endif
		
		if( ! stateFlags.systemIsInStandby  || anyTemperatureError() ) {
			
			bool discharge = BATTCUR_IsDischargeOngoing();
			bool allOK = checkBatteryTemperature(cellTemperature[0], discharge);
			
			// Check the second thermistor if configured and first was okey. No need to check if one battery already outside limits
#if HIGHEST_VADC == DEF_VADC1
			{
				if( allOK ) {
					allOK = checkBatteryTemperature(cellTemperature[1], discharge);
				}
			}
#endif //HIGHEST_VADC == DEF_VADC1
			
			// If all temperatures were okay, set flags to safe
			if( allOK ) {
				errorFlags.cellTemperatureTooHigh = false;
				errorFlags.cellTemperatureTooLow = false;
			}
		}
#endif // else from HIGHEST_VADC == 0
}

/**************************/
/*! \brief Check if a battery temperature is within the limits
 *
 * \param temperature Temperature to check (in 0.1Kelvin)
 * \param discharge  True if battery is discharging, false if battery is charging
 *
 * \retval true  Battery temperature is within the limits
 * \retval false  Battery temperature was outside the limits. The corresponding FET have been disabled
 *
 */
static int8_t checkBatteryTemperature(int16_t temperature, bool discharge) {
	bool ret = true;
	
	if( discharge ) {
		if( battParams.cellMaxDischargeTemperature < temperature ) {
			FETCTRL_DisableDischargeFet();
			errorFlags.cellTemperatureTooHigh = true;
			ret = false;
			
		} else if( battParams.cellMinDischargeTemperature > temperature ) {
			FETCTRL_DisableDischargeFet();
			errorFlags.cellTemperatureTooLow = true;
			ret = false;
		}
	} else {
		if( battParams.cellMaxChargeTemperature < temperature ) {
			FETCTRL_DisableChargeFet();
			errorFlags.cellTemperatureTooHigh = true;
			ret = false;
			
		} else if( battParams.cellMinChargeTemperature > temperature ) {
			FETCTRL_DisableChargeFet();
			errorFlags.cellTemperatureTooLow = true;
			ret = false;
		}
	}
	return ret;
}

/***********************************/
/*! \brief Handle a new cell voltage for cells
 *
 * - Update Voltage based SoC (uses all cell voltages so only run for last cell)
 * - If cell voltage too low and discharging, disable discharge fet and don't enable it until charge current is detected
 * - If cell voltage too high, disable charge fet and don't enable it until discharge current is detected
 * - Check if cell balancing should be enabled
 *
 */
static void handleNewCellVoltage(vadcIndex_t TempCellVoltage) {
	
	// Clear voltageTooHigh and tooLow flags here
	// They can get set by either the handling of CELL1 or the CELL2,3 and 4,
	// but can only be cleared by CELL1.
	if (CELL1 == TempCellVoltage){
		//disableDUVRIfPossible();
	errorFlags.voltageTooLow = false;
	errorFlags.voltageTooHigh = false;
}


	if (HIGHEST_CELL == TempCellVoltage){
	// Update the voltage based state of charge. It assumes to be run once every second
	// Returns true if a new OCV SoC have been written, so use it to calculate new
	// remainingCapacity if that is considered inaccurate
	if(VBSoC_Update() && stateFlags.remainingCapacityInaccurate) {
		CCGASG_SetStateofCharge(VBSoC_OCSoC());
			stateFlags.remainingCapacityInaccurate = false;
	}
	}
	
	bool discharge = BATTCUR_IsDischargeOngoing();
	
	// Check that battery voltage is within spec - if not disable the charge/discharge FET.
	if( discharge ){
		if(BATTVMON_IsCellVoltageTooLow(TempCellVoltage) ){
		stateFlags.dischargingProhibited = true;
		FETCTRL_DisableDischargeFet();
		errorFlags.voltageTooLow = true;
		sbsFlags.fullyDischarged = true;
		}
		
		if (HIGHEST_CELL == TempCellVoltage){
			if( errorFlags.voltageTooLow ) {
		// When voltage is too low, it might also be so low that we should enter poweroff
		// Require two poweroff voltage in a row to shut down
		// If poweroff not is enabled, enterPoweroff() sets simulatePowerOff and disable the FETs
				if( VADC_readCellVoltage(CELL1) <= battParams.cellPoweroffVoltage
#if BATTPARAM_CELLS_IN_SERIES >= 2
		   || VADC_readCellVoltage(CELL2) <= battParams.cellPoweroffVoltage
#if BATTPARAM_CELLS_IN_SERIES >= 3
				 || VADC_readCellVoltage(CELL3) <= battParams.cellPoweroffVoltage
#if BATTPARAM_CELLS_IN_SERIES >= 4
				 || VADC_readCellVoltage(CELL4) <= battParams.cellPoweroffVoltage
#endif
#endif
#endif
				){
			if( stateFlags.poweroffCondition ) {
				enterPoweroff();
			} else {
				stateFlags.poweroffCondition = true;
			}
		} else {
			stateFlags.poweroffCondition = false;
		}
	}
	}
	} else if ( !discharge ){
		
		// Enable cell balancing if not in DUVR mode
		if( ! stateFlags.inDUVR ) {
			if( VADC_RUNNING != VADC_ScanState() ){
				if(SoC_State > 80){
				BATTVMON_EnableCellBalancing();
			}
		}
		}
		
		// Check if voltage is too high
		if( BATTVMON_IsCellVoltageTooHigh(TempCellVoltage) ) {
			stateFlags.chargingProhibited = true;
			FETCTRL_DisableChargeFet();
			errorFlags.voltageTooHigh = true;
		}
	}

}


/***********************************************/
/*! \brief Handle a new result from the CC-ADC
 *
 * - Update average current
 * - See if in standby or not
 * - Reset relaxation timer for the voltage based SoC
 * - If in (significant) discharge, some charging protection doesn't need to be checked
 * -
 * - Update full charge capacity if needed
 *
 */
static void handleNewCCADCResult() {
	int32_t offsetCompensatedCurrent = BATTCUR_GetOffsetCalibratedCurrent();
	
	BATTCUR_UpdateAverageCurrent( offsetCompensatedCurrent );
	
	// Set standby flag
	// Check if charge current is too high
	// Reset the relaxation timer for the voltage based gas gauging if device not is in standby anymore
	if( BATTCUR_IsCurrentLow(offsetCompensatedCurrent) )
	{
		stateFlags.systemIsInStandby = true;
	}
	else
	{
		stateFlags.systemIsInStandby = false;
		
		// Reset the relaxation counter
		VBSoC_ResetTimer();
		
		// If discharging (and not only standby discharge) clear charge protection flags
		// And also clear the fully charged bit
		if( BATTCUR_IsDischargeOngoing() ) {
			errorFlags.reoccuringChargeProtection = false;
			stateFlags.chargingProhibited = false;
			
			sbsFlags.fullyCharged = false;
			
		// When charging, make sure discharging not is prohibited anymore
		// If charging with too high current, do something perhaps....
		} else {
			stateFlags.dischargingProhibited = false;
			
			// Clear terminateDischarge as the host can start to draw power again when a charger is connected
			sbsFlags.terminateDischarge = false;
		}
	}
	
	// When charging, check if battery is fully charged
	// TODO: this will be wrong if the charger is on purpose charging with low current
	// or if it is incapable of delivering more current.
	// Also check if fullyDischarged flag can be cleared
	if( offsetCompensatedCurrent > 0 ) {
		if( sbsFlags.fullyDischarged && GASG_StateOfCharge(offsetCompensatedCurrent, battParams_sram.capacityInCCAccumulated) >= 20 ) {
			sbsFlags.fullyDischarged = false;
		}
		
		// Note: using lowest voltage raises the possibility of charging getting stopped
		//       but not considered a full charge because of cell misbalance. But this is
		//       probably what you want.
		uint16_t lowestVoltage;
#if BATTPARAM_CELLS_IN_SERIES == 1
		{
			lowestVoltage = VADC_readCellVoltage( CELL1 );
		}
#elif BATTPARAM_CELLS_IN_SERIES == 2
		{
			uint16_t cell1 = VADC_readCellVoltage( CELL1 );
			uint16_t cell2 = VADC_readCellVoltage( CELL2 );
			lowestVoltage = cell1 < cell2 ? cell1 : cell2;
		}
#elif BATTPARAM_CELLS_IN_SERIES == 3
		{
			uint16_t cell1 = VADC_readCellVoltage( CELL1 );
			uint16_t cell2 = VADC_readCellVoltage( CELL2 );
			uint16_t cell3 = VADC_readCellVoltage( CELL3 );
			lowestVoltage = cell1 < cell2 ? cell1 : cell2;
			lowestVoltage = lowestVoltage < cell3 ? lowestVoltage : cell3;
		}
#elif BATTPARAM_CELLS_IN_SERIES == 4
		{
			uint16_t cell1 = VADC_readCellVoltage( CELL1 );
			uint16_t cell2 = VADC_readCellVoltage( CELL2 );
			uint16_t cell3 = VADC_readCellVoltage( CELL3 );
			uint16_t cell4 = VADC_readCellVoltage( CELL4 );
			lowestVoltage = cell1 < cell2 ? cell1 : cell2;
			lowestVoltage = lowestVoltage < cell3 ? lowestVoltage : cell3;
			lowestVoltage = lowestVoltage < cell4 ? lowestVoltage : cell4;
		}
#endif
		
		// Update fullChargeCapacity if the battery is considered fully charged
		// TODO: update remainingCapacity calibration table here
		// This is only run during charging, so no point in avoiding calculations
		if( lowestVoltage > battParams.fullChargeVoltage ) {
			if( BATTCUR_mA2Ticks( battParams.fullChargeCurrent ) > offsetCompensatedCurrent ) {
				sbsFlags.fullyCharged = true;
				
				battParams_sram.capacityInCCAccumulated = CCGASG_ReadAccumulatedCC();
				
				// Only update in eeprom if there was a change
				int16_t temp = CCGASG_Acc2mAh(battParams_sram.capacityInCCAccumulated);
				if(temp != battParams.fullChargeCapacity) {
					battParams.fullChargeCapacity = temp;
				}
			}
		}
	}
}


/***********************************************/
/*! \brief Configure VADC scanning
 *
 * VADC measuring is started every second (every time a new CCADC result is ready)
 * but checking temperature every time is not necessary.
 *
 * Configure VADC for checking temperature for cells and core only every fourth second,
 * but not on the same second.
 */
static void configureVADC() {
	static uint8_t counter = 0;
	
	++counter;
	
	if(counter == 4) {
		VADC_ScanConfig((vadcIndex_t)HIGHEST_CELL, (vadcIndex_t)HIGHEST_VADC, (vadcIndex_t)0);
		counter = 0;
	} else if (counter == 2) {
		VADC_ScanConfig((vadcIndex_t)HIGHEST_CELL, (vadcIndex_t)0, (vadcIndex_t)VTEMP);
	} else {
		VADC_ScanConfig((vadcIndex_t)HIGHEST_CELL, (vadcIndex_t)0, (vadcIndex_t)0);
	}
}


/******************************************/
/*! \brief Disable DUVR mode if possible
 *
 */
static void disableDUVRIfPossible()
{
	if ( !(FCSR & (1<<DUVRD)) ) {
		stateFlags.inDUVR = true;
		if( SUCCESS == DUVR_DisableCheck() ){
			stateFlags.inDUVR = false;
		}
	}
}


/******************************************/
/*! \brief Return true if there is any temperature error
 *
 * \retval  true  Temperature is either too high or too low (for the battery)
 * \retval false  Temperature is okey for the battery
 */
static bool anyTemperatureError() {
	return (errorFlags.cellTemperatureTooHigh || errorFlags.cellTemperatureTooLow );
}


/******************************************/
/*! \brief Enter power-off if allowed
 *
 */
static void enterPoweroff()
{
#ifdef POWEROFF_ENABLED
	//SLEEP_ENTER_POWER_OFF();
#else
	FETCTRL_DisableFets();
	stateFlags.simulatePowerOff = true;
#endif
}


/***********************************/
/*! \brief Handle a SBS command
 *
 * Determine if it is a read or write, and whether it is a word or block command - and process accordingly
 *
 */
static void handleSBSCommand() {
	taskFlags.newSbsCommandAvailable = false;
	
	
	switch (sbs.command){
		//			SBSCMD_manufacturerAccess
	    case (SBS_WRITE | SBSCMD_manufacturerAccess):
		{
			break;
		}
	    case (SBS_READ | SBSCMD_manufacturerAccess):
		{
			// TODO: replace dummy value
			sbs.payloadW = 0x00;
			Comm_SbsWordCommandReply( &sbs );
			break;
		}
		//			SBSCMD_remainingCapacityAlarm
	    case (SBS_WRITE | SBSCMD_remainingCapacityAlarm):
		{
			battParams.remainingCapacityAlarm = sbs.payloadW;
			break;
		}
		//			SBSCMD_remainingCapacityAlarm
	    case (SBS_READ | SBSCMD_remainingCapacityAlarm):
		{
			sbs.payloadW = battParams.remainingCapacityAlarm;
			Comm_SbsWordCommandReply( &sbs );
			break;
		}
		//			SBSCMD_remainingTimeAlarm
	    case (SBS_WRITE | SBSCMD_remainingTimeAlarm):
		{
			battParams.remainingTimeAlarm = sbs.payloadW;
			break;
		}
		//			SBSCMD_remainingTimeAlarm
	    case (SBS_READ | SBSCMD_remainingTimeAlarm):
		{
			sbs.payloadW = battParams.remainingTimeAlarm;
			Comm_SbsWordCommandReply( &sbs );
			break;
		}
		//			SBSCMD_batteryMode
	case (SBS_WRITE | SBSCMD_batteryMode):
		{
			//Only supports setting the R/W bit for CAPACITY_MODE
			uint16_t temp = (battParams.batteryMode & 0x7FFF);
			temp |= (sbs.payloadW & 0x8000);	
			battParams.batteryMode = temp;
			break;
		}
		//			SBSCMD_batteryMode
	case (SBS_READ | SBSCMD_batteryMode):
		{
			// Add FCSR (fet control status register) to bits 2-5. These bits are reservered, but use them anyway...
			// Also add information about cell balancing to bits 10 and 11
			uint16_t temp = ((0x0F&FCSR)<<2); //DUVR, FETs & Current Protection Active Bits
			
#if BATTPARAM_CELLS_IN_SERIES >= 2
			temp |= ((CBCR & 0x08) << 3); //Balancing CELL4 bit
			temp |= ((CBCR & 0x07) << 10);//Balancing CELL1-3 bits
			
#endif
			sbs.payloadW = battParams.batteryMode | temp;
			Comm_SbsWordCommandReply( &sbs );
			break;
		}
		//			SBSCMD_atRate
	    case (SBS_WRITE | SBSCMD_atRate):
		{
			battParams_sram.atRate = (int16_t)sbs.payloadW;
			break;
		}
		//			SBSCMD_atRate
	    case (SBS_READ | SBSCMD_atRate):
		{
			sbs.payloadW = (uint16_t)battParams_sram.atRate;
			Comm_SbsWordCommandReply( &sbs );
			break;
		}
		//			SBSCMD_atRateTimeToFull
	    case (SBS_READ | SBSCMD_atRateTimeToFull):
		{
			sbs.payloadW = GASG_TimeToFull(battParams_sram.atRate);
			Comm_SbsWordCommandReply( &sbs );
			break;
		}
		//			SBSCMD_atRateTimeToEmpty
	    case (SBS_READ | SBSCMD_atRateTimeToEmpty):
		{
			sbs.payloadW = GASG_TimeToEmpty(battParams_sram.atRate);
			Comm_SbsWordCommandReply( &sbs );
			break;
		}
		//			SBSCMD_atRateOK
	    case (SBS_READ | SBSCMD_atRateOK):
		{
			sbs.payloadW = GASG_AtRateOK(battParams_sram.atRate);
			Comm_SbsWordCommandReply( &sbs );
			break;
		}
		//			SBSCMD_temperature
	    case (SBS_READ | SBSCMD_temperature):
		{
			sbs.payloadW = coreTemperature;
			Comm_SbsWordCommandReply( &sbs );
			break;
		}
		//			SBSCMD_voltage
	    case (SBS_READ | SBSCMD_voltage):
		{
#if BATTPARAM_CELLS_IN_SERIES == 1
			sbs.payloadW = VADC_readCellVoltage( CELL1 );
#elif BATTPARAM_CELLS_IN_SERIES == 2
			sbs.payloadW = VADC_readCellVoltage( CELL2 )
			             + VADC_readCellVoltage( CELL1 );
#elif BATTPARAM_CELLS_IN_SERIES == 3
			sbs.payloadW = VADC_readCellVoltage( CELL3 )
			             + VADC_readCellVoltage( CELL2 )
			             + VADC_readCellVoltage( CELL1 );
#elif BATTPARAM_CELLS_IN_SERIES == 4
			sbs.payloadW = VADC_readCellVoltage( CELL4 )
			             + VADC_readCellVoltage( CELL3 )
			             + VADC_readCellVoltage( CELL2 )
			             + VADC_readCellVoltage( CELL1 );
#endif
			Comm_SbsWordCommandReply( &sbs );
			break;
		}
		
		//			SBSCMD_current
	    case (SBS_READ | SBSCMD_current):
		{
			sbs.payloadW = BATTCUR_Ticks2mA( BATTCUR_GetOffsetCalibratedCurrent() );
			Comm_SbsWordCommandReply( &sbs );
			break;
		}
		//			SBSCMD_averageCurrent
	    case (SBS_READ | SBSCMD_averageCurrent):
		{
			sbs.payloadW = BATTCUR_Ticks2mA( BATTCUR_GetAverageCurrent() );
			Comm_SbsWordCommandReply( &sbs );
			break;
		}
		//			SBSCMD_maxError
	    case (SBS_READ | SBSCMD_maxError):
		{
			// TODO REMOVE clear simulatePowerOff when read, to avoid having to go in via debugwire
			stateFlags.simulatePowerOff = false;
			// TODO replace dummy reply value with correct reply.
			sbs.payloadW = 0x0000;
			Comm_SbsWordCommandReply( &sbs );
			break;
		}
		//			SBSCMD_relativeStateOfCharge
	    case (SBS_READ | SBSCMD_relativeStateOfCharge):
		{
			sbs.payloadW = GASG_StateOfCharge(BATTCUR_GetAverageCurrent(), battParams_sram.capacityInCCAccumulated);
			Comm_SbsWordCommandReply( &sbs );
			break;
		}
		//			SBSCMD_absoluteStateOfCharge
	    case (SBS_READ | SBSCMD_absoluteStateOfCharge):
		{
			
                        sbs.payloadW = GASG_StateOfCharge(BATTCUR_GetAverageCurrent(), CCGASG_mAh2Acc(battParams.designCapacity));
			Comm_SbsWordCommandReply( &sbs );
			break;
		}
		//			SBSCMD_remainingCapacity
	    case (SBS_READ | SBSCMD_remainingCapacity):
		{
			sbs.payloadW = CCGASG_Acc2mAh(GASG_RemainingCapacity(BATTCUR_GetAverageCurrent()));
                       	Comm_SbsWordCommandReply( &sbs );
			break;
		}
		//			SBSCMD_fullChargeCapacity
	    case (SBS_READ | SBSCMD_fullChargeCapacity):
		{
			sbs.payloadW = battParams.fullChargeCapacity;
			Comm_SbsWordCommandReply( &sbs );
			break;
		}
		//			SBSCMD_runTimeToEmpty
	    case (SBS_READ | SBSCMD_runTimeToEmpty):
		{
			
			int32_t temp = BATTCUR_GetOffsetCalibratedCurrent();
			temp = BATTCUR_Ticks2mA(temp);
			sbs.payloadW = GASG_TimeToEmpty( (int16_t)temp );
			Comm_SbsWordCommandReply( &sbs );
			break;
		}
		//			SBSCMD_averageTimeToEmpty
	    case (SBS_READ | SBSCMD_averageTimeToEmpty):
		{
			int32_t temp = BATTCUR_GetAverageCurrent();
			temp = BATTCUR_Ticks2mA(temp);
			sbs.payloadW = GASG_TimeToEmpty((int16_t)temp);
			Comm_SbsWordCommandReply( &sbs );
			break;
		}
		//			SBSCMD_averageTimeToFull
	    case (SBS_READ | SBSCMD_averageTimeToFull):
		{
			int32_t temp = BATTCUR_GetAverageCurrent();
			temp = BATTCUR_Ticks2mA(temp);
			sbs.payloadW = GASG_TimeToFull((int16_t)temp);
			Comm_SbsWordCommandReply( &sbs );
			break;
		}
		//			SBSCMD_chargingCurrent
	    case (SBS_READ | SBSCMD_chargingCurrent):
		{
			sbs.payloadW = battParams.chargingCurrent;
			Comm_SbsWordCommandReply( &sbs );
			break;
		}
		//			SBSCMD_chargingVoltage
	    case (SBS_READ | SBSCMD_chargingVoltage):
		{
			sbs.payloadW = battParams.chargingVoltage;
			Comm_SbsWordCommandReply( &sbs );
			break;
		}
		//			SBSCMD_batteryStatus
	    case (SBS_READ | SBSCMD_batteryStatus):
		{
			sbs.payloadW =
			  (         stateFlags.chargingProhibited << BATTSTAT_OVER_CHARGED_ALARM_BIT)
			| (             errorFlags.voltageTooHigh << BATTSTAT_TERMINATE_CHARGE_ALARM_BIT)
			| (  (bool)(VREGMON_ConditionTriggered()) << BATTSTAT_VREGMON_TRIGGERED_BIT)
			| (     errorFlags.cellTemperatureTooHigh << BATTSTAT_OVER_TEMP_ALARM_BIT)
			| (           sbsFlags.terminateDischarge << BATTSTAT_TERMINATE_DISCHARGE_ALARM_BIT)
			| (      BATTPROT_IsProtectionTriggered() << BATTSTAT_BATTERY_PROTECTION_TRIGGERED_BIT)
			| (       sbsFlags.remainingCapacityAlarm << BATTSTAT_REMAINING_CAPACITY_ALARM_BIT)
			| (           sbsFlags.remainingTimeAlarm << BATTSTAT_REMAINING_TIME_ALARM_BIT)
			| (! errorFlags.criticalConditionDetected << BATTSTAT_INITIALIZED_BIT)
			| (          BATTCUR_IsDischargeOngoing() << BATTSTAT_DISCHARGING_BIT)
			| (                 sbsFlags.fullyCharged << BATTSTAT_FULLY_CHARGED_BIT)
			| (              sbsFlags.fullyDischarged << BATTSTAT_FULLY_DISCHARGED_BIT) ;
			
			Comm_SbsWordCommandReply( &sbs );
			break;
		}
		//			SBSCMD_cycleCount
	    case (SBS_READ | SBSCMD_cycleCount):
		{
			sbs.payloadW = BATTPARAM_GetCycleCount();
			Comm_SbsWordCommandReply( &sbs );
			break;
		}
		//			SBSCMD_designCapacity
	    case (SBS_READ | SBSCMD_designCapacity):
		{
			sbs.payloadW = battParams.designCapacity;
			Comm_SbsWordCommandReply( &sbs );
			break;
		}
		//			SBSCMD_designVoltage
	    case (SBS_READ | SBSCMD_designVoltage):
		{
			
			sbs.payloadW = battParams.designVoltage;
			Comm_SbsWordCommandReply( &sbs );
			break;
		}
		//			SBSCMD_specificationInfo
	    case (SBS_READ | SBSCMD_specificationInfo):
		{
			sbs.payloadW = battParams.specificationInfo;
			Comm_SbsWordCommandReply( &sbs );
			break;
		}
		//			SBSCMD_manufactureDate
	    case (SBS_READ | SBSCMD_manufactureDate):
		{
			sbs.payloadW = battParams.manufactureDate;
			Comm_SbsWordCommandReply( &sbs );
			break;
		}
		//			SBSCMD_serialNumber
	    case (SBS_READ | SBSCMD_serialNumber):
		{
			sbs.payloadW = battParams.serialNumber;
			Comm_SbsWordCommandReply( &sbs );
			break;
		}
		//			SBSCMD_manufacturerName
	    case (SBS_READ | SBSCMD_manufacturerName):
		{
			BATTPARAM_GetString( (BATTPARAM_blockParameter_t*)&battParams.manufacturerName, (SBS_command_t*)&sbs );
			Comm_SbsBlockCommandReply( &sbs );
			break;
		}
		//			SBSCMD_deviceName
	    case (SBS_READ | SBSCMD_deviceName):
		{
			BATTPARAM_GetString( (BATTPARAM_blockParameter_t*)&battParams.deviceName, (SBS_command_t*)&sbs );
			Comm_SbsBlockCommandReply( &sbs );
			break;
		}
		//			SBSCMD_deviceChemistry
	    case (SBS_READ | SBSCMD_deviceChemistry):
		{
			BATTPARAM_GetString( (BATTPARAM_blockParameter_t*)&battParams.deviceChemistry, (SBS_command_t*)&sbs );
			Comm_SbsBlockCommandReply( &sbs );
			break;
		}
		//			SBSCMD_manufacturerData
	    case (SBS_READ | SBSCMD_manufacturerData):
		{
			BATTPARAM_GetString( (BATTPARAM_blockParameter_t*)&battParams.manufacturerData, (SBS_command_t*)&sbs );
			Comm_SbsBlockCommandReply( &sbs );
			break;
		}
		//			SBSCMD_authentication
	    case (SBS_WRITE | SBSCMD_authentication):
		{
			// Run the authentication in the main loop
			taskFlags.doAuthentication = true;
			break;
		}
	    case (SBS_READ | SBSCMD_authentication):
		{
			//Get pointer to the authenticated message.
			sbsReply = AUTH_GetResponse();
			// Pass pointer to authenticated message to communication module.
			Comm_SbsBlockCommandReply( sbsReply );
			break;
		}
		//			SBSCMD_shuntCalibration
	    case (SBS_WRITE | SBSCMD_shuntCalibration):
		{
			// This is the shunt resistance in microOhms
			uint16_t shuntTemp = sbs.payloadW; // Get variable as soon as possible to avoid overwriting
			battParams.shuntResistance = shuntTemp;
			BATTCUR_CalculateShuntCoeffient(shuntTemp,CCADC_VOLTREF);
                        uint16_t temp = RCCAL_CalculateUlpRCperiod(coreTemperature/10);
			CCGASG_CalculateShuntCoefficients(temp,shuntTemp,CCADC_VOLTREF);
			break;
		}
	    case (SBS_READ | SBSCMD_shuntCalibration):
		{
			sbs.payloadW = battParams.shuntResistance;
			Comm_SbsWordCommandReply( &sbs );
			break;
		}
		// SBSCMD_FETdisable
	    case (SBS_WRITE | SBSCMD_FETdisable):
		{
			if(sbs.payloadW & (1<<FETDIS_CHARGE)) {
				sbsFlags.forceChargeFETDisabled = true;
				FETCTRL_DisableChargeFet();
			} else {
				sbsFlags.forceChargeFETDisabled = false;
			}
			
			if(sbs.payloadW & (1<<FETDIS_DISCHARGE)) {
				sbsFlags.forceDischargeFETDisabled = true;
				FETCTRL_DisableDischargeFet();
			} else {
				sbsFlags.forceDischargeFETDisabled = false;
			}
			break;
		}
		// SBSCMD_storageMode
	    case (SBS_WRITE | SBSCMD_storageMode):
		{
			// Enter poweroff if the received word is correct
			if(sbs.payloadW == 0xFADE) {
				enterPoweroff();
			}
			break;
		}
		//			SBSCMD_temperatureCell2
	    case (SBS_READ | SBSCMD_temperatureNTC2):
		{
			// If no thermistor, return invalid value
			// If thermistor, return second temperature
#if HIGHEST_VADC < DEF_VADC1
			sbs.payloadW = 0x0000;
#else
			sbs.payloadW = cellTemperature[1];
#endif
			Comm_SbsWordCommandReply( &sbs );
			break;
		}
		//			SBSCMD_temperatureCell1
	    case (SBS_READ | SBSCMD_temperatureNTC1):
		{
			// If no thermistor, return invalid value
			// If thermistor, return first temperature
#if HIGHEST_VADC < DEF_VADC0
			sbs.payloadW = 0x0000;
#else
			sbs.payloadW = cellTemperature[0];
#endif
			Comm_SbsWordCommandReply( &sbs );
			break;
		}
		//			SBSCMD_OptionalMfgFunction5
		//			SBSCMD_voltageCell4            // Commonly referred to as OptionalMfgFunction4, read only
	    case (SBS_READ | SBSCMD_voltageCell4):
		{
			sbs.payloadW = VADC_readCellVoltage( CELL4 );
			Comm_SbsWordCommandReply( &sbs );
			break;
		}
		//			SBSCMD_voltageCell3            // Commonly referred to as OptionalMfgFunction3, read only
	    case (SBS_READ | SBSCMD_voltageCell3):
		{
			sbs.payloadW = VADC_readCellVoltage( CELL3 );
			Comm_SbsWordCommandReply( &sbs );
			break;
		}
		//			SBSCMD_voltageCell2            // Commonly referred to as OptionalMfgFunction2, read only
	    case (SBS_READ | SBSCMD_voltageCell2):
		{
			sbs.payloadW = VADC_readCellVoltage( CELL2 );
			Comm_SbsWordCommandReply( &sbs );
			break;
		}
		//			SBSCMD_voltageCell1            // Commonly referred to as OptionalMfgFunction1, read only
	    case (SBS_READ | SBSCMD_voltageCell1):
		{
			sbs.payloadW = VADC_readCellVoltage( CELL1 );
			Comm_SbsWordCommandReply( &sbs );
			break;
		}
		
	    default:
		break;
	}
	
}
