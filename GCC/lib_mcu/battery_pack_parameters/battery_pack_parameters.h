/* This file has been prepared for Doxygen automatic documentation generation.*/
/*! \file *********************************************************************
 *
 * \brief
 *      Definition file for a smart battery pack.
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
 * $Revision: 6090 $
 * $Date: 2009-10-05 21:40:12 +0800 (Mon, 05 Oct 2009) $  \n
 *
 * Copyright (c) 2009, Atmel Corporation All rights reserved.
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
 ******************************************************************************/


#ifndef BATT_PACK_PARAM_H
#define BATT_PACK_PARAM_H

#include "sbs_commands.h"
#include "vadc.h"
#include "battery_protection.h"
#include "battery_current_monitoring.h"
#include "crc16.h"
#include <avr/eeprom.h>

/******************************************************************************
 Commonly used units, to make reading the defines easier
******************************************************************************/
//Units
#define V    //!< Units: Volts
#define mV   //!< Units: Milli volts
#define A    //!< Units: Amps
#define uA   //!< Units: micro Amps
#define mA   //!< Units Milli amps
#define uOhm //!< Units Micro Ohm
#define mOhm //!< Units Milli Ohm
#define us   //!< Units: Micro seconds
#define ms   //!< Units: Milli seconds
#define mAh  //!< Units: Milli amp hours
#define minutes //!< Units: minutes
#define dC  //!< Units: degrees Celcius
#define dK  //!< Units: degrees Kelvin
#define percent //!< Units: percentage

/******************************************************************************
 Common parameters of AVR Device
******************************************************************************/
/*! \brief      CC-ADC internal reference voltage in mV. */
#define CCADC_VOLTREF_110mV 110
#define CCADC_VOLTREF_220mV 220

#define CCADC_VOLTREF       CCADC_VOLTREF_220mV


/******************************************************************************
 Battery parameters
******************************************************************************/
#define SB202_2
#ifdef SB202_2 				//  placed in Project Options, Compiler, Preprossesor
  #define BATTPARAM_CELLS_IN_SERIES (2)                  //!< Number of cells in series.
#endif
#ifdef SB202_3
  #define BATTPARAM_CELLS_IN_SERIES (3)                  //!< Number of cells in series.
#endif
#ifdef SB202_4
  #define BATTPARAM_CELLS_IN_SERIES (4)                  //!< Number of cells in series.
#endif

#define BATTPARAM_MANUFACTURE_DAY (21)		//!< Manufacturing day: 20th. Format: dd
#define BATTPARAM_MANUFACTURE_MONTH (2)	//!< Manufacturing month: October. Format m/mm
#define BATTPARAM_MANUFACTURE_YEAR (2009)	//!< Manufacturing year: 2008. Format: yyyy.

#define BATTPARAM_REVISION (0x01)
#define BATTPARAM_VERSION (0x02)
#define BATTPARAM_VSCALE (0)
#define BATTPARAM_IPSCALE (0)

/*! Block data (initial values for the standard SBS commands) */
#define BATTPARAM_MANUFACTURERNAME ("Atmel")  //!< string: Name of manufacturer.
#ifdef SB202_2
  #define BATTPARAM_DEVICENAME ("SB202-2 - dual cells") //!< string: device (product) name
#endif
#ifdef SB202_3
  #define BATTPARAM_DEVICENAME ("SB202-3 - three cells") //!< string: device (product) name
#endif
#ifdef SB202_4
  #define BATTPARAM_DEVICENAME ("SB202-4 - four cells") //!< string: device (product) name
#endif
#define BATTPARAM_DEVICECHEMISTRY ("Lithium-Ion") //!< string: Battery type
#define BATTPARAM_MANUFACTURERDATA ("App note AVR474")  //!< String: Optional data that the manufacturer can provide.

/*!Word data (initial values for the standard SBS commands) */
#define BATTPARAM_BATTERYMODE (0x6000)        //!< Refer to the SBS specification.
#define BATTPARAM_CHARGING_CURRENT (650 mA)   //!< Maximum charging current.
#define BATTPARAM_CHARGING_VOLTAGE (4200 mV)  //!< Maximum charging voltage.
#define BATTPARAM_DESIGN_VOLTAGE (3600 mV)   //!< Typical operation voltage.
#define BATTPARAM_SPECIFICATION_INFO ( (BATTPARAM_REVISION & 0x000F) | \
	((BATTPARAM_VERSION & 0x000F)<<4) | \
	((BATTPARAM_VSCALE & 0x000F)<< 8) | \
	((BATTPARAM_IPSCALE & 0x000F)<<12) )  //!< Various pack information (composed of other defines)
#define BATTPARAM_MANUFACTURE_DATE ((BATTPARAM_MANUFACTURE_DAY) | \
	(BATTPARAM_MANUFACTURE_MONTH<<5) | \
	( (BATTPARAM_MANUFACTURE_YEAR-1980)<<9))	//!< Manufacturing data (composed of other defines)
#define BATTPARAM_SERIALNUMBER (0x08)  //!< Serial number (SBS standard)

/*! Word data (non-SBS data) */
#define BATTPARAM_SERIALNUMBER1 (0x0101)   //!< More serial number.
#define BATTPARAM_SERIALNUMBER2 (0x0202)   //!< More serial number.
#define BATTPARAM_SERIALNUMBER3 (0x0303)   //!< More serial number.

#define BATTPARAM_CELL_CHARGE_THRESHOLD ( 50 mV )      //!< How many mV above the charging voltage the charger is allowed to charge
#define BATTPARAM_CELL_MIN_VOLTAGE (2700 mV)           //!< Minimum operation voltage (disable further discharging below this point).
#define BATTPARAM_CELL_POWEROFF_VOLTAGE (2500 mV)      //!< If cell voltage is below this, the device will power off
//! Maximum operation voltage (disable further charging above this point).
#define BATTPARAM_CELL_MAX_VOLTAGE (BATTPARAM_CHARGING_VOLTAGE + BATTPARAM_CELL_CHARGE_THRESHOLD)
#define BATTPARAM_MISBALANCE_VOLTAGE_THRESHOLD (5 mV)  //!< If voltage difference between cells are more than this value the balancing FET for cell with the highest voltage will be enabled.
//! If all cell voltages is above this, disable DUVR mode
#define BATTPARAM_CELL_DUVR_DISABLE (2000 mV)

#define BATTPARAM_FULL_CHARGE_VOLTAGE ( 4190 mV )      //!< If lowest cell is above this and current is low enough, assume the battery is fully charged
#define BATTPARAM_FULL_CHARGE_CURRENT ( 15 mA )        //!< If less than this charge current is flowing and lowest cell voltage is high enough, assume the battery is fully charged

#define BATTPARAM_CELL_MIN_DISCHARGE_TEMPERATURE ((-20 dC + 273)*10) //!< -20°C converted to 0.1 degrees Kelvin
#define BATTPARAM_CELL_MAX_DISCHARGE_TEMPERATURE (( 60 dC + 273)*10) //!<  60°C converted to 0.1 degrees Kelvin
#define BATTPARAM_CELL_MIN_CHARGE_TEMPERATURE    (( -5 dC + 273)*10) //!<  -5°C converted to 0.1 degrees Kelvin
#define BATTPARAM_CELL_MAX_CHARGE_TEMPERATURE    (( 40 dC + 273)*10) //!<  40°C converted to 0.1 degrees Kelvin

#define CORE_MAX_TEMPERATURE (( 85 dC + 273)*10) //!<  85°C converted to 0.1 degrees Kelvin
#define CORE_MIN_TEMPERATURE ((-22 dC + 273)*10) //!< -22°C converted to 0.1 degress Kelvin

//Short Circuit Protection Level: MaxCurrent * ShuntResistor = VMaxLevel
//                                3000mA * 5000uOhm = 16.25mV
#define BATTPARAM_SHORTCIRCUIT_REACTIONTIME ( BATTPROT_ScrtUsecToHex(125 us) ) //!< Must shut off the FETs if battery shorted for more than 125 us
#define BATTPARAM_SHORTCIRCUIT_CURRENT (BATTPROT_VMAX_15mV)                   //!< Short-circuit current level.

//Over Current Protection Level: NearestOffset + (MaxCurrent * ShuntResistor) = NearestOffset + VMaxLevel
//Over Current Charge Protection Level:     1000mA * 5000uOhm = 5mV
//Over Current DisCharge Protection Level:  2000mA * 5000uOhm = 10mV
#define BATTPARAM_OVERCURRENT_REACTIONTIME ( BATTPROT_OcrtMsecToHex(2 ms) )    //!< Must shut off the FETs if over-current is present for more than 2 ms
#define BATTPARAM_OVERCURRENT_CHARGE  (BATTPROT_VMAX_5mV)                    //!< Overcurrent level (charging).
#define BATTPARAM_OVERCURRENT_DISCHARGE  (BATTPROT_VMAX_10mV)                 //!< Overcurrent level (discharging).

//High Current Protection Level: MaxCurrent * ShuntResistor = VMaxLevel
//High Current Charge Protection Level:     1000mA * 5000uOhm = 5mV
//High Current DisCharge Protection Level:  2000mA * 5000uOhm = 10mV
#define BATTPARAM_HIGHCURRENT_REACTIONTIME ( BATTPROT_HcrtMsecToHex(20 ms) )   //!< Must shut off the FETs if high-current is present for more than 20 ms
#define BATTPARAM_HIGHCURRENT_CHARGE  (BATTPROT_VMAX_5mV)                    //!< Highcurrent level (charging).
#define BATTPARAM_HIGHCURRENT_DISCHARGE (BATTPROT_VMAX_10mV)                  //!< Highcurrent level (discharging).

#define BATTPARAM_SHUNT_RESISTANCE ( 5000 uOhm )                //!< Resistance of the shunt resistor in uOhm (microOhm)
#define BATTPARAM_DESIGN_CAPACITY (650 mAh)	                     //!< Original capacity of cells.
#define BATTPARAM_FULL_CHARGE_CAPACITY BATTPARAM_DESIGN_CAPACITY //!< Full charge capacity is assumed to be design capacity at beginning
#define BATTPARAM_REMAINING_CAPACITY_ALARM (BATTPARAM_DESIGN_CAPACITY / 10) //!< Default low capacity alarm
#define BATTPARAM_REMAINING_TIME_ALARM (10 minutes)              //!< Remaining time alarm limit
#define BATTPARAM_TERMINATE_DISCHARGE_LIMIT (5 mAh)              //!< If remaining capacity is lower than this, flag the TERMINATE_DISCHARGE alarm in the BatteryStatus command


#define BATTPARAM_ACTIVE_CURRENT_THRESHOLD ( 2 mA )    //!< If current drawn from the battery is above this threshold the application is assumed to be in active mode.
#define BATTPARAM_RELAXATION_TIME		30     //!< Relaxation time after active current before it is considered open circuit again in minutes.

//! CRC checksum for all parameters saved in eeprom.
#define BATTPARAM_CRCCHECKSUM (0x0000)  // Checksum for the BATTPARAM data - if checksum is incorrect the battery will not be enabled.


/*****************************************************************************
 Defines derived from the battery parameters
 ****************************************************************************/

// Define the number highest cell, to be used when specifiying variables etc
// Defines are used instead of the enums because enums can't be used in preprocessor directives
#if BATTPARAM_CELLS_IN_SERIES == 2
# define HIGHEST_CELL DEF_CELL2
#elif BATTPARAM_CELLS_IN_SERIES == 3
# define HIGHEST_CELL DEF_CELL3
#elif BATTPARAM_CELLS_IN_SERIES == 4
# define HIGHEST_CELL DEF_CELL4
#endif

// Defines for the number of thermistors to use
// The DEF_ are used because using enums in preprocessor directive doesn't work
// HELP: If 0 thermistors are used: Set HIGHEST_VADC to 0 and CELLTEMPERATURE_INPUTS to 0
//			 If 1 thermistor is used:   Set HIGHEST_VADC to DEF_VADC0 and CELLTEMPERATURE_INPUTS to 1
//			 If 2 thermistors are used: Set HIGHEST_VADC to DEF_VADC1 and CELLTEMPERATURE_INPUTS to 2
#define HIGHEST_VADC 0
#define CELLTEMPERATURE_INPUTS 0


/******************************************************************************
 Misc. defines
*******************************************************************************/
//! If defined, the part is allowed to enter poweroff. Good idea to undefine during normal testing/debugging, because debugwire doesn't like poweroff
//#define POWEROFF_ENABLED

#define AVERAGE_CURRENT_SCALING 4 //!< The averaging current is scaled up by 2^x to increase accuracy.

// Decides which authentication method to use (only define one)
//#define AUTH_USE_AES  //!< Use AES encryption as the authentication algorithm
#define AUTH_USE_HMAC_SHA2 //!< Use HMAC-SHA256 for the authentication algorithm


#define AES_KEY_SIZE 16 //!< Size in bytes of the AES key
//#define AES_INCLUDE_DECRYPT //!< If this is defined, the aes_lib will include functions for decrypting

#define SHA_INPUT_BLOCK_SIZE 32 //!< Size in bytes of the HMAC input data (if the same size as SBSDATA_BLOCK_LENGTH, the status byte is used as well)


#define CORE_TEMPERATURE_CHANGE_THRESHOLD ((2 dK)*10)           //!< If temperature changes more than 2 degrees Kelvin - it is an official change. *10 because all internal temperatures are 0.1Kelvin

#define INTERNAL_BATT_RESISTOR_TIMES_128 28 //!< The internal batt resistance in ohms *128

#define CONVERSION_COUNT	10	//!< Number of conversions to run before changing polarity of the CC-ADC

//! If defined, the code will compensate for that the first reading after a polarity switch is wrong.
#define COMPENSATE_FOR_ERROR_AFTER_POLARITY_SWITCH

//! Size of the remaining capacity compensation table
#define REMAINING_CAPACITY_TABLE_SIZE 5


/**************************************
 Defines derived from the misc defines
 And some error checking
 **************************************/
#if defined(AUTH_USE_AES) && defined(AUTH_USE_HMAC_SHA2)
# error Cannot use both AES and HMAC-SHA2
#endif


#if SHA_INPUT_BLOCK_SIZE > 52
# error SHA_INPUT_BLOCK_SIZE cannot be larger than 52 (it can be, but the code has to be changed)
#endif
#if SHA_INPUT_BLOCK_SIZE > SBSDATA_BLOCK_LENGTH
#	error SHA_INPUT_BLOCK_SIZE cannot be larger than SBSDATA_BLOCK_LENGTH
#endif
#if SHA_INPUT_BLOCK_SIZE % 4 != 0
#	error SHA_INPUT_BLOCK_SIZE has to a multiple of 4
#endif

#define SHA_INPUT_BLOCK_WORDS (SHA_INPUT_BLOCK_SIZE/4)


/*!
 * If charge protection occurs BATTPROT_REOCCURING_CHARGE_PROTECTION_LIMIT times
 * "in a row", the charge fet will get disabled until a discharge current is
 * detected. This protects the battery from a malfunctioning charger.
 *
 * Because the FET automatically will be disabled for one second after a over/high
 * current has occured, it might not occur again the second after. So to be able to
 * count how many times it has occured in a row, more than BATTPROT_REOCCURING_CHARGE_PROTECTION_GAP
 * seconds have to elapse until it is not considered as in a row again.
 */
#define BATTPROT_REOCCURING_CHARGE_PROTECTION_LIMIT 5
#define BATTPROT_REOCCURING_CHARGE_PROTECTION_GAP 3

#define WDR_LIMIT                 3     //!< Watchdog System Reset count cannot exceed this limit.	

enum batteryStatusBitsEnum {
	BATTSTAT_OVER_CHARGED_ALARM_BIT = 15,
	BATTSTAT_TERMINATE_CHARGE_ALARM_BIT = 14,
	BATTSTAT_VREGMON_TRIGGERED_BIT = 13, //!< Reserved
	BATTSTAT_OVER_TEMP_ALARM_BIT = 12,
	BATTSTAT_TERMINATE_DISCHARGE_ALARM_BIT = 11,
	BATTSTAT_BATTERY_PROTECTION_TRIGGERED_BIT = 10, //!< Reserved
	BATTSTAT_REMAINING_CAPACITY_ALARM_BIT = 9,
	BATTSTAT_REMAINING_TIME_ALARM_BIT = 8,
	
	BATTSTAT_INITIALIZED_BIT = 7,
	BATTSTAT_DISCHARGING_BIT = 6,
	BATTSTAT_FULLY_CHARGED_BIT = 5,
	BATTSTAT_FULLY_DISCHARGED_BIT = 4
};

enum batteryModeExtraStatusBitsEnum {
	BATTMODE_BALANCING_CELL1 = 10,
	BATTMODE_BALANCING_CELL2 = 11
};

enum FETdisableCommandBitsEnum {
	FETDIS_CHARGE = 0,
	FETDIS_DISCHARGE = 1
};

/******************************************************************************
 Commonly used macro functions for various conversions etcetera
*******************************************************************************/
#define DEGREES_C_TO_K( degrees ) ((degrees) + 273) //!< Convert degrees Celsius to Kelvin.


/******************************************************************************
Type definitions
******************************************************************************/

typedef uint16_t BATTPARAM_wordParameter_t;
typedef uint8_t BATTPARAM_blockParameter_t;

struct battParam_sramstruct {
	//! Used by all the AtRate commands, it doesn't do anything if this is reset on reset
	int16_t  atRate;
	
	//! If current drawn from the battery is above this threshold the application is assumed to be in active mode.
	uint16_t activeCurrentThresholdInTicks;
	
	//! Full charge capacity in CC accumulated
	uint32_t capacityInCCAccumulated;
	
	//! Terminate discharge limit in CC accumulated
	int32_t terminateDischargeLimit;
	
	//! In ticks
	uint16_t remainingCapacityCalibrationCurrents[REMAINING_CAPACITY_TABLE_SIZE];
	//! In CCGASG accumulation divided by 2^8 (to fit in uint16_t. The difference should be low enough to make sure an overflow doesn't occur even if the possibility exists)
	uint16_t remainingCapacityCalibration[REMAINING_CAPACITY_TABLE_SIZE];
};



/******************************************************************************
 Public variables
******************************************************************************/
extern struct battParam_sramstruct battParams_sram; //!< Battery parameters that are calculated at startup from eeprom variables

/* Values for remainingCapacity "calibration" depending on current */
extern const uint16_t remainingCapacityCalibrationCurrents[];
extern const uint8_t  remainingCapacityCalibration[];

// Defines for at which address to locate eeprom variables
#define EEPROM_BATTPARAMS 0x00
#define EEPROM_AESKEY 0xD0
#define EEPROM_ACCOFFSET 0xE0
#define EEPROM_WDR_COUNT 0xE2
#define EEPROM_CYCLE_COUNT 0xE4

extern uint8_t EEMEM wdr_count;   //!< Counts number of Watchdog resets. Initialize to 0 on first programming.

/*! Key for the AES authentication */
extern uint8_t EEMEM AUTH_aes_key[16];

/*!
 * Offset for the CCADC acc conversion. Can be added to result from negative
 * polarity and removed from results with positive polarity to get more "steady"
 * results
 *
 * It will be recalculated at startup if it is "32767", and that takes between 20 and 30
 * seconds, so set this to 0 if you want to avoid that.
 *
 */
extern int16_t EEMEM accOffset;


/*!
 * Struct containing all parameters for the battery
 */
struct battParam_struct{
	BATTPARAM_blockParameter_t manufacturerName[SBSDATA_BLOCK_LENGTH];
	BATTPARAM_blockParameter_t deviceName[SBSDATA_BLOCK_LENGTH];
	BATTPARAM_blockParameter_t deviceChemistry[SBSDATA_BLOCK_LENGTH];
	BATTPARAM_blockParameter_t manufacturerData[SBSDATA_BLOCK_LENGTH];
	BATTPARAM_wordParameter_t batteryMode;
	BATTPARAM_wordParameter_t chargingCurrent;
	BATTPARAM_wordParameter_t chargingVoltage;
	BATTPARAM_wordParameter_t designCapacity;
	BATTPARAM_wordParameter_t fullChargeCapacity;
	BATTPARAM_wordParameter_t designVoltage;
	BATTPARAM_wordParameter_t specificationInfo;
	BATTPARAM_wordParameter_t manufactureDate;
	BATTPARAM_wordParameter_t serialNumber;
	BATTPARAM_wordParameter_t serialNumber1;
	BATTPARAM_wordParameter_t serialNumber2;
	BATTPARAM_wordParameter_t serialNumber3;
	BATTPARAM_wordParameter_t cellsInSeries;
	BATTPARAM_wordParameter_t cellMinVoltage;
	BATTPARAM_wordParameter_t cellMaxVoltage;
	BATTPARAM_wordParameter_t cellPoweroffVoltage;
	BATTPARAM_wordParameter_t cellDuvrDisableVoltage;
	BATTPARAM_wordParameter_t fullChargeVoltage;
	BATTPARAM_wordParameter_t fullChargeCurrent;
	BATTPARAM_wordParameter_t misbalanceVoltageThreshold;
	BATTPARAM_wordParameter_t cellMinDischargeTemperature;
	BATTPARAM_wordParameter_t cellMaxDischargeTemperature;
	BATTPARAM_wordParameter_t cellMinChargeTemperature;
	BATTPARAM_wordParameter_t cellMaxChargeTemperature;
	BATTPARAM_wordParameter_t shortcircuitReactionTime;
	BATTPARAM_wordParameter_t shortcircuitCurrent;
	BATTPARAM_wordParameter_t overcurrentReactionTime;
	BATTPARAM_wordParameter_t overcurrentCharge;
	BATTPARAM_wordParameter_t overcurrentDischarge;
	BATTPARAM_wordParameter_t highcurrentReactionTime;
	BATTPARAM_wordParameter_t highcurrentCharge;
	BATTPARAM_wordParameter_t highcurrentDischarge;
	BATTPARAM_wordParameter_t shuntResistance;
	BATTPARAM_wordParameter_t remainingCapacityAlarm;
	BATTPARAM_wordParameter_t remainingTimeAlarm;
	BATTPARAM_wordParameter_t terminateDischargeLimit;
	BATTPARAM_wordParameter_t activeCurrentThreshold;
	BATTPARAM_wordParameter_t openCircuitRelaxationTime;
	BATTPARAM_wordParameter_t crcChecksum;
	
};
extern struct EEMEM battParam_struct battParams;

/******************************************************************************
 Function prototypes
******************************************************************************/
void BATTPARAM_GetString( BATTPARAM_blockParameter_t * source, SBS_command_t* destination );

uint16_t BATTPARAM_GetCycleCount();
void BATTPARAM_IncreaseCycleCount();

bool BATTPARAM_CheckCRC();

void BATTPARAM_InitSramParameters();

#endif //BATT_PACK_PARAM
