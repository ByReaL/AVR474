/* This file has been prepared for Doxygen automatic documentation generation.*/
/*! \file *********************************************************************
 *
 * \brief
 *      Headerfile for sbs_commands.c.
 *
 *      Contains defines, typedefinitions and functions prototypes for sb_data.c.
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
 * $Revision: 5627 $
 * $Date: 2009-05-15 14:49:21 +0800 (Fri, 15 May 2009) $  \n
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


#ifndef SBS_COMMANDS_H
#define SBS_COMMANDS_H

#include "common.h"

/******************************************************************************
Defines
******************************************************************************/
#define SBSDATA_BLOCK_LENGTH	32  //!< Defines the default length of any string in the smart battery data set

/******************************************************************************
Type definitions
******************************************************************************/
// ! \brief All defined SBS commands (note that OptionalmfgFunctions are renamed).
typedef enum SBS_commands_enum{
	SBSCMD_manufacturerAccess = 0x00,
	SBSCMD_remainingCapacityAlarm = 0x01,
	SBSCMD_remainingTimeAlarm = 0x02,
	SBSCMD_batteryMode = 0x03,
	SBSCMD_atRate = 0x04,
	SBSCMD_atRateTimeToFull = 0x05,
	SBSCMD_atRateTimeToEmpty = 0x06,
	SBSCMD_atRateOK = 0x07,
	SBSCMD_temperature = 0x08,
	SBSCMD_voltage = 0x09,
	SBSCMD_current = 0x0A,
	SBSCMD_averageCurrent = 0x0B,
	SBSCMD_maxError = 0x0C,
	SBSCMD_relativeStateOfCharge = 0x0D,
	SBSCMD_absoluteStateOfCharge = 0x0E,
	SBSCMD_remainingCapacity = 0x0F,
	SBSCMD_fullChargeCapacity = 0x10,
	SBSCMD_runTimeToEmpty = 0x11,
	SBSCMD_averageTimeToEmpty = 0x12,
	SBSCMD_averageTimeToFull = 0x13,
	SBSCMD_chargingCurrent = 0x14,
	SBSCMD_chargingVoltage = 0x15,
	SBSCMD_batteryStatus = 0x16,
	SBSCMD_cycleCount = 0x17,
	SBSCMD_designCapacity = 0x18,
	SBSCMD_designVoltage = 0x19,
	SBSCMD_specificationInfo = 0x1A,
	SBSCMD_manufactureDate = 0x1B,
	SBSCMD_serialNumber = 0x1C,
	//reserved = 0x1D-0x1F,
	SBSCMD_manufacturerName = 0x20,
	SBSCMD_deviceName = 0x21,
	SBSCMD_deviceChemistry = 0x22,
	SBSCMD_manufacturerData = 0x23,
	SBSCMD_authentication = 0x24,		// Extended string command
	//reserved = 0x25-0x29,
	SBSCMD_shuntCalibration = 0x2A,   // Reserved command. Set/read shunt resistor value.
	SBSCMD_FETdisable      = 0x2B,		// Extended command. Force disabling of FETs
	SBSCMD_storageMode     = 0x2C,		// Extended command. Enter storage mode (power-off)
	SBSCMD_temperatureNTC2 = 0x2D,		// Extended command. Read temperature on NTC 2
	SBSCMD_temperatureNTC1 = 0x2E,		// Extended command. Read temperature on NTC 1
	SBSCMD_OptionalMfgFunction5 = 0x2F,
	// reserved = 0x30-0x3b,
	SBSCMD_voltageCell4 = 0x3C,	// Commonly referred to as OptionalMfgFunction4, read only
	SBSCMD_voltageCell3 = 0x3D,	// Commonly referred to as OptionalMfgFunction3, read only
	SBSCMD_voltageCell2 = 0x3E,	// Commonly referred to as OptionalMfgFunction2, read only
	SBSCMD_voltageCell1 = 0x3F,	// Commonly referred to as OptionalMfgFunction1, read only
	SBSCMD_LAST_STOP = 0x40
	} SBS_commands_t;

#define SBSDATA_NUMBER_OF_SBS_COMMANDS (SBSCMD_LAST_STOP)  //!< Number of SBS commands.

// ! \brief Type used for SBS commands.
typedef uint16_t SBS_commandData_t;
typedef union SBS_command_union{
	uint8_t entire[SBSDATA_BLOCK_LENGTH+1];
	struct{
		SBS_commands_t command;
		union{
			uint16_t payloadW;
			uint8_t payload[SBSDATA_BLOCK_LENGTH];
		};
	};
} SBS_command_t;

/******************************************************************************
Proto types
******************************************************************************/
bool SBS_IsCommandStaticStringType( SBS_commands_t cmd );
bool SBS_IsCommandBlockDataType( SBS_commands_t cmd);
bool SBS_IsCommandReserved( SBS_commands_t cmd );
bool SBS_IsCommandExtended( SBS_commands_t cmd );

#endif //SBS_COMMANDS_H
