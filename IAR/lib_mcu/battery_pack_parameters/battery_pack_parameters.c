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
 * $Revision: 5667 $
 * $Date: 2009-05-27 17:00:07 +0800 (Wed, 27 May 2009) $  \n
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


#include "common.h"
#include "battery_pack_parameters.h"
#include "battery_current_monitoring.h"
#include "battery_protection.h"
#include "cc_gas_gauging.h"
#include "gas_gauging.h"
#include "rtc.h"


/******************************************************************************
 File-scope parameters
******************************************************************************/
//! Cycle count, saved in eeprom
__eeprom uint16_t cycleCount @ EEPROM_CYCLE_COUNT = 0;

/*****************************************************************************
 Variables extern'ed in header file
 *****************************************************************************/
struct battParam_sramstruct battParams_sram;

// When those are changed, make sure to update REMAINING_CAPACITY_TABLE_SIZE
// in battery_pack_parameters.h
//! At which currents the remainingCapacity calibration values are saved
__flash uint16_t remainingCapacityCalibrationCurrents[] = {0, 163, 325, 650, 1000};
//! RemainingCapacity calibration for the different current
// TODO: Different temperatures and it should be updated at runtime
__flash uint8_t  remainingCapacityCalibration[] = {0,  4,  8,  12,  25};

/******************************************************************************
 Functions declarations
******************************************************************************/
/******************************************************************************/
/*! \brief Copy a SBS block
 *
 * Copies a block the length of one block command from eeprom to destination
 *
 *	\param source Is storage (pointer) that the string is read from.
 *	\param destination  Is storage (pointer) that the string is written to.
 */
void BATTPARAM_GetString( BATTPARAM_blockParameter_t * source, SBS_command_t* destination )
{
	for(uint8_t i = 0; i < SBSDATA_BLOCK_LENGTH; ++i) {
		destination->payload[i] = *((uint8_t __eeprom*)source + i);
	}
}

/*! \brief Return the number of cycles
 *
 * Returns the number of discharge/charge cycles
 */
uint16_t BATTPARAM_GetCycleCount() {
	return cycleCount;
}

/*! \brief Increase the cycle count by one
 *
 */
void BATTPARAM_IncreaseCycleCount() {
	++cycleCount;
}


/*! \brief Checks that the saved checksum in the battParams struct is correct
 *
 * \note TODO: this requires manufacturerName to be the first in the struct, as i couldn't cast the struct to a uint8_t
 *             And checksum must be last
 *
 * \retval  true  CRC was correct
 * \retval  false CRC was not correct
 */
bool BATTPARAM_CheckCRC() {
	uint8_t __eeprom *paramByte = battParams.manufacturerName;
	uint8_t __eeprom *endByte = (uint8_t __eeprom*)&battParams.crcChecksum;
	uint16_t crc = 0xFFFF; // Normal start value for CRC-CCITT
	
	for( ; paramByte != endByte; ++paramByte) {
		crc = crc_ccitt_update(crc, *paramByte);
	}
	
	return (crc == battParams.crcChecksum);
}

/*********************************************/
/*! Calculate sram battery parameters on init
 *
 */
void BATTPARAM_InitSramParameters() {
	// Active current threshold in ticks
	battParams_sram.activeCurrentThresholdInTicks = (uint16_t) BATTCUR_mA2Ticks(battParams.activeCurrentThreshold);
	
	// Full charge capacity in CC accumulated
	battParams_sram.capacityInCCAccumulated = (uint32_t) CCGASG_mAh2Acc(battParams.fullChargeCapacity);
	
	// Terminate discharge limit in CC accumulated
	battParams_sram.terminateDischargeLimit = CCGASG_mAh2Acc(battParams.terminateDischargeLimit);
	
	// Values for remaining capacity calibration
	GASG_CalculateRemainingCapacityValues();
}
