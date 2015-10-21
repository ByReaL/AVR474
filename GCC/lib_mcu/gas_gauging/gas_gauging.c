/* This file has been prepared for Doxygen automatic documentation generation.*/
/*! \file *********************************************************************
 *
 * \brief
 *      Gas gauging module for calculating time to empty, time to full etc.
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
#include "cc_gas_gauging.h"
#include "battery_current_monitoring.h"
#include "gas_gauging.h"


uint32_t minutesLeft; //!< Calculation variable to avoid using cstack

/******************************************************************************/
/*! \brief Estimate time to full.
 *
 *  Estimate how long until the battery is full based on specified current.
 *
 *  The estimate is not accurate at all, because the current will decrease at
 *  the end of the charge. Shouldn't be to hard to characterize though.
 *
 *	\param current Current in mA.
 *
 *	\return Time to full at given current in minutes.
 */
uint16_t GASG_TimeToFull(int16_t current)
{
	if(current <= 0) {
		return 65535; // Cannot get full with zero or negative rate...
	} else {
		int16_t remainingCapacity = CCGASG_Acc2mAh(GASG_RemainingCapacity(BATTCUR_mA2Ticks(current)));
		if(battParams.fullChargeCapacity <= remainingCapacity) {
			return 0; // If fully loaded, return zero minutes to full
		} else {
			// Capacity to load in mAminutes
			uint32_t capacityToLoad = 60*(uint32_t)(battParams.fullChargeCapacity - remainingCapacity);
			// Current is always positive here, so unsigned minutes is okey
			minutesLeft = capacityToLoad/current;
			if(minutesLeft > 65534) {
				minutesLeft = 65534;
			}
			return (uint16_t)minutesLeft;
		}
	}
}


/******************************************************************************/
/*! \brief Estimate time to empty.
 *
 *  Estimate how long until the battery is empty based on specified current.
 *
 *	\param current Current in mA.
 *
 *	\return Time to full at given current in minutes. 65536 if invalid.
 */
uint16_t GASG_TimeToEmpty(int16_t current)
{
	if(current >= 0) {
		return 65535; // Cannot get empty with zero or positive rate...
	} else {
		int16_t remainingCapacity = CCGASG_Acc2mAh(GASG_RemainingCapacity(BATTCUR_mA2Ticks(current)));
		if(remainingCapacity <= 0) {
			return 0; // If fully discharged, return zero minutes to empty
		} else {
			// Capacity to empty in mAminutes
			uint32_t capacityToEmpty = 60*(uint32_t)remainingCapacity;
			current = -current; // current is always negative before this
			minutesLeft = capacityToEmpty/current;
			if(minutesLeft > 65534) {
				minutesLeft = 65534;
			}
			return (uint16_t)minutesLeft;
		}
	}
}


/*********************************************************************************/
/*! \brief Check if battery can support specified discharge current for 10seconds.
 *
 *	\param current In mAh
 *
 *	\retval  true  The battery can deliver that current for 10 seconds
 *  \retval false  The battery probably can not deliver that current for 10 seconds
 */
bool GASG_AtRateOK(int16_t current)
{
	if(current >= 0) {
		return true; // Can always support zero discharge or charging
	} else {
		int16_t currentBattery = BATTCUR_Ticks2mA(BATTCUR_GetCurrent());
		if(currentBattery >= current) {
			return true; // If charging with more than or equal AtRate, we can support this
		} else {
			int16_t remainingCapacity = CCGASG_Acc2mAh(CCGASG_ReadAccumulatedCC());
			if(remainingCapacity <= 0) {
				return false; // If fully discharged, cannot support discharge
			} else {
				// TODO: Need to draw high currents to empty even a few mAh from the battery, but the voltage drop
				//       may be quite significant, so calculate that.
				current = -current; // make it positive
				uint16_t voltage_drop = ((uint32_t)current*INTERNAL_BATT_RESISTOR_TIMES_128) >> 7; // Divide by 128, safe because current is always positive
				// See which cell has the lowest voltage and use that in the calculations
				uint16_t lowestCellVoltage;

#if BATTPARAM_CELLS_IN_SERIES == 2
				{
					uint16_t voltageCell1 = VADC_readCellVoltage(CELL1);
					uint16_t voltageCell2 = VADC_readCellVoltage(CELL2);
					lowestCellVoltage = (voltageCell1 < voltageCell2) ? voltageCell1 : voltageCell2;
				}
#endif
#if BATTPARAM_CELLS_IN_SERIES == 3
				{
					uint16_t voltageCell1 = VADC_readCellVoltage(CELL1);
					uint16_t voltageCell2 = VADC_readCellVoltage(CELL2);
					uint16_t voltageCell3 = VADC_readCellVoltage(CELL3);
					lowestCellVoltage = (voltageCell1 < voltageCell2)      ? voltageCell1 : voltageCell2;
					lowestCellVoltage = (lowestCellVoltage < voltageCell3) ? lowestCellVoltage : voltageCell3;
				}
#endif
#if BATTPARAM_CELLS_IN_SERIES == 4
				{
					uint16_t voltageCell1 = VADC_readCellVoltage(CELL1);
					uint16_t voltageCell2 = VADC_readCellVoltage(CELL2);
					uint16_t voltageCell3 = VADC_readCellVoltage(CELL3);
					uint16_t voltageCell4 = VADC_readCellVoltage(CELL4);
					lowestCellVoltage = (voltageCell1 < voltageCell2)      ? voltageCell1 : voltageCell2;
					lowestCellVoltage = (lowestCellVoltage < voltageCell3) ? lowestCellVoltage : voltageCell3;
					lowestCellVoltage = (lowestCellVoltage < voltageCell4) ? lowestCellVoltage : voltageCell4;
				}
#endif
				if(lowestCellVoltage - voltage_drop < battParams.cellMinVoltage) {
					return false;
				} else {
					return true;
				}
			}
		}
	}
}

/*******************************************/
/*! \brief Returns state of charge
 *
 * Defined as RemainingCapacity/maxCharge with remainingCapacity at the
 * specified current
 *
 * for relativeStateOfCharge, maxCharge would be fullChargeCapacity
 * and for absolutStateOfCharge, maxCharge would be designCapacity
 *
 * \param   current State of charge at this current
 * \param maxCharge Maximum value for remainingCapacity in CCGASG ticks
 *
 * \return State of charge in %
 */
uint16_t GASG_StateOfCharge(int32_t current, uint32_t maxCharge)
{
	int32_t remainingCapacity = GASG_RemainingCapacity(current);
	if(remainingCapacity <= 0) {
		return 0;
	}
	
	maxCharge /= 100;
	
	// Can return more than 100%
	return (uint16_t)(remainingCapacity / maxCharge);
}


/**************************************************************/
/*! \brief Calculate the remaining capacity calibration values
 *
 * Since the values are saved as mA/mAh in eeprom they are recalculated to
 * ccadc_ticks/CCGASG_acc and saved in sram
 */
void GASG_CalculateRemainingCapacityValues()
{
	for(uint8_t i = 0; i < REMAINING_CAPACITY_TABLE_SIZE; ++i) {
		battParams_sram.remainingCapacityCalibrationCurrents[i] = (uint16_t) ( BATTCUR_mA2Ticks(remainingCapacityCalibrationCurrents[i]) );
		battParams_sram.remainingCapacityCalibration[i] = (uint16_t) ( CCGASG_mAh2Acc( remainingCapacityCalibration[i] ) >> 8 );
	}
}

/************************************************************/
/*! \brief Returns the remaining capacity at specified current
 *
 * When charging this will return the remainingCapacity for standby current
 * When discharging, it will return the remainingCapacity for the specified current
 *
 * \note There is a high risk of overflowing when calling this function with
 *       high (above 2A estimate) and when the difference between higher and lower
 *       capacity is high.
 *
 * \param  current  Current to calculate remaining capacity for. In ticks.
 *
 * \return Remaining capacity in CCGASG acc at the specified current
 */
int32_t GASG_RemainingCapacity(int32_t current)
{
	uint32_t diff;
	
	// If charging return remaining capacity for standby (ie, total remaining capacity)
	if(current >= 0) {
		diff = 0;

	} else {
		// Make current positive because calculations require that
		current = -current;
		
		uint16_t lowerCurrent, higherCurrent;
		uint16_t lowerCapacity, higherCapacity;
		
		// Find the two stored current values around the specified current
		uint8_t i = 1;
		while(battParams_sram.remainingCapacityCalibrationCurrents[i] <= current && i < REMAINING_CAPACITY_TABLE_SIZE - 1) {
			++i;
		}
		lowerCurrent   = battParams_sram.remainingCapacityCalibrationCurrents[i-1];
		higherCurrent  = battParams_sram.remainingCapacityCalibrationCurrents[i];
		// Note that capacity values are scaled down 8 bits in storage
		lowerCapacity  = battParams_sram.remainingCapacityCalibration[i-1];
		higherCapacity = battParams_sram.remainingCapacityCalibration[i];
		
		// Since higherCapacity and lowerCapacity is scaled down 8 bits, that have to be
		// scaled up to get corrent result. But do it after division to avoid overflow
		// TODO: this isn't very accurate, but the value is used to present to the user
		//       so it will probably by converted to mAh (or minutes left) and then this
		//       inaccuracy probably won't be a big deal.
		diff = ((uint32_t)lowerCapacity << 8) + ((uint32_t)(((higherCapacity - lowerCapacity)*(current - lowerCurrent)) / (higherCurrent - lowerCurrent)) << 8);
	}
		
	return CCGASG_ReadAccumulatedCC() - diff;
}

