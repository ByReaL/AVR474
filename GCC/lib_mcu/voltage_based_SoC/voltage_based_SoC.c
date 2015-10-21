/* This file has been prepared for Doxygen automatic documentation generation.*/
/*! \file *********************************************************************
 *
 * \brief
 *      Functions for estimating State of Charge based on voltage
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

/* Includes. */
#include "common.h"
#include "vadc.h"
#include "ccadc.h"
#include "voltage_based_SoC.h"
#include "rtc.h"
#include "battery_current_monitoring.h"
#include <avr/pgmspace.h>


/* Definitions for the voltage */
#define VOLTAGE_EMPTY		(VoltageTable[0][VOLTAGE_TABLE_SIZE-1])
#define VOLTAGE_FULL		(VoltageTable[0][0])

uint16_t ConvertVoltageToSoC(uint16_t inputVoltage);
int16_t ConvertDigitalToCurrent(void);
int16_t ConvertDigitalToVoltage(void);

/* The voltage table contains mapping between voltage and SoC.
*  The values represent the open circuit voltage of the cell. These values
*  should be measured by discharging the cell at a low rate <0.05C or by
*  discharge and charge at a specific rate and then use the mean voltage of
*  the measurements.
*  The SoC values are ranged from 0 - 1000 where 0 represents 0% SoC and 1000
*  represents 100% SoC.
*/
// Array for TrustFire Cell
//#define VOLTAGE_TABLE_SIZE	12
//uint16_t VoltageTable[2][VOLTAGE_TABLE_SIZE] = { {4000,3940,3848,3779,3756,3729,3672,3639,3575,3502,3313,3000}, {1000,900,700,500,400,300,150,100,70,50,20,0} };

// Array for TrustFire Cell att 10mA load from keithley instrument
#define VOLTAGE_TABLE_SIZE 21
const uint16_t VoltageTable[2][VOLTAGE_TABLE_SIZE] PROGMEM = { 
	{4180,4112,4074,4013,3973,3922,3858,3810,3782,3762,3751,3725,3701,3684,3678,3663,3607,3511,3350,3200,2771 },
	{1000,950 ,900 ,850 ,800 ,700 ,600 ,500 ,400 ,300 ,250 ,200 ,150 ,120 ,100 ,80  ,60  ,40  ,20  ,10  ,0 } };

// Array for Dell Cell
//uint16_t VoltageTable[2][VOLTAGE_TABLE_SIZE] = { {4100,3923,3755,3718,3696,3664,3638,3596,3530,3423,3232,2950}, {1000,850,600,500,400,300,250,200,150,100,50,0} };

/* This varaible contains the most recent SoC that is based on open circuit voltage */
uint16_t OpenCircuitSoC;

/* This variable contains the time in seconds since last update of SoC.
 *  (The SoC is only updated when the voltage is relaxed).
 * It will stop at 65534 and not overflow
 */
uint16_t LastUpdated;

/* At which minute there have been battParam.openCircuitRelaxationTime minutes since the last active current */
uint8_t relaxationTimer;

/* Functions. */


/*! \brief Calculate the initial SoC
 *
 * This function updates both the estimated and open circuit SoC, run when sure
 * the voltage is a open circuit voltage (for example, at startup after being shut down
 * for a couple of days)
 *
 */
void VBSoC_Init(void)
{
	uint16_t lowestCellVoltage;
	
	// See which cell has the lowest voltage and use that in the calculations 
#if BATTPARAM_CELLS_IN_SERIES == 2
	{
		uint16_t voltageCell1 = VADC_readCellVoltage(CELL1);
		uint16_t voltageCell2 = VADC_readCellVoltage(CELL2);
		lowestCellVoltage = (voltageCell1 < voltageCell2) ? voltageCell1 : voltageCell2;
	}
#elif BATTPARAM_CELLS_IN_SERIES == 3
	{
		uint16_t voltageCell1 = VADC_readCellVoltage(CELL1);
		uint16_t voltageCell2 = VADC_readCellVoltage(CELL2);
		uint16_t voltageCell3 = VADC_readCellVoltage(CELL3);
		lowestCellVoltage = (voltageCell1 < voltageCell2)      ? voltageCell1 : voltageCell2;
		lowestCellVoltage = (lowestCellVoltage < voltageCell3) ? lowestCellVoltage : voltageCell3;
	}
#elif BATTPARAM_CELLS_IN_SERIES == 4
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
	
	OpenCircuitSoC = ConvertVoltageToSoC(lowestCellVoltage);
}


/*! \brief Updates the SoC
 *
 * Should be run once a second
 *
 * Updates open circuit SoC if have been in relaxation for battParams.openCircuitRelaxationTime
 *
 * \return Returns true if openCircuitSoC was updated
 */
bool VBSoC_Update(void)
{
	// Don't want LastUpdated to overflow back to zero, but that it stays at 
	// 65534 seconds doesn't matter.
	// It shouldn't really be this long between LastUpdated is reset, but better
	// to be safe.
	if(LastUpdated < 65535) {
		++LastUpdated;
	}
	
	// Check if it's time to update the open circuit SoC
	if(RTC_GetMinutes() == relaxationTimer) {
		// Only update if it was more than 30 seconds ago last time
		// TODO: maybe a define, but the actual value is not that important, its just to avoid 
		//       doing the calculations every second. And since the current is low (or else 
		//       it wouldn't get here) the state of charge changes very slowly
		if(LastUpdated >= 30) {
			// See which cell has the lowest voltage and use that in the calculations 
			uint16_t lowestCellVoltage;
			
#if BATTPARAM_CELLS_IN_SERIES == 2
			{
				uint16_t voltageCell1 = VADC_readCellVoltage(CELL1);
				uint16_t voltageCell2 = VADC_readCellVoltage(CELL2);
				lowestCellVoltage = (voltageCell1 < voltageCell2) ? voltageCell1 : voltageCell2;
			}
#elif BATTPARAM_CELLS_IN_SERIES == 3
			{
				uint16_t voltageCell1 = VADC_readCellVoltage(CELL1);
				uint16_t voltageCell2 = VADC_readCellVoltage(CELL2);
				uint16_t voltageCell3 = VADC_readCellVoltage(CELL3);
				lowestCellVoltage = (voltageCell1 < voltageCell2)      ? voltageCell1 : voltageCell2;
				lowestCellVoltage = (lowestCellVoltage < voltageCell3) ? lowestCellVoltage : voltageCell3;
			}
#elif BATTPARAM_CELLS_IN_SERIES == 4
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
			
			OpenCircuitSoC = ConvertVoltageToSoC(lowestCellVoltage);
			LastUpdated = 0;
			return true;
		}
	}
	return false;
}


/**********************************************************/
/*! \brief Reset the relaxation timer
 *
 * Run this function when device not is in standby mode
 *
 */	
void VBSoC_ResetTimer(void) {
	relaxationTimer = RTC_GetMinutes() + battParams.openCircuitRelaxationTime;
}

/*********************************************************************************/
/*! \brief Returns how many seconds ago the open circuit voltage SoC was updated
 *
 * \return Seconds since last time OCV SoC was updated
 */
uint16_t VBSoC_OCSoCLastUpdated(void) {
	return LastUpdated;
}

/********************************************************************************/
/*! \brief Returns the the latest open circuit state of charge
 *
 * Check how old this is with the VBSoC_OCSoCLastUpdated function
 */
uint16_t VBSoC_OCSoC(void) {
	return OpenCircuitSoC;
}

/********************************************************************************/
/*! \brief Returns the the latest estimated state of charge
 *
 */
uint16_t VBSoC_EstimatedSoC(void) {
	int16_t result_volt;
	uint16_t lowestCellVoltage;
	
	// See which cell has the lowest voltage and use that in the calculations 
#if BATTPARAM_CELLS_IN_SERIES == 2
	{
		uint16_t voltageCell1 = VADC_readCellVoltage(CELL1);
		uint16_t voltageCell2 = VADC_readCellVoltage(CELL2);
		lowestCellVoltage = (voltageCell1 < voltageCell2) ? voltageCell1 : voltageCell2;
	}
#elif BATTPARAM_CELLS_IN_SERIES == 3
	{
		uint16_t voltageCell1 = VADC_readCellVoltage(CELL1);
		uint16_t voltageCell2 = VADC_readCellVoltage(CELL2);
		uint16_t voltageCell3 = VADC_readCellVoltage(CELL3);
		lowestCellVoltage = (voltageCell1 < voltageCell2)      ? voltageCell1 : voltageCell2;
		lowestCellVoltage = (lowestCellVoltage < voltageCell3) ? lowestCellVoltage : voltageCell3;
	}
#elif BATTPARAM_CELLS_IN_SERIES == 4
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
	
	// Calculate the voltage drop from current (is negative while discharging, and positive while charging)
	result_volt = ConvertDigitalToVoltage();

	// Estimate the SoC bases on cell voltage and voltage drop
	return ConvertVoltageToSoC(lowestCellVoltage - result_volt );
}
/**********************************************************/
/*! \brief SoC conversion.
 *
 *  Conversion of voltage to SoC by using conversion table.
 *
 * \return Soc (0 to 100)
 */
uint16_t ConvertVoltageToSoC(uint16_t inputVoltage)
{
	uint16_t socval = 0;
	uint8_t i = 1;
      
	// Check if voltage is above treshold.
	if(inputVoltage >= VOLTAGE_FULL)
	{
		// Set SoC to 100%.
		socval = 100;
	}
	// Check if voltage is below treshold.
	else if(inputVoltage <= VOLTAGE_EMPTY)
	{
		// Set SoC to 0%.
		socval = 0;
	}
	// Look up interval in table and interpolate
	else
	{
		// Find the actual interval in the voltage-SoC table and interpolate.
		while((i<VOLTAGE_TABLE_SIZE) && (socval==0))
		{
			if(VoltageTable[0][i] <= inputVoltage)
			{
				socval = VoltageTable[1][i] + (inputVoltage - VoltageTable[0][i]) * ( VoltageTable[1][i-1] - VoltageTable[1][i] ) / (VoltageTable[0][i-1] - VoltageTable[0][i]);
				socval /= 10; // To get percent instead of 0.1percent
			}

			i++;
		}
	}

	return socval;
}

/**********************************************************/
/*! \brief Current conversion.
 *
 *  Conversion of digital value to current (in mA).
 *
 * \return Current (mA)
 */
int16_t ConvertDigitalToCurrent(void)
{       
	return BATTCUR_Ticks2mA(BATTCUR_GetOffsetCalibratedCurrent());
}

/**********************************************************/
/*! \brief Voltage conversion.
 *
 *  Conversion of digital value to voltage on battery internal resistor (mV).
 *
 * \return current (mA)
 */
int16_t ConvertDigitalToVoltage(void)
{
	int32_t temp;
	
	temp = ConvertDigitalToCurrent();
	temp *= INTERNAL_BATT_RESISTOR_TIMES_128;
	
	if(temp < 0) {
		temp = -temp;
		temp += 64; // rounding
		temp >>= 7; // divide by 128
		temp = -temp;
	} else {
		temp += 64; // rounding
		temp >>= 7; // divide by 128
	}

	return ((int16_t) temp);
}

/**********************************************************/
/*! \brief SoC-Voltage conversion.
 *
 *  Conversion of SoC to voltage by using conversion table.
 *
 * \return Voltage(OCV)in mV
 */
 
uint16_t VBSoC_ConvertSoCToVoltage(uint16_t inputsoc)
{
  
        uint16_t volval = 0;
	uint8_t i = 1;
      
	// Check if soc is above 1000.
	if(inputsoc >= 1000){
	
	        // Set voltage to full voltage.
		volval = VOLTAGE_FULL;
		
	// Check if soc is below 0.
	}else if(inputsoc <= 0){
	
		// Set voltage to empty voltage.
		volval = VOLTAGE_EMPTY;
		
	// Look up interval in table and interpolate
	}else{
		// Find the actual interval in the voltage-SoC table and interpolate.
		while((i<VOLTAGE_TABLE_SIZE) && (volval==0))
		{
			if(VoltageTable[1][i] <= inputsoc)
			{
				//socval = VoltageTable[1][i] + (inputVoltage - VoltageTable[0][i]) * ( VoltageTable[1][i-1] - VoltageTable[1][i] ) / (VoltageTable[0][i-1] - VoltageTable[0][i]);
				volval = VoltageTable[0][i] + (inputsoc - VoltageTable[1][i]) * ( VoltageTable[0][i-1] - VoltageTable[0][i] ) / (VoltageTable[1][i-1] - VoltageTable[1][i]);
				
			}

			i++;
		}
	}

	return volval;
}
