/* This file has been prepared for Doxygen automatic documentation generation.*/
/*! \file *********************************************************************
 *
 * \brief
 *      CC-ADC result processing firmware module.
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

/******************************************************************************
 Included files
******************************************************************************/
#include "common.h"
#include "battery_pack_parameters.h"
#include "battery_current_monitoring.h"
#include "ccadc.h"

/******************************************************************************
 Global (file scope) variable
******************************************************************************/
int32_t lastCurrent;
int32_t lastOffsetCalibratedCurrent;
bool discharge;
int32_t averageCurrent; //<! Is scaled up by 2^AVERAGE_CURRENT_SCALING

uint16_t CCADC2mACoefficient; // Used in this formula to get mA: (ccadc_ticks*CCADC2mACoefficient)/2^16 = mA

/******************************************************************************
 Function declarations.
******************************************************************************/

/******************************************************************************/
/*! \brief Stores the current from the last CC-ADC reading ( mV, 18-bit signed).
 *
 *	\param current The current flowing in/out of the battery pack (represented
 * as mV accros the shunt).
 *
 */
void BATTCUR_StoreCurrent( int32_t current )
{
	lastCurrent = current;
	lastOffsetCalibratedCurrent = current + CCADC_GetAccOffset();
	if( 0 >= lastOffsetCalibratedCurrent) {
		discharge = true;
	} else {
		discharge = false;
	}
}

/******************************************************************************/
/*! \brief Provides the current from the last CC-ADC reading (18-bit signed).
 *
 *	\return The current flowing in/out of the battery pack (represented
 * as mV accros the shunt).
 */
int32_t BATTCUR_GetCurrent( void )
{
	return lastCurrent;
}


/******************************************************************************/
/*! \brief Provides the offset calibrated current from the last CC-ADC reading.
 *
 *	\return The current flowing in/out of the battery pack (represented
 * as mV accros the shunt).
 */
int32_t BATTCUR_GetOffsetCalibratedCurrent( void )
{
	return lastOffsetCalibratedCurrent;
}

/******************************************************************************/
/*! \brief Calculates and stores the average current during the last minut (18-bit signed).
 *
 * Assumes that the time base for the sampling is app 1 second. The average is
 * calculated using an "exponential filter" and is an estimate for the average current.
 * This method instead of a real average to reduce memory consumption.
 *
 *	\param current The current flowing in/out of the battery pack (represented
 * as mV accros the shunt).
 *
 */
void BATTCUR_UpdateAverageCurrent( int32_t current )
{
	// An exponential filter is used to
	// average = 31/32 * old value + 1/32 * new value.
	// But to remove the truncation error, the current is scaled up
	averageCurrent *= 31;
	averageCurrent += current * (1<<AVERAGE_CURRENT_SCALING);
	
	// Right-shifting works nice for dividing positive numbers, and oppositing if negative is
	// a lot faster than doing a division
	if(averageCurrent > 0) {
		// Add half divisor to get proper mathematical rounding
		averageCurrent += 16;
		averageCurrent >>= 5; // /32
	} else if (averageCurrent < 0) {
		averageCurrent = -averageCurrent;
		// Add half divisor to get proper mathematical rounding
		averageCurrent += 16;
		averageCurrent >>= 5; // /32
		averageCurrent = -averageCurrent;
	}
}
/******************************************************************************/
/*! \brief Provide an estimate for the average current throughout the last minute (18-bit signed).
 *
 *	\return The average current flowing in/out of the battery pack (represented
 * as mV accros the shunt).
 */
int32_t BATTCUR_GetAverageCurrent( void )
{
	if(averageCurrent < 0) {
		// TODO: perhaps round this division, but the value is not used for anything important so skip that for now
		return -(-averageCurrent >> AVERAGE_CURRENT_SCALING);
	} else {
		return (averageCurrent >> AVERAGE_CURRENT_SCALING);
	}
}


/******************************************************************************/
/*! \brief Is the current below the limit that defines standby operation
 * (time to enter low power mode?).
 *
 *	\param current  The current to check if below standby operation limit (in CCADC ticks)
 *
 *	\return Returns true if the current is below the "low" limit.
 */
bool BATTCUR_IsCurrentLow( int32_t current )
{
	if(0 > current) {
		current = -current;
	}
	
	return ( (int32_t)battParams_sram.activeCurrentThresholdInTicks > current );
}

/******************************************************************************/
/*! \brief Init the average current counter
 *
 *	\param current First current
 *
 */
void BATTCUR_InitializeAverageCurrent( int32_t current )
{
	averageCurrent = current * (1<<AVERAGE_CURRENT_SCALING);
}

/******************************************************************************/
/*! \brief Is the current charging or discharging the battery.
 *
 *  Uses the offset calibrated current
 *
 *	\return Returns true if the current flow is in the discharging direction.
 */
bool BATTCUR_IsDischargeOngoing( void )
{
	return discharge;
}

/******************************************************************************/
/*! \brief Calculate the Shunt Coefficient for CCADC_Value to mA conversion
 *
 * The goal for this function is to find a coefficient that can be used in the
 * following formula: CCADC_Value*coefficient = mA
 * But since coefficient is a very small number, it is scaled up by 2^16, so the formula
 * is: (CCADC_Value*coefficient)/2^16 = mA
 *
 * Formula for Shunt Coefficient for CCADC_Value to mA conversion
 *            CCADC2mACoefficient = I(mA)*10^6/CCADC_Value
 *                               = ref(mV)*10^6*2^16/(shunt(uohm)*2^17)
 *
 *      ref(mV): the internal reference voltage of CCADC with the unit "mV"
 *      shunt(uohm): the current sensor resistance with the unit "microOhm"
 *      10^6: compensation for using the shunt with "microOhm"
 *      2^17: the CCADC measurement result range
 *      2^16: up-scaling to increase accuracy
 *
 *
 * \param  shuntResistance  Shunt resistance in microOhm
 * \param  VrefSel          Selects the voltage reference to use
 *
 */
void BATTCUR_CalculateShuntCoeffient(uint16_t shuntResistance,uint8_t VrefSel)
{
	
        // CCADC2mACoefficient = I(mA)*10^6/tick
        //                    = ref(mV)*10^6*2^16/(shunt(uohm)*2^17)
        // define: dividend = ref(mV)*10^6*2^16/*2^17
        // So:     CCADC2mACoefficient = dividend/shunt(uohm)
        // ref(mV):the internal reference voltage of CCADC with the unit "mV"
        // shunt(uohm): the current sensor resistance with the unit "microOhm"
        // 10^6:compensation for using the shunt with "microOhm"
        // 2^17:the CCADC measurement result range
        // 2^16:up-scaling to increase accuracy
	uint32_t dividend = 0;

        // VrefSel==0, select 220mV as CCADC internal voltage reference
        // VrefSel==1, Select 110mV as CCADC internal voltage reference
        if(VrefSel==CCADC_VOLTREF_220mV){
            dividend = CCADC_VOLTREF_220mV * 500000;
	}else if(VrefSel==CCADC_VOLTREF_110mV){
            dividend = CCADC_VOLTREF_110mV * 500000;
        }

	dividend += (shuntResistance >> 1); // Add halv divisor to get rounding
	
	CCADC2mACoefficient = (uint16_t) (dividend/shuntResistance);
}

/*******************************************************************/
/*! \brief Convert CCADC ticks to mA
 *
 * Formula: (ccadc_ticks*CCADC2mACoefficient)/2^16 = mA
 *
 * \note The ticks value must not be more than 15-bits, or else the calculation
 *       may overflow.
 *
 * \param ticks  CCADC accumulating ticks to convert to mA
 *
 */
int16_t BATTCUR_Ticks2mA(int32_t ticks)
{
	ticks *= CCADC2mACoefficient;
	
	if(ticks >= 0) {
		ticks += (1ul<<15);
		ticks >>= 16;
	} else {
		ticks = -ticks;
		ticks += (1ul<<15);
		ticks >>= 16;
		ticks = -ticks;
	}
	
	return (int16_t)ticks;
}


/*******************************************************************/
/*! \brief Convert mA to CCADC tick
 *
 * Formula: ccadc_ticks = (mA*2^16)/CCADC2mACoefficient
 *
 * \note mA cannot be higher than +- 16384 (ie, dont use more than 14 bits excluding the sign bit)
 *
 * \param mA_in  mA to convert to CCADC accumulating ticks
 */
int32_t BATTCUR_mA2Ticks(int16_t mA_in)
{
	int32_t temp = (int32_t)mA_in*(1ul<<16); // 2^16
	
	temp /= (int32_t)CCADC2mACoefficient;
	
	return temp;
}

/****************************************/
/*! \brief Convert mA to Rcc CCADC ticks
 *
 * \param  mA_in mA to convert to Rcc ticks
 */
int32_t BATTCUR_mA2RccTicks(int16_t mA_in)
{
	uint32_t temp = (uint32_t) BATTCUR_mA2Ticks(mA_in);
	
	// Rcc ticks are 2^5 times as "big" as accumululating ticks
	temp >>= 5;
	
	return temp;
}
