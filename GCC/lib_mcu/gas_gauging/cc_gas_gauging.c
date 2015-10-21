/* This file has been prepared for Doxygen automatic documentation generation.*/
/*! \file *********************************************************************
 *
 * \brief
 *      Coulomb based gas gauging.
 *
 *      This module contains functions for coulomb based gas gauging.
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
#include "cc_gas_gauging.h"
#include "battery_pack_parameters.h"
#include "rc_calibration.h"

int32_t AccumulatedCCADCvalue;     //!< Time compensated and accumulated current measurements.
uint32_t ChargeCycleAcc;           //!< Accumulator for counting charge cycles. Not time compensated as that much accuracy not is needed

uint16_t acc2mAhCoefficient; // Used in this formula to get mAh: (ccaccumulated*acc2mAhCoefficient)/2^24 = mAh
uint32_t mAh2accCoefficient; // Used in this formula to get acc: ccaccumulated = mAh*mAh2accCoefficient


/******************************************************************************
 Function declarations.
******************************************************************************/

/******************************************************************************/
/*! \brief Accumulation of CCADC measurements.
 *
 *  Accumulated CCADC measurements are compensated with actual sampling period
 *  before being stored in a file internal variable used for other functions in
 *  this module.
 *
 *	\param ccadcMeasurement The CCADC measurement to be accumulated.
 *  \param slowRCperiod  Period of CCADC clock (Slow RC) in us*1024.
 */
void CCGASG_AccumulateCCADCMeasurements(int32_t ccadcMeasurement, uint16_t slowRCperiod)
{
	// Sampling period dependant on configuration of CCADC sampling..
	int32_t temp = ccadcMeasurement * (int32_t)slowRCperiod;
	
	bool polChange = false;
	if(temp < 0) {
		temp = -temp;
		polChange = true;
	}
	
	// Add 0.5*divisor to get proper rounding
	temp += (1<<(CCGASG_ACC_SCALING-1));
	temp >>= CCGASG_ACC_SCALING;
	
	if(polChange) {
		temp = -temp;
	}

	AccumulatedCCADCvalue += temp;
	
	// If it was a charge, update the charge cycle counter
	if(ccadcMeasurement > 0) {
		ChargeCycleAcc += ccadcMeasurement;
		if(ChargeCycleAcc >= battParams_sram.capacityInCCAccumulated) {
			 ChargeCycleAcc -= battParams_sram.capacityInCCAccumulated;
			 BATTPARAM_IncreaseCycleCount();
		}
	} else {
		// If it was a discharge, AccumulatedCADCvalue can be negative, and that
		// is "impossible", so set it to zero
		if(AccumulatedCCADCvalue < 0) {
			AccumulatedCCADCvalue = 0;
		}
	}
}


/******************************************************************************/
/*! \brief Read out raw values accumulated.
 *
 *  The resolution is dependant the CCADC, timing compensation, sense resistor
 *  and the #CCGASG_ACC_SCALING. The CCGASG_Acc2mAh(int32_t accumulated) function
 *  can be used if result in mAh is desired.
 *
 *	\return The accumulated and time compensated current measurement is returned.
 */
int32_t CCGASG_ReadAccumulatedCC(void)
{
	return AccumulatedCCADCvalue;
}


/******************************************************************************/
/*! \brief Update State of Charge.
 *
 *  Update SoC according to input from voltage based gas gauging. 0% and 100%
 *  used for updating full and empty. Any value between sets the accumulated
 *  variable #AccumulatedCCADCvalue to the corresponding mAh.
 *
 *	\param stateofCharge SoC in percent.
 *
 *	\retval void
 */
void CCGASG_SetStateofCharge(uint16_t stateofCharge)
{
	// Samping period dependant on configuration of CCADC sampling..
	if ( stateofCharge == 0 ) {
		AccumulatedCCADCvalue = 0;
	} else if ( stateofCharge == 1000 ) {
		AccumulatedCCADCvalue = battParams_sram.capacityInCCAccumulated;
	} else {
		AccumulatedCCADCvalue = ((battParams_sram.capacityInCCAccumulated)/100)*stateofCharge;
	}
}


/************************************************************************************/
/*! \brief Calculate the Shunt Coefficient for converting between accumulated and mAh
 *
 * Definition:
 *      Period = CCADC_CLK_Period * 2^10
 *
 *         CCADC_CLK_Period: the period of CCADC clock, read it from signature row
 *         2^10: 10bit up-scaling
 *
 *      acc = CCADC_Value*Period/2^13
 *
 *         CCADC_Value: CCADC conversion value
 *         Period: time compensation factor
 *         2^13: 13bit down-scaling
 *
 * Formula for conversion factor from acc to mAh:
 *
 *      acc2mAhCoefficient = mAh*2^24 / acc
 *                         = ref(mV)*h*10^6*2^13*2^24/(Shunt(uOhm)*Period*2^17)
 *
 *         ref(mV): CCADC internal voltage reference in mV
 *         h: the accumulation time, 1s, that is 1/3600 hour
 *         10^6: compensation for using the shunt with "microOhm"
 *         2^13: 13bit down-scaling from "acc" definition
 *         2^24: 24bit up-scaling
 *         Shunt(uOhm): current sensor resistance in uOhm
 *         2^17: the full conversion value for ref(mV)
 *
 * Formula for conversion factor from mAh to acc
 *
 *      mAh2accCoefficient = acc / mAh
 *                         = Shunt(uOhm)*Period*2^17 / (ref(mV)*h*10^6*2^13)
 *
 *         ref(mV): CCADC internal voltage reference in mV
 *         h: the accumulation time, 1s, that is 1/3600 hour
 *         10^6: compensation for using the shunt with "microOhm"
 *         2^13: 13bit down-scaling from "acc" definition
 *         Shunt(uOhm): current sensor resistance in uOhm
 *         Period: time compensation factor
 *         2^17: the full conversion value for ref(mV)
 *
 * \param  ccadc_clkPeriod  The actual CC-ADC Clock period
 * \param  shuntResistance  Shunt resistance in microohms
 * \param  VrefSel          Selects the voltage reference to use
 */
#pragma optimize=3
void CCGASG_CalculateShuntCoefficients(uint16_t ccadc_clkPeriod,uint16_t shuntResistance, uint8_t VrefSel)
{
        // Period = CCADC_CLK_Period * 2^10
        // CCADC_CLK_Period: the period of CCADC clock,can read it from signature row
        //                   and temperature compensation should be used
        // 2*10: 10bit up-scaling
    uint64_t Period = (uint64_t)ccadc_clkPeriod; //typical value: 7813 when ULP_RC clk: 131072Hz

        // acc = CCADC_Value*Period/2^13
        // CCADC_Value: CCADC conversion value
        // Period: time compensation factor
        // 2^13: 13bit down-scaling
        //
        // acc2mAhCoefficient = mAh*2^24/acc
        //                        = ref(mV)*h*10^6*2^13*2^24/(Shunt(uOhm)*Period*2^17)
        //
        // ref(mV): CCADC internal voltage reference in mV
        // h: the accumulation time, 1s, that is 1/3600 hour
        // 10^6: compensation for using the shunt with "microOhm"
        // 2^13: 13bit down-scaling from "acc" definition
        // 2^24: 24bit up-scaling
        // Shunt(uOhm): current sensor resistance in uOhm
        // Period: time compensation factor
        // 2^17: the full conversion value for ref(mV)


        uint64_t divisor = (uint64_t)Period * 36;
        uint64_t dividend;

        if(VrefSel==CCADC_VOLTREF_220mV){
	        // VrefSel==CCADC_VOLTREF_220mV, select 220mV as CCADC internal voltage reference
		dividend = (uint64_t) CCADC_VOLTREF_220mV * 10000 *(1<<13) *(1<<(24-17)); // Decimal: 2 306 867 200 000, HEX: 0000 0219 1C00 0000
        }else if(VrefSel==CCADC_VOLTREF_110mV){
		// VrefSel==CCADC_VOLTREF_110mV, Select 110mV as CCADC internal voltage reference
		dividend = (uint64_t) CCADC_VOLTREF_110mV * 10000 *(1<<13) *(1<<(24-17)); // Decimal: 1 153 433 600 000, HEX: 0000 010C 8E00 0000
        }

        dividend += divisor >> 1;
        dividend = dividend / divisor;
	
	// Add half to get rounding
	dividend += shuntResistance >> 1;

	// Calculate the resulting acc2mAhCoefficient based on the shuntResistance
	acc2mAhCoefficient = dividend/shuntResistance;
	
        // mAh2accCoefficient = acc / mAh
        //                    = Shunt(uOhm)*Period*2^17 / (ref(mV)*h*10^6*2^13)
        //
        // ref(mV): CCADC internal voltage reference in mV
        // h: the accumulation time, 1s, that is 1/3600 hour
        // 10^6: compensation for using the shunt with "microOhm"
        // 2^13: 13bit down-scaling from "acc" definition
        // Shunt(uOhm): current sensor resistance in uOhm
        // Period: time compensation factor
        // 2^17: the full conversion value for ref(mV)

        dividend = (uint64_t)shuntResistance * (1<<(17-13)) * Period *36; //Decimal: 106 156 800 000, HEX: 0000 0018 B770 3800

        if(VrefSel==CCADC_VOLTREF_220mV){
	    divisor = (uint64_t)CCADC_VOLTREF_220mV * 10000;
        }else if(VrefSel==CCADC_VOLTREF_110mV){
            divisor = (uint64_t)CCADC_VOLTREF_110mV * 10000;
        }

	// Add half to get rounding
	dividend += (divisor>>1);
		
	mAh2accCoefficient = dividend / divisor;  // 10228 with Period =7813
}


/*******************************************************************/
/*! \brief Convert accumulated to mAh
 *
 * Formula: (accumulated*acc2mAhCoefficient)/2^24 = mAh
 *
 * \param  accumulated The accumulated current in internal units
 *
 * \return The accumulated capacity in mAh
 */
int16_t CCGASG_Acc2mAh(int32_t accumulated)
{
	int64_t temp = (int64_t)accumulated * (int64_t)acc2mAhCoefficient;

	bool polChange = false;
	
	if(temp < 0) {
		polChange = true;
		temp = - temp;
	}
	
	temp += (1ul<<23);
	temp >>= 24;
	
	if(polChange) {
		temp = -temp;
	}
	
	return (int16_t)temp;
}

/*******************************************************************/
/*! \brief Convert mAh to accumulated
 *
 * Formula: accumulated = mAh*mAh2accCoefficient
 *
 * \param mA_in mAh to convert to internal units
 *
 * \return mA_in converted to internal units.
 */
int32_t CCGASG_mAh2Acc(int16_t mA_in)
{
	return (int32_t)mAh2accCoefficient*(int32_t)mA_in;
}
