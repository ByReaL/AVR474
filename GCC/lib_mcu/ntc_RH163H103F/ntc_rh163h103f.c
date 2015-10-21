/*This file has been prepared for Doxygen automatic documentation generation.*/
/*! \file *********************************************************************
 *
 * \brief 
 *     Routines for converting result from a RH163h103f thermistor to either
 *     celsius or kelvin.
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
 * Copyright (c) 2008, Atmel Corporation All rights reserved.
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

// Includes
#include "common.h"
#include "ntc_rh163h103f.h"
#include <avr/pgmspace.h>

//! The following define gives how many �C there is between each major step. 
//! This value must match the major step size that was used when generating the
//! data tables.
#define NTC_MAJOR_STEPS			5


//! The following define gives how many minor steps there are per �C
#define NTC_MINOR_STEPS			10


//! The following defines gives the minimum temperature that is supported by the 
//! conversion software. This value must match the minimum value that was used 
//! when generating the data tables and the minimum VADC value.
#define NTC_MIN_TEMPERATURE	-30


//! The following defines gives the maximum temperature that is supported by the 
//! conversion software. This value must match the maximum value that was used 
//! when generating the data tables.
#define NTC_MAX_TEMPERATURE	85


//! The following table gives the scaled voltage for each major step in temperature.
//! Scaled according to VADC_TEMP_VOLTAGE_SCALING in the VADC module.
//! ucft_voltage[0] is the voltage at the minimum temperature, and the last
//! value is the voltage at the maximum temperature
//!
//! NOTE: The difference between two neighbouring value * TEMP_MAJOR_STEPS * TEMP_MINOR_STEPS cannot
//! be bigger than what fits into an unsigned 16bit variable
//!
//! Mitsubishi RH16-3h103f thermistor, A = 10000, B = 3486
const uint16_t ucft_voltage[] PROGMEM =   {20412,19956,19408,18764,18022,17188,16268,15276,14232,13158,12072,11000,9960,8968,8035,7172,6381,5663,5018,4442,3929,3475,3074,2719};


//! Index in ucft_voltage for the minimum temperature
#define NTC_MIN_TEMPERATURE_INDEX 0
//! Index in ucft_voltage for the maximum temperature
#define NTC_MAX_TEMPERATURE_INDEX ((uint8_t)((NTC_MAX_TEMPERATURE-NTC_MIN_TEMPERATURE)/NTC_MAJOR_STEPS))


/*! \brief Convert ADC reading to degress celsius.
 *
 * This function uses the temperature data tables to find the corresponding 
 * temperature in �C. The resolution is given by the minor step size.
 * The result is given as �C * number of minor steps/�C
 * The data tables were generated by using typical numbers from the thermistor 
 * datasheet.
 *
 * The function will return MinimumTemperature if temp is below minimum temp
 * and will return MaximumTemperature if temperature is above maximum
 * 
 * \param ui_input_value Scaled mV
 *
 * \return Temperature (in 0.1 Celcius)
 */
int16_t NTC_ReadTemperatureCelsius(uint16_t ui_input_value)
{
	// Points to index of the voltage representing the closest major temp step below the last temperature
	// At reset it will be set to 20 degrees, as ambient temperature most likely is between 20 and 25 degrees
	static uint8_t uc_last_major_temp = (uint8_t)((20-NTC_MIN_TEMPERATURE)/NTC_MAJOR_STEPS);
	
	int16_t i_temperature;
	
	// If temperature is below minimum legal value, return that (minimum temperature = maximum voltage)
	if(ui_input_value >= ucft_voltage[NTC_MIN_TEMPERATURE_INDEX]) {
		return NTC_MIN_TEMPERATURE * NTC_MINOR_STEPS;
	}
	// If temperature is above maximum legal value, return that (minimum voltage = maximum temperature)
	if(ui_input_value <= ucft_voltage[NTC_MAX_TEMPERATURE_INDEX]) {
		return NTC_MAX_TEMPERATURE * NTC_MINOR_STEPS;
	}
	
	// Want uc_last_major_temp to point to the major temperature step just below the voltage ui_input_value represents
	// If voltage is higher than at uc_last_major_temp, search downwards
	// else, search upwards
	if(ui_input_value > ucft_voltage[uc_last_major_temp] ) {
		do {
			uc_last_major_temp--;
		} while(ui_input_value >= ucft_voltage[uc_last_major_temp]);
		
	} else {
		do {
			uc_last_major_temp++;
		} while(ui_input_value <= ucft_voltage[uc_last_major_temp]);
		
		uc_last_major_temp--; //Point to a lower temp/higher voltage than what ui_input_value_represents
	}
	
	// Requires TEMP_MIN_TEMPERATURE to be negative
	i_temperature = (uc_last_major_temp + NTC_MIN_TEMPERATURE/NTC_MAJOR_STEPS)*NTC_MINOR_STEPS*NTC_MAJOR_STEPS;
	
	// ui_input_value will now be between ucft_voltage[uc_last_major_temp] and ucft_voltage[uc_last_major_temp+1]
	// Linear interpolation to get temperature to add to i_temperature: (MinorSteps*MajorSteps*diff + roundFix)/divisor
	uint16_t diff = ucft_voltage[uc_last_major_temp] - ui_input_value;
	uint16_t divisor = ucft_voltage[uc_last_major_temp] - ucft_voltage[uc_last_major_temp+1];
	uint16_t roundFix = divisor>>1;
	
	i_temperature += (NTC_MAJOR_STEPS*NTC_MINOR_STEPS*diff + roundFix)/divisor;
	
	return(i_temperature);									// Return the converted temperature as temperature in �C * number of minor steps per �C
}

/*! \brief Convert ADC reading to degress kelvin.
 *
 * This function will just call NTC_ReadTemperatureCelsius and then add 273,1 degrees to the
 * result
 * 
 * \param ui_input_value Scaled mV
 * \return Temperature (in 0.1 Kelvin)
 */
uint16_t NTC_ReadTemperatureKelvin(uint16_t ui_input_value)
{
	return (uint16_t)(NTC_ReadTemperatureCelsius(ui_input_value) + 2731);
}
