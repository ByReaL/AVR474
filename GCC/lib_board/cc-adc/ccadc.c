/* This file has been prepared for Doxygen automatic documentation generation.*/
/*! \file *********************************************************************
 *
 * \brief
 *      CCADC module.
 *
 *      Module for configuration and sampling with the Coulomb Counter ADC
 *      (CCADC).
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
 * $Revision: 5957 $
 * $Date: 2009-08-27 19:38:17 +0800 (Thu, 27 Aug 2009) $  \n
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
#include "ccadc.h"
#include "iar_compat.h"
#include <avr/wdt.h>
#include <avr/interrupt.h>

/******************************************************************************
 Global (file scope) variables.
******************************************************************************/

#ifdef COMPENSATE_FOR_ERROR_AFTER_POLARITY_SWITCH
int16_t iccBeforeCadpolChange;
#endif

volatile int32_t CCADC_accResult;
volatile int16_t CCADC_iccResult;

union CCADC_conversionFlag_union{
		uint8_t allFlags;
		struct{
			uint8_t accResultReady : 1 ;
			uint8_t iccResultReady :1 ;
			uint8_t regularCurrentDetected : 1 ;
			uint8_t polaritySwitchOccured : 1 ;
		};
} volatile CCADC_statusFlags = {0x00};

/******************************************************************************
 Proto type for private functions.
******************************************************************************/
static void CCADC_PolarityManager( void );

/*----------------------------------------------------------------------------*/
/*!	\brief Configuration of the CC-ADC and Regular Current Level.
 *
 * Sets the conversion time for both accumulating and regular current measuring,
 * as well as regular current level.
 *
 * Note that the CC-ADC is turned off if it was running, and is not started again
 *
 * NOTE that the Bandgap voltage refence must be enabled and calibrated to used
 * the CC-ADC
 *
 * \param accConversionTime [ACCT_128, ACCT_256, ACCT_512, ACCT_1024]
 * \param rccSamplingInterval [RSI_256, RSI_512, RSI_1024, RSI_2048]
 * \param regularCurrentLevel Given in milliAmps
 *
 * \retval void
 */
void CCADC_Init( CCADC_AccConvPeriod_t accConversionTime, CCADC_RccConvPeriod_t rccSamplingInterval, uint16_t regularCurrentLevel )
{
	// Wait until CC-ADC is ready for configuration change.
	//(wait for synchronization between clock domains to be completed since previous change).
	while( CADCSRA & (1<<CADUB) ){};

#if (CCADC_VOLTREF == CCADC_VOLTREF_110mV)
	//Enable voltage scaling on reference
	CADCSRC |= (1<<CADVSE);
#elif (CCADC_VOLTREF == CCADC_VOLTREF_220mV)
	//Disable voltage scaling on reference
	CADCSRC &= ~(1<<CADVSE);
#else
#error CCADC_VOLTREF must be either CCADC_VOLTREF_220mV or CCADC_VOLTREF_110mV
#endif
	
	// Wait again
	while( CADCSRA & (1<<CADUB) ){};
	
	// Set Regular current detection level.
	CADRCC = BATTCUR_mA2RccTicks(regularCurrentLevel);

	// Wait again
	while( CADCSRA & (1<<CADUB) ){};
	
	// Select CC-ADC Conversion Time, Current Sampling Interval and polarity bit
	// Note: turns off CC-ADC if it was running
	CADCSRA = ((uint8_t)accConversionTime) | ((uint8_t)rccSamplingInterval) | (CCADC_POLARITY_POS);
}


/*----------------------------------------------------------------------------*/
/*!	\brief Selects mode of operation for the CC-ADC
 *	
 * Select between
 * - CCADC_DISABLE (To disable the CC-ADC, and disable and clear interrupts).
 * - CCADC_IAC (CC_ADC is enabled and both Instantaneous and Accumulated Conversions generate interrupts).
 * - CCADC_IC (CC_ADC is enabled and Instantaneous Conversions generate interrupts).
 * - CCADC_ACC (CC_ADC is enabled and Instantaneous Conversions generate interrupts).
 * - CCADC_RCC (Regular Current Condition is enabled - Note that accumulator is cleared)
 *
 * Note that the enabling and disabling does _not_ modify the configuration of CC-ADC. It
 * only enable and disable the mode and interrupts.
 *
 * NOTE that the Bandgap voltage refence must be enabled and calibrated to used
 * the CC-ADC
 *
 *	\param  mode Which mode to set the CCADC to
 *
 */
void CCADC_SetMode( CCADC_modes_t mode )
{
	// Wait for synchronization between clock domains before initiating write
	while( CADCSRA & (1<<CADUB) ){};
	
	switch( mode )
	{
	case CCADC_DISABLE:
		// Disable CC-ADC and disable interrupts (and clear pending interrupts).
		CADCSRA = CADCSRA & ~( (1<<CADEN) | (1<<CADSE) );
		CADCSRB = (0<<CADACIE) | (0<<CADRCIE) | (0<<CADICIE) | (1<<CADACIF) | (1<<CADRCIF) | (1<<CADICIF);
		break;
		
	case CCADC_IAC:
		// Enable CC-ADC with both ICC and ACC interrupts running (and clear pending interrupts).
		CADCSRA = ( CADCSRA & ~(1<<CADSE) ) | (1<<CADEN);
		CADCSRB = (1<<CADACIE) | (0<<CADRCIE) | (1<<CADICIE) | (1<<CADACIF) | (1<<CADRCIF) | (1<<CADICIF);
		break;

	case CCADC_ICC:
		// Enable CC-ADC with ICC interrupt running  (and clear pending interrupts).
		CADCSRA = ( CADCSRA & ~(1<<CADSE) ) | (1<<CADEN);
		CADCSRB = (0<<CADACIE) | (0<<CADRCIE) | (1<<CADICIE) | (1<<CADACIF) | (1<<CADRCIF) | (1<<CADICIF);
		break;

	case CCADC_ACC:
		// Enable CC-ADC with ACC interrupt running  (and clear pending interrupts).
		CADCSRA = ( CADCSRA & ~(1<<CADSE) ) | (1<<CADEN);
		CADCSRB = (1<<CADACIE) | (0<<CADRCIE) | (0<<CADICIE) | (1<<CADACIF) | (1<<CADRCIF) | (1<<CADICIF);
		break;

	case CCADC_RCC:
		// Enable CC-ADC with RCC and ICC interrupts running (ICC due to RCC unable to wake up device from power save).
		CADCSRA = CADCSRA | (1<<CADSE) | (1<<CADEN);
		CADCSRB = (0<<CADACIE) | (1<<CADRCIE) | (1<<CADICIE) | (1<<CADACIF) | (1<<CADRCIF) | (1<<CADICIF);
		break;

	default:
		break;
	}
}

/*----------------------------------------------------------------------------*/
/*!	\brief Provide information whether CC-ADC is in RCC mode of IC/ACC mode.
 *
 *
 * NOTE that the Bandgap voltage reference must be enabled and calibrated to used
 * the CC-ADC
 *
 *	\return mode [CCADC_RCC, CCADC_IAC]
 */
uint8_t CCADC_GetMode( void )
{
	//!< Wait for synchronization between clock domains before reading state
	while( CADCSRA & (1<<CADUB) ){};

	if( CADCSRA & (1<<CADSE) )
	{
		return CCADC_RCC;
	}
	else
	{
		return CCADC_IAC;
	}
}

/*----------------------------------------------------------------------------*/
/*!	\brief Returns result of an CC-ADC Accumulated Conversion.
 *	
 * Returns the 18 bit signed result of an Accumulated Conversion as signed 32-bit.
 * The result is scaled so to use a 24.8 signed fixed point format before it is returned.
 *
 * NOTE that the Bandgap voltage refence must be enabled and calibrated to used
 * the CC-ADC.
 *
 *	\return Latest CCADC ACC result is returned in a 24.8 signed fixed point format
 */
int32_t CCADC_GetAccResult( void )
{
	int32_t result;
	uint8_t interruptState = __save_interrupt();

	__disable_interrupt();
	result = CCADC_accResult;
	__restore_interrupt(interruptState);

	return result;
}

/*----------------------------------------------------------------------------*/
/*!	\brief Returns result of an CC-ADC Instantaneous Conversion.
 *	
 * Returns the 13 bit signed result of an Instantaneous Conversion as signed 16-bit
 *
 *
 * NOTE that the Bandgap voltage refence must be enabled and calibrated to used
 * the CC-ADC
 *
 *	\return Latest CCADC ICC result.
 */
int32_t CCADC_GetIccResult( void )
{
	int16_t result;
	uint8_t interruptState = __save_interrupt();
	
	__disable_interrupt();
	result = CCADC_iccResult;
	__restore_interrupt(interruptState);
	
	return (int32_t)(result) * (1<<5); // Scale the 13-bit Icc result to 18-bit like the Acc result.
}

/*----------------------------------------------------------------------------*/
/*! \brief Controls when to alter the CC-ADC input polarity.
 *
 * Changes the CC-ADC input polarity every \ref CONVERSION_COUNT conversion. Since
 * this should be done as soon after the conversion has completed as possible this
 * function is embedded in the ACC conversion complete interrupt routine. It should
 * not be called from elsewhere, and is therefore static/local file scope only.
 *
 */
#pragma inline = forced
static void CCADC_PolarityManager( void )
{
	static uint8_t polarityCounter = CONVERSION_COUNT;
	
	polarityCounter--;
	
	if(polarityCounter == 0)
	{
		// Save ICC reading from before the CADPOL is changed
		// This is used to compensate for that the first icc (and therefore also
		// acc) readings are wrong after a polarity change.
#ifdef COMPENSATE_FOR_ERROR_AFTER_POLARITY_SWITCH	
		{
			if( CCADC_IS_POLARITY_NEGATIVE() )
			{	
				iccBeforeCadpolChange = -CADIC;
			}
			else
			{
				iccBeforeCadpolChange = CADIC;
			}
		}
#endif
		
		CADCSRA ^= (1<<CADPOL);
		polarityCounter = CONVERSION_COUNT;
		CCADC_statusFlags.polaritySwitchOccured = true;
		
	} else {
		CCADC_statusFlags.polaritySwitchOccured = false;
	}
}

/******************************************************************************/
/*! \brief Used to determine if a new CCADC accumulated current result is ready.
 *
 * Since the internal status flag is also accessed by the interrupt, it is protected
 * by disabling the global interrupt while accessing it.
 *
 *	\return Returns true is new result is ready, otherwise false.
 */
bool CCADC_isAccResultReady( void )
{
	uint8_t interruptState = __save_interrupt();
	__disable_interrupt();

	bool flagState = CCADC_statusFlags.accResultReady;
	CCADC_statusFlags.accResultReady = false;

	__restore_interrupt( interruptState );
	
	return flagState;
}

/******************************************************************************/
/*! \brief Used to determine if a new CCADC instant current result is ready.
 *
 * Since the internal status flag is also accessed by the interrupt, it is protected
 * by disabling the global interrupt while accessing it.
 *
 *	\return Returns true is new result is ready, otherwise false.
 */
bool CCADC_isIccResultReady( void )
{
	uint8_t interruptState = __save_interrupt();
	__disable_interrupt();

	bool flagState = CCADC_statusFlags.iccResultReady;
	CCADC_statusFlags.iccResultReady = false;

	__restore_interrupt( interruptState );
	
	return flagState;
}

/******************************************************************************/
/*! \brief Check if polarity was changed after last completed ACC conversion
 *
 * \retval true  Polarity was changed after last completed ACC conversion
 * \retval false Polarity was not changed after last ACC conversion
 *
 */
bool CCADC_GetPolarityChange( void )
{
	uint8_t interruptState = __save_interrupt();
	__disable_interrupt();

	bool flagState = CCADC_statusFlags.polaritySwitchOccured;

	__restore_interrupt( interruptState );
	
	return flagState;
}


/******************************************************************************/
/*! \brief Returns the raw offset for the accumulating conversion
 *
 * Add this to results from negative polarity and remove from positive polarity
 *
 * \note This will return 32767 if the offset hasn't been calculated
 *
 * \return Offset for the accumulating conversion
 *
 */
int16_t CCADC_GetRawAccOffset( void )
{
	return accOffset;
}


/*****************************************************************************/
/*! \brief Returns the offset for the accumulating conversion
 *
 * Checks the polarity of last conversion, so that the values returned from this
 * function can be directly added to result of last conversion to get a offset
 * calibrated result.
 *
 * If accOffset not is calculated (it is 32767), zero is returned
 *
 * \return Value that should be added to last current.
 *
 */
int16_t CCADC_GetAccOffset( void )
{
	if(accOffset == 32767) {
		return 0;
	}
	bool polChange = CCADC_GetPolarityChange();
	if ( CCADC_IS_POLARITY_NEGATIVE() ) {
		if ( polChange ) {
			return -accOffset;
		} else {
			return accOffset;
		}
	} else {
		if( polChange ) {
			return accOffset;
		} else {
			return -accOffset;
		}
	}
}


/******************************************************************************/
/*! \brief Calculates the accumulating conversion offset
 *
 * Waits until a polarity change occurs, then gathers values for each polarity
 * and calculates the offset.
 *
 * \note TODO: Will take all CPU except for interrupts, so communication etc
 *             will not work. Also the current is not sent to the CCGASG module
 *             at the moment, so this should only be run when very low currents.
 *
 * \note Even more important note: The current has to be constant when this function is run!
 *       Keeping the FETs disabled is the easiest solution.
 */
void CCADC_CalculateAccOffset( void )
{
	int16_t average;
	int16_t firstSampleAverage;
	int16_t secondSampleAverage;
	uint8_t values;
	
	// Wait for a polarity change
	while( ! CCADC_GetPolarityChange() ) {
		while( ! CCADC_isAccResultReady() ) {};
		wdt_reset();
	}
	// Get polarity (wait for synchronization just in case)
	while( CADCSRA & (1<<CADUB) ){};
	bool firstSampleNegative = CCADC_IS_POLARITY_NEGATIVE();
	
	// Discard first reading after polarity change
	while( ! CCADC_isAccResultReady() ) {};
	wdt_reset();
	
	// Save each value until a new polarity change
	values = 0;
	average = 0;
	while( ! CCADC_GetPolarityChange() ) {
		while( ! CCADC_isAccResultReady() ) {};
		wdt_reset();
		average += CCADC_GetAccResult();
		++values;
	}
	firstSampleAverage = average/values;
	
	// Discard first reading after polarity change
	while( ! CCADC_isAccResultReady() ) {};
	wdt_reset();
	
	// Save each value until a new polarity change
	values = 0;
	average = 0;
	while( ! CCADC_GetPolarityChange() ) {
		while( ! CCADC_isAccResultReady() ) {};
		wdt_reset();
		average += CCADC_GetAccResult();
		++values;
	}
	secondSampleAverage = average/values;
	
	accOffset = (firstSampleAverage - secondSampleAverage) / 2;
	if(firstSampleNegative)  {
		accOffset = -accOffset;
	}
}

/******************************************************************************/
/*! \brief CC-ADC instant current conversion complete interrupt.
 *
 */
ISR(CCADC_CONV_vect)
{
	//!< Wait for synchronization between clock domains before reading state
	while( CADCSRA & (1<<CADUB) ){};
	
	if( CCADC_IS_POLARITY_NEGATIVE() )
	{
		CCADC_iccResult = -CADIC;
	}
	else
	{
		CCADC_iccResult = CADIC;
	}
	
	CCADC_statusFlags.iccResultReady = true;
}

/******************************************************************************/
/*! \brief CC-ADC Regular current threshold passed interrupt.
 *
 */
ISR(CCADC_REG_CUR_vect)
{
	CCADC_statusFlags.regularCurrentDetected = true;
}

/******************************************************************************/
/*! \brief CC-ADC Accumulated current conversion complete interrupt.
 *
 */
ISR(CCADC_ACC_vect)
{
	//!< Wait for synchronization between clock domains before reading state
	while( CADCSRA & (1<<CADUB) ){};
	
	if( CCADC_IS_POLARITY_NEGATIVE() )
	{
		CCADC_accResult = - ( CADAC0 + (CADAC1 << 8) + (CADAC2 << 16) + (CADAC3 << 24) );
	}
	else
	{
		CCADC_accResult = ( CADAC0 + (CADAC1 << 8) + (CADAC2 << 16) + (CADAC3 << 24) );
	}
	
#ifdef COMPENSATE_FOR_ERROR_AFTER_POLARITY_SWITCH
	// If polarityCounter is equal to 1, the current result stored in CCADC_accResults
	// should be compensated because it's the first conversion after a polarity switch
	if(CCADC_statusFlags.polaritySwitchOccured) {
		CCADC_accResult += (iccBeforeCadpolChange>>2);
	}
#endif
	
	CCADC_PolarityManager();
	CCADC_statusFlags.accResultReady = true;
}
