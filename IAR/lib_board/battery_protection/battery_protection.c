/* This file has been prepared for Doxygen automatic documentation generation.*/
/*! \file *********************************************************************
 *
 * \brief
 *      Battery Protection peripheral module driver.
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

/******************************************************************************
 Included files
******************************************************************************/
#include "common.h"
#include "battery_protection.h"

/******************************************************************************
 Included files
******************************************************************************/
bool batteryProtectionTriggered = false;
uint8_t chargeProtectionCounter = 0;

/******************************************************************************
 Interrupt function declarations.
******************************************************************************/
#pragma vector = BPINT_vect
__interrupt void BATTPROT_BatteryProtection_ISR(void)
{
	batteryProtectionTriggered = true;
	
	// If short circuit is detected
	if( BPIFR & (1<<SCIF) )
	{
		// TODO: Add short circuit handler here if the standard handling (reenabling) is not sufficient.
	}

	// If Discharging overcurrent is detected
	if( BPIFR & (1<<DOCIF) )
	{
		// TODO: Add discharging overcurrent handler here if the standard handling (reenabling) is not sufficient.
	}

	// If charging overcurrent is detected
	if( BPIFR & (1<<COCIF) )
	{
		// TODO: Add charging overcurrent handler here if the standard handling (reenabling) is not sufficient.
		chargeProtectionCounter++;
	}
	
	// If Discharging high-current is detected
	if( BPIFR & (1<<DHCIF) )
	{
		// TODO: Add discharging high-current handler here if the standard handling (reenabling) is not sufficient.
		chargeProtectionCounter++;
	}

	// If charging high-current is detected
	if( BPIFR & (1<<CHCIF) )
	{
		// TODO: Add charging high-current handler here if the standard handling (reenabling) is not sufficient.
	}

	// Clear all Bettery Protection interrupt flags.
	BPIFR = (1<<SCIF)|(1<<DOCIF)|(1<<COCIF)|(1<<DHCIF)|(1<<CHCIF);
}

/******************************************************************************
 Function declarations.
******************************************************************************/

/******************************************************************************/
/*! \brief Set the short-circuit current detection level and reaction time.
 *
 *	\param dischargingLevel is an enumerator type that sets the level in Amps.
 *	\param reacTime is the value to write to the timing register (\ref BATTPROT_ScrtUsecToHex).
 *
 *	\retval void
 */
void BATTPROT_SetShortCircuitDetection( uint8_t dischargingLevel, uint8_t reacTime )
{
	BPSCTR = reacTime;
	BPSCD = dischargingLevel;
}

/******************************************************************************/
/*! \brief Set the discharging/charging over-current detection level and reaction time.
 *
 *	\param dischargingLevel is an enumerator type that sets the level in Amps.
 *	\param chargingLevel is an enumerator type that sets the level in Amps.
 *	\param reacTime is the value to write to the timing register (\ref BATTPROT_OcrtMsecToHex).
 *
 *	\retval void
 */
void BATTPROT_SetOverCurrentDetection( uint8_t dischargingLevel, uint8_t chargingLevel, uint8_t reacTime )
{
	BPOCTR = reacTime;
	BPDOCD = dischargingLevel;
	BPCOCD = chargingLevel;
}

/******************************************************************************/
/*! \brief Set the charging high-current detection level and reaction time.
 *
 *	\param dischargingLevel is an enumerator type that sets the level in Amps.
 *	\param chargingLevel is an enumerator type that sets the level in Amps.
 *	\param reacTime is the value to write to the timing register (\ref BATTPROT_HcrtMsecToHex).
 *
 *	\retval void
 */
void BATTPROT_SetHighCurrentDetection( uint8_t dischargingLevel, uint8_t chargingLevel, uint8_t reacTime )
{
	BPHCTR = reacTime;
	BPDHCD = dischargingLevel;
	BPCHCD = chargingLevel;
}

/******************************************************************************/
/*! \brief Lock the battery protection settings (only unlocked by reset).
 *
 *	\retval void.
 */
void BATTPROT_LockBatteryProtectionConfig( void )
{
	uint8_t temp = (0<<BPPLE)|(1<<BPPL);
	BPPLR = (1<<BPPLE)|(1<<BPPL);
	BPPLR = temp;
}
	
/******************************************************************************/
/*! \brief Checks if Battery Protection has been triggered.
 *
 * Checks if the Battery Protection module has detected short-circuit, overcurrent
 * or/and high-current. If true is returned a detection has occurred. By checking if
 * detection has occurred the internal detecting flag is cleared.
 *
 *	\retval bool Will return true if the Battery Protection has been triggered, false otherwise.
 */
bool BATTPROT_IsProtectionTriggered( void )
{
	bool temp = batteryProtectionTriggered;

	batteryProtectionTriggered = false;
	return temp;
}


/******************************************************************************/
/*! \brief Clear charge protection counter
 *
 */
void BATTPROT_ClearChargeProtectionCounter( void )
{
	uint8_t interruptState = __save_interrupt();

	__disable_interrupt();
	chargeProtectionCounter = 0;
	__restore_interrupt(interruptState);
}

/******************************************************************************/
/*! \brief Get charge protection counter
 *
 */
uint8_t BATTPROT_GetChargeProtectionCounter( void )
{
	uint8_t ret;
	
	uint8_t interruptState = __save_interrupt();
	
	__disable_interrupt();
	ret = chargeProtectionCounter;
	__restore_interrupt(interruptState);
	
	return ret;
}
