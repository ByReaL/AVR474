/* This file has been prepared for Doxygen automatic documentation generation.*/
/*! \file *********************************************************************
 *
 * \brief
 *      Watchdog Timer peripheral module driver.
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
#include <inavr.h>
#include "common.h"
#include "watchdog.h"

/******************************************************************************
 Interrupt functions
******************************************************************************/

/******************************************************************************/
/*! \brief Watchdog Interrupt Handler - does nothing except waking up the device.
 */
#pragma vector = WDT_vect
__interrupt void WDT_Timeout_ISR( void )
{
}

/******************************************************************************
 Function declarations
******************************************************************************/

/******************************************************************************/
/*! \brief Watchdog disabling - nice when debugging.
 *
 *	To be able to stop the Watchdog, the WDR bit in MCUSR must be cleared.
 *
 */
void WDT_Disable(void)
{
	WDT_ResetTimer();
	MCUSR &= ~(1<<WDRF);
	WDTCSR |= (1<<WDCE) | (1<<WDE);
	WDTCSR = 0x00;
}

/******************************************************************************/
/*! \brief Change the timeout period of the Watchdog.
 * 
 * Can be used to change the timeout period while using the Watchdog as a wake-up
 * timer
 *
 * \note Do not optimize because then IAR messes it up...
 */
#pragma optimize=0
void WDT_SetTimeOut( WDT_timeout_t timeout )
{    
	uint8_t temp;
	
	/* Setup Watchdog */
	temp = (1<<WDIF)|(0<<WDIE)|(0<<WDCE)|(1<<WDE )| // Set Change Enable bit and Enable Watchdog System Reset Mode,
			timeout;           // and set Watchdog timeout period to some number of msec.
	WDTCSR   =   (1<<WDCE)|(1<<WDE);  // Set Change Enable bit
	WDTCSR   =   temp;
}
