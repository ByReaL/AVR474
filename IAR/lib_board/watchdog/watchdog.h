/* This file has been prepared for Doxygen automatic documentation generation.*/
/*! \file *********************************************************************
 *
 * \brief
 *      Header file for Watchdog Timer peripheral module driver.
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

#ifndef WATCHDOG_H
#define WATCHDOG_H

/******************************************************************************
 Includes
******************************************************************************/
#include "common.h"

/******************************************************************************
 Type definitions
******************************************************************************/
/*! Enumerator type for Watchdog timeouts */
typedef enum WDT_timeout_enum {
	WDTO_16ms =  (0<<WDP3) | (0<<WDP2) | (0<<WDP1) | (0<<WDP0),
	WDTO_32ms =  (0<<WDP3) | (0<<WDP2) | (0<<WDP1) | (1<<WDP0),
	WDTO_64ms =  (0<<WDP3) | (0<<WDP2) | (1<<WDP1) | (0<<WDP0),
	WDTO_128ms = (0<<WDP3) | (0<<WDP2) | (1<<WDP1) | (1<<WDP0),
	WDTO_256ms = (0<<WDP3) | (1<<WDP2) | (0<<WDP1) | (0<<WDP0),
	WDTO_512ms = (0<<WDP3) | (1<<WDP2) | (0<<WDP1) | (1<<WDP0),
	WDTO_1Kms =  (0<<WDP3) | (1<<WDP2) | (1<<WDP1) | (0<<WDP0),
	WDTO_2Kms =  (0<<WDP3) | (1<<WDP2) | (1<<WDP1) | (1<<WDP0),
	WDTO_4Kms =  (1<<WDP3) | (0<<WDP2) | (0<<WDP1) | (0<<WDP0),
	WDTO_8Kms =  (1<<WDP3) | (0<<WDP2) | (0<<WDP1) | (1<<WDP0),
	WDT0_mask =  (1<<WDP3) | (1<<WDP2) | (1<<WDP1) | (1<<WDP0)  //!< Mask for getting timing bits from the WDTCSR register.
} WDT_timeout_t;

/******************************************************************************
 Defines
******************************************************************************/


/******************************************************************************
 Macro functions
******************************************************************************/
/******************************************************************************/
/*! \brief Reset the Watchdog Timer.
 */
#define WDT_ResetTimer() (__watchdog_reset())

/******************************************************************************/
/*! \brief Enable the Watchdog (wake-up) interrupt.
 * 
 * To be able to get a known timeout the Watchdog timer should be reset just 
 * before of after calling this function.
 */
#define WDT_EnableWakeUpInterrupt()   (WDTCSR |= (1<<WDIE))   //!< Enable Watchdog Interrupt Mode  (wakeup interrupt).

/******************************************************************************/
/*! \brief disable the Watchdog (wake-up) interrupt.
 */
#define WDT_DisableWakeUpInterrupt()  (WDTCSR &= ~(1<<WDIE)) //!< Disable Watchdog Interrupt Mode (wakeup interrupt).

/******************************************************************************
 Function proto-types
******************************************************************************/
void WDT_SetTimeOut( WDT_timeout_t timeout );
void WDT_Disable(void);

#endif
