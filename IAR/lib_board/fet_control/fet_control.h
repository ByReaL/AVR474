/* This file has been prepared for Doxygen automatic documentation generation.*/
/*! \file *********************************************************************
 *
 * \brief
 *      Header file for FET control peripheral module driver.
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

#ifndef FET_CONTROL_H
#define FET_CONTROL_H

/******************************************************************************
 Includes
******************************************************************************/
#include "common.h"

/******************************************************************************/
/*! \brief Check if the Current Protection is currently active.
 *
 * The current Protection module has a 1 second timeout timer that will keep the
 * Current Protection STatus bit high for 1 second after a current violation
 * has triggered the battery protection. The bit is cleared by hardware when
 * the timeout has expired.
 *
 *	\return True if Current Protection is active
 */
#define FETCTRL_IsCurrentProtectionActive() (FCSR & (1<<CPS))

/******************************************************************************/
/*! \brief Check if the discharge FET is enabled (conducting).
 *
 *	\return True if discharge FET is enabled (conducting)
 */
#define FETCTRL_IsDischargeFetEnabled() (FCSR & (1<<DFE))

/******************************************************************************/
/*! \brief Check if the charge FET is enabled (conducting).
 *
 *	\return True if charge FET is enabled (conducting)
 */
#define FETCTRL_IsChargeFetEnabled() (FCSR & (1<<CFE))

/******************************************************************************
 Function proto-types
******************************************************************************/
void FETCTRL_EnableDischargeFet( void );
void FETCTRL_DisableDischargeFet( void );
void FETCTRL_EnableChargeFet( void );
void FETCTRL_DisableChargeFet( void );
void FETCTRL_EnableFets( void );
void FETCTRL_DisableFets( void );

void FETCTRL_EnableDeepUnderVoltageMode( void );
void FETCTRL_DisableDeepUnderVoltageMode( void );

#endif
