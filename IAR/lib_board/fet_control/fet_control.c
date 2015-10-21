/* This file has been prepared for Doxygen automatic documentation generation.*/
/*! \file *********************************************************************
 *
 * \brief
 *      FET control peripheral module driver.
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
#include "fet_control.h"


/******************************************************************************/
/*! \brief Enable the discharge FET.
 * 
 * If the Current Protection Status bit is set a write to the FCSR register is
 * ignored, the protection bit must be cleared from the calling function in advance. 
 * A write the FCSR is also ignored if synchronisation between clock domains is
 * ongoing (while guard time is active).
 *
 */
void FETCTRL_EnableDischargeFet( void )
{
	//If the device has not entered protection mode, write until success (waiting for synchronization mechanism).
	while( !(FCSR & (1<<CPS)) && !(FCSR &(1<<DFE)) )
	{
		FCSR |= (1<<DFE);
	}
}
/******************************************************************************/
/*! \brief Disable the discharge FET.
 * 
 * If the Current Protection Status bit is set a write to the FCSR register is
 * ignored, however the FETs are already disabled in that case...
 * A write the FCSR is also ignored if synchronisation between clock domains is
 * ongoing (while guard time is active).
 *
 */
void FETCTRL_DisableDischargeFet( void )
{
	//Write until success (waiting for synchronization mechanism).
	while( (FCSR &(1<<DFE)) )
	{
		FCSR &= ~(1<<DFE);
	}
}


/******************************************************************************/
/*! \brief Enable the charge FET.
 * 
 * If the Current Protection Status bit is set a write to the FCSR register is
 * ignored, the protection bit must be cleared from the calling function in advance. 
 * A write the FCSR is also ignored if synchronisation between clock domains is
 * ongoing (while guard time is active).
 *
 */
void FETCTRL_EnableChargeFet( void )
{
	//If the device has not entered protection mode, write until success (waiting for synchronization mechanism).
	while( !(FCSR & (1<<CPS)) && !(FCSR &(1<<CFE)) )
	{
		FCSR |= (1<<CFE);
	}
}

/******************************************************************************/
/*! \brief Disable the charge FET.
 * 
 * If the Current Protection Status bit is set a write to the FCSR register is
 * ignored, however the FETs are already disabled in that case...
 * A write the FCSR is also ignored if synchronisation between clock domains is
 * ongoing (while guard time is active).
 *
 */
void FETCTRL_DisableChargeFet( void )
{
	//Write until success (waiting for synchronization mechanism).
	while( (FCSR &(1<<CFE)) )
	{
		FCSR &= ~(1<<CFE);
	}
}

/******************************************************************************/
/*! \brief Enable both the charge and the discharge FETs.
 * 
 * If the Current Protection Status bit is set a write to the FCSR register is
 * ignored, the protection bit must be cleared from the calling function in advance. 
 * A write the FCSR is also ignored if synchronisation between clock domains is
 * ongoing (while guard time is active).
 *
 */
void FETCTRL_EnableFets( void )
{
	//If the device has not entered protection mode, write until success (waiting for synchronization mechanism).
	while( !(FCSR & (1<<CPS)) && !((FCSR & (1<<CFE)) && (FCSR & (1<<DFE))) )
	{
		FCSR |= (1<<CFE)|(1<<DFE);
	}
}

/******************************************************************************/
/*! \brief Disable both the charge and the discharge FETs.
 * 
 * If the Current Protection Status bit is set a write to the FCSR register is
 * ignored, however the FETs are already disabled in that case...
 * A write the FCSR is also ignored if synchronisation between clock domains is
 * ongoing (while guard time is active).
 *
 */
void FETCTRL_DisableFets( void )
{
	//Write until success (waiting for synchronization mechanism).
	while( (FCSR & (1<<CFE)) || (FCSR & (1<<DFE)) )
	{
		FCSR &= ~( (1<<CFE) | (1<<DFE) );
	}
}

/******************************************************************************/
/*! \brief Enable the Deep Under Voltage mode (precharging at very low battery voltage).
 * 
 * A write the FCSR is also ignored if synchronisation between clock domains is
 * ongoing (while guard time is active).
 *
 */
void FETCTRL_EnableDeepUnderVoltageMode( void )
{
	//Write until success (waiting for synchronization mechanism).
	while( (FCSR & (1<<DUVRD)) )
	{
		FCSR &= ~(1<<DUVRD);
	}
}

/******************************************************************************/
/*! \brief Disable the Deep Under Voltage mode (precharging at very low battery voltage).
 * 
 * A write the FCSR is also ignored if synchronisation between clock domains is
 * ongoing (while guard time is active).
 *
 */
void FETCTRL_DisableDeepUnderVoltageMode( void )
{
	//Write until success (waiting for synchronization mechanism).
	while( !(FCSR & (1<<DUVRD)) )
	{
		FCSR |= (1<<DUVRD);
	}
}
