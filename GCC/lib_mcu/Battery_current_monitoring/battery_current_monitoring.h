/* This file has been prepared for Doxygen automatic documentation generation.*/
/*! \file *********************************************************************
 *
 * \brief
 *      Header file for CC-ADC result processing firmware module.
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

#ifndef BATT_CURRENT_MONITOR_H
#define BATT_CURRENT_MONITOR_H

/******************************************************************************
 Includes
******************************************************************************/
#include "common.h"
#include "battery_pack_parameters.h"


/******************************************************************************
 Function proto-types
******************************************************************************/
void BATTCUR_StoreCurrent( int32_t current );
int32_t BATTCUR_GetCurrent( void );
int32_t BATTCUR_GetOffsetCalibratedCurrent( void );
void BATTCUR_UpdateAverageCurrent( int32_t current );
int32_t BATTCUR_GetAverageCurrent( void );
bool BATTCUR_IsCurrentLow( int32_t current );
void BATTCUR_InitializeAverageCurrent( int32_t current );
bool BATTCUR_IsDischargeOngoing( void );

void BATTCUR_CalculateShuntCoeffient(uint16_t shuntResistance,uint8_t VrefSel);
int16_t BATTCUR_Ticks2mA(int32_t ticks);
int32_t BATTCUR_mA2Ticks(int16_t mA_in);
int32_t BATTCUR_mA2RccTicks(int16_t mA_in);


#endif
