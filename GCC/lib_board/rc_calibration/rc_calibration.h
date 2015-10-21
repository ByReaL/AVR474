/* This file has been prepared for Doxygen automatic documentation generation.*/
/*! \file *********************************************************************
 *
 * \brief
 *      Defines and prototypes for rc_calibration.c.
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

#include "common.h"

#ifndef RC_CALIB_H
#define RC_CALIB_H

/******************************************************************************
 Defines
******************************************************************************/

// Prescalers for Timer0. 0 = OFF

typedef enum TIM0_prescaler_enum{
	T0_PRESCALER_0 = ((0 << CS02)|(0 << CS01)|(0 << CS00)),
	T0_PRESCALER_1 = ((0 << CS02)|(0 << CS01)|(1 << CS00)),
	T0_PRESCALER_8 = ((0 << CS02)|(1 << CS01)|(0 << CS00)),
	T0_PRESCALER_64 = ((0 << CS02)|(1 << CS01)|(1 << CS00)),
	T0_PRESCALER_256 = ((1 << CS02)|(0 << CS01)|(0 << CS00)),
	T0_PRESCALER_1024 = ((1 << CS02)|(0 << CS01)|(1 << CS00))
} TIM0_prescaler_t;

#define T0_PRESCALER (T0_PRESCALER_1)

//! System clock frequency in Hz.
#define FAST_RC_TARGET_MHZ   SYSTEM_CLK_MHz    //!< Fast RC target frequency in Hz. Only whole MHz currently allowed.
#define SLOW_RC_NOMINAL_PERIOD (7813)         //!< Nominal period time for the Slow RC oscillator (1/131072), scaled by 2^10.
//#define SLOW_RC_NOMINAL_PERIOD (15625)         //!< Nominal period time for the Slow RC oscillator (1/131072), scaled by 2^11.
#define FAST_RC_SEGMENT_BIT   (5)                                //!< FOSCCAL bit deciding range. Used for segment search.
#define FAST_RC_NO_SEGMENTS   (256/(1 << FAST_RC_SEGMENT_BIT))   //!< Number of segments in Fast RC.
#define MAX_FAST_RC_ERROR     (10)                               //!< Max error tolerated on Fast RC in ~0,1%, i.e. 10 ~= 1%.

#define RC_CALIB_CYCLES       (8)              //!< Number of calibration cycles cycles for OSI calibration.

#define SLOW_RC 				(1 << OSISEL0)      //!< For selecting Fast RC calibration source
#define ULP_RC					(0 << OSISEL0)      //!< For selecting Fast RC calibration source

typedef enum {
	RCCAL_CALIB_INIT,
	RCCAL_CALIB_WORKING,
	RCCAL_CALIB_FINISHED,
	RCCAL_CALIB_FAILURE
		
} RCCAL_CalibState_t;

typedef enum {
	RCCAL_IC_INITIAL_CAPTURE,
	RCCAL_IC_CAPTURES,
	RCCAL_IC_FINISHED
		
} RCCAL_IC_State_t;

/***********************
 Function-like macros
************************/
//! Checks if calibration is running (ie, if the input capture interrupt is enabled)
#define RCCAL_IS_RUNNING() (TIMSK0 & (1<<ICIE0))

/******************************************************************************
 Function proto-types
******************************************************************************/
uint16_t RCCAL_CalculateSlowRCperiod(int16_t temperature);
uint16_t RCCAL_CalculateUlpRCperiod(int16_t temperature);

// Commented out because it is never used
//uint16_t RCCAL_MeasureULPRCperiod(uint16_t cal_RC_period);
void RCCAL_StartCalibrateFastRC();
void RCCAL_CalibrateFastRCruntime(uint16_t cal_RC_period);

RCCAL_CalibState_t RCCAL_GetState();


#endif // RC_CALIB_H
