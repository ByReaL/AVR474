/* This file has been prepared for Doxygen automatic documentation generation.*/
/*! \file *********************************************************************
 *
 * \brief
 *      Contains defines for setting of sleep modes and entering sleep, and for 
 *      disabling and re-enabling clock to various modules by setting and 
 *      clearing the power reduction register.
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

#ifndef PWR_MGMNT_H
#define PWR_MGMNT_H

#include "common.h"

/* Defines */

//! Enum for sleep modes.
typedef enum  {
	SLEEP_MODE_DISABLE    = ((0<<SM2) | (0<<SM1) | (0<<SM0) | (0<<SE)),	//!< Sleep mode disable.
	SLEEP_MODE_IDLE       = ((0<<SM2) | (0<<SM1) | (0<<SM0) | (1<<SE)),	//!< Sleep mode idle.
	SLEEP_MODE_ADC_NR     = ((0<<SM2) | (0<<SM1) | (1<<SM0) | (1<<SE)),	//!< Sleep mode adc noise reduction.
	SLEEP_MODE_POWER_SAVE = ((0<<SM2) | (1<<SM1) | (1<<SM0) | (1<<SE)),	//!< Sleep mode power save.
	SLEEP_MODE_POWER_OFF  = ((1<<SM2) | (0<<SM1) | (0<<SM0) | (1<<SE))	//!< Sleep mode power off.
} sleepModes_t;


//! Disable sleep modes
#define SLEEP_DISABLE() {\
	SMCR = SLEEP_MODE_DISABLE; }

//! Set idle sleep mode
#define SLEEP_SET_IDLE_MODE() { SMCR = SLEEP_MODE_IDLE; }

//! Enter idle sleep mode
#define SLEEP_ENTER_IDLE()  {\
	SMCR = SLEEP_MODE_IDLE; \
	__sleep(); }

//! Set ADC noise reduction mode
#define SLEEP_SET_ADC_NR_MODE() { SMCR = SLEEP_MODE_ADC_NR; }

//! Enter ADC noise reduction mode
#define SLEEP_ENTER_ADC_NR()  {\
	SMCR = SLEEP_MODE_ADC_NR; \
	__sleep(); }

//! Set powersave mode
#define SLEEP_SET_POWER_SAVE_MODE() { SMCR = SLEEP_MODE_POWER_SAVE; }

//! Enter powersave mode
#define SLEEP_ENTER_POWER_SAVE()  {\
	SMCR = SLEEP_MODE_POWER_SAVE; \
	__sleep(); }

//! Enter power-off mode
#define SLEEP_ENTER_POWER_OFF()  {\
	__disable_interrupt(); \
	SMCR = SLEEP_MODE_POWER_OFF; \
	__sleep(); }



/******************************************************************************
 *
 * Macros to disable the clock to various modules. This can significantly reduce
 * current consumption in active and idle modes.
 *
 ******************************************************************************/
//! Enable power reduction mode for TWI
#define PRR_DISABLE_TWI()      (PRR0 |= (1<<PRTWI))
//! Enable power reduction mode for VREGMON
#define PRR_DISABLE_VREGMON()  (PRR0 |= (1<<PRVRM))
//! Enable power reduction mode for SPI
#define PRR_DISABLE_SPI()      (PRR0 |= (1<<PRSPI))
//! Enable power reduction mode for TIMER1
#define PRR_DISABLE_TIMER1()   (PRR0 |= (1<<PRTIM1))
//! Enable power reduction mode for TIMER0
#define PRR_DISABLE_TIMER0()   (PRR0 |= (1<<PRTIM0))
//! Enable power reduction mode for VADC
#define PRR_DISABLE_VADC()     (PRR0 |= (1<<PRVADC))


/******************************************************************************
 *
 * Macros to re-enable the clock to various modules by clearing bits in the
 * power reduction register
 *
 ******************************************************************************/
//! Disable power reduction mode for TWI
#define PRR_ENABLE_TWI()      (PRR0 &= ~(1<<PRTWI))
//! Disable power reduction mode for VREGMON
#define PRR_ENABLE_VREGMON()  (PRR0 &= ~(1<<PRVRM))
//! Disable power reduction mode for SPI
#define PRR_ENABLE_SPI()      (PRR0 &= ~(1<<PRSPI))
//! Disable power reduction mode for TIMER1
#define PRR_ENABLE_TIMER1()   (PRR0 &= ~(1<<PRTIM1))
//! Disable power reduction mode for TIMER0
#define PRR_ENABLE_TIMER0()   (PRR0 &= ~(1<<PRTIM0))
//! Disable power reduction mode for VADC
#define PRR_ENABLE_VADC()     (PRR0 &= ~(1<<PRVADC))


#endif

