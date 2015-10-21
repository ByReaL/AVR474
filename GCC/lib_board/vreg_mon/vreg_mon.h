/* This file has been prepared for Doxygen automatic documentation generation.*/
/*! \file *********************************************************************
 *
 * \brief
 *      Headerfile for vreg_mon.c.
 *
 *      Contains defines and prototype(s) for vreg_mon.c.
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

#ifndef VREGMON_H
#define VREGMON_H

/******************************************************************************
 * Includes.
 ******************************************************************************/
#include "common.h"


/******************************************************************************
 * Defines.
 ******************************************************************************/
typedef enum  { 
	BATPROT_SAVE = 0,
	EXTINT_SAVE,
	WDT_SAVE,
	TIMER0_SAVE,
	TIMER1_SAVE,
	SPI_SAVE,
	VADC_SAVE,
	CCADC_SAVE,
	EEPROM_SAVE,
	PRR0_SAVE
} vregmonSaveStaus_t;


/******************************************************************************
 * Macros.
 ******************************************************************************/

/******************************************************************************/
/*! \brief Enable VREGMON Interrupt.
 *
 *  This macro enables the voltage regulator monitoring interrupt which detects 
 *  if the regulator operates in battery pack short mode.
 ******************************************************************************/
#define VREGMON_InterruptEnable()  ( ROCR |= (1<<ROCWIE) )

/******************************************************************************/
/*! \brief Disable VREGMON Interrupt.
 *
 *  This macro disables the voltage regulator monitoring interrupt which detects 
 *  if the regulator operates in battery pack short mode.
 ******************************************************************************/
#define VREGMON_InterruptDisable()  ( ROCR &= ~(1<<ROCWIE) )

/******************************************************************************/
/*! \brief Check if device operates in Battery pack short mode.
 *
 *  This macro enables checks the ROC Status bit in Regulator Operating 
 *  Condition Register which is set if the regulator operates in Battery 
 *  pack short mode.
 ******************************************************************************/
#define VREGMON_ConditionExists()  ( ROCR & (1<<ROCS) )


/******************************************************************************
 * Prototypes.
 ******************************************************************************/
uint8_t VREGMON_ConditionTriggered(void);


#endif  // VREGMON_H

