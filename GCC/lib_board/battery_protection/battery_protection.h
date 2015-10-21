/* This file has been prepared for Doxygen automatic documentation generation.*/
/*! \file *********************************************************************
 *
 * \brief
 *      Header file for Battery Protection peripheral module driver.
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
 Includes
******************************************************************************/
#ifndef BATTERY_PROTECTION_H
#define BATTERY_PROTECTION_H

#include "common.h"

/******************************************************************************
 Type definitions
******************************************************************************/
typedef enum currentProtectionDetectionLevel_enum {	
	BATTPROT_VMAX_5mV   = 0xE0,
	BATTPROT_VMAX_10mV  = 0xF1,
	BATTPROT_VMAX_15mV  = 0xF2,
	BATTPROT_VMAX_20mV  = 0xF3,
	BATTPROT_VMAX_25mV  = 0xF4,
	BATTPROT_VMAX_30mV  = 0xF5,
	BATTPROT_VMAX_35mV  = 0xF6,
	BATTPROT_VMAX_40mV  = 0xF7,
	BATTPROT_VMAX_45mV  = 0xF8,
	BATTPROT_VMAX_50mV  = 0xF9,
	BATTPROT_VMAX_55mV  = 0xFA,
	BATTPROT_VMAX_60mV  = 0xFB,
	BATTPROT_VMAX_65mV  = 0xFC,
	BATTPROT_VMAX_70mV  = 0xFD,
	BATTPROT_VMAX_75mV  = 0xFE,
	BATTPROT_VMAX_80mV  = 0x77,
	BATTPROT_VMAX_90mV  = 0x78,
	BATTPROT_VMAX_100mV = 0x79,
	BATTPROT_VMAX_110mV = 0x7A,
	BATTPROT_VMAX_120mV = 0x7B,
	BATTPROT_VMAX_130mV = 0x7C,
	BATTPROT_VMAX_140mV = 0x7D,
	BATTPROT_VMAX_160mV = 0x37,
	BATTPROT_VMAX_180mV = 0x38,
	BATTPROT_VMAX_200mV = 0x39,
	BATTPROT_VMAX_220mV = 0x3A,
	BATTPROT_VMAX_240mV = 0x3B,
	BATTPROT_VMAX_260mV = 0x3C,
	BATTPROT_VMAX_280mV = 0x3D,
	BATTPROT_VMAX_320mV = 0x17,
	BATTPROT_VMAX_360mV = 0x18,
	BATTPROT_VMAX_400mV = 0x19,
	BATTPROT_VMAX_440mV = 0x1A
} currentProtDetectLevel_t; //!< Enumerator type for Battery protection level (BATTPROT)



/******************************************************************************
 Defines
******************************************************************************/
/*!
 * If charge protection occurs 5 times "in a row", the charge fet will get disabled
 * until a discharge current is detected.
 *
 * Because the FET automatically will be disabled for one second after a over/high
 * current has occured, it will not occur again the second after. So to be able to
 * count haw many times it has occured in a row, more then BATTPROT_REOCCURING_CHARGE_PROTECTION_GAP
 * seconds have to elapse until it is not considered as in a row again.
 */
#define BATTPROT_REOCCURING_CHARGE_PROTECTION_LIMIT 5
#define BATTPROT_REOCCURING_CHARGE_PROTECTION_GAP 3

/******************************************************************************
 Macro functions
******************************************************************************/
#define BATTPROT_ScrtUsecToHex(microSeconds) ((uint8_t)(microSeconds/62.5))  //!< Conversion from useconds to the register/hex value used when setting the register. Provide hex that gives a max time of n us.
#define BATTPROT_OcrtMsecToHex(milliSeconds) ((uint8_t)(milliSeconds/0.5))  //!< Conversion from mseconds to the register/hex value used when setting the register. Provide hex that gives a max time of n ms.
#define BATTPROT_HcrtMsecToHex(milliSeconds) ((uint8_t)(milliSeconds/2.0))  //!< Conversion from mseconds to the register/hex value used when setting the register. Provide hex that gives a max time of n ms.

//Enabling and disabling of the interrupts (setting and clearing the interrupt masks)
#define BATTPROT_EnableShortCircuitInterrupt() (BPIMSK |= (1<<SCIE))    //!< Enable short-circuit interrupt (Battery Protection Interrupt)
#define BATTPROT_DisableShortCircuitInterrupt() (BPIMSK &= ~(1<<SCIE))  //!< Disable short-circuit (Battery Protection Interrupt)
#define BATTPROT_EnableChargeOverCurrentInterrupt() (BPIMSK |= (1<<COCIE))  //!< Enable charging over-current interrupt (Battery Protection Interrupt)
#define BATTPROT_DisableChargeOverCurrentInterrupt() (BPIMSK &= ~(1<<COCIE))//!< Disable charging over-current interrupt (Battery Protection Interrupt)
#define BATTPROT_EnableChargeHighCurrentInterrupt() (BPIMSK |= (1<<CHCIE))  //!< Enable charging high-current interrupt (Battery Protection Interrupt)
#define BATTPROT_DisableChargeHighCurrentInterrupt() (BPIMSK &= ~(1<<CHCIE))//!< Disable charging high-current interrupt (Battery Protection Interrupt)
#define BATTPROT_EnableDiscargeOverCurrentInterrupt() (BPIMSK |= (1<<DOCIE))  //!< Enable discharging over-current interrupt (Battery Protection Interrupt)
#define BATTPROT_DisableDiscargeOverCurrentInterrupt() (BPIMSK &= ~(1<<DOCIE))//!< Disable discharging over-current interrupt (Battery Protection Interrupt)
#define BATTPROT_EnableDischargeHighCurrentInterrupt() (BPIMSK |= (1<<DHCIE))  //!< Enable discharging high-current interrupt (Battery Protection Interrupt)
#define BATTPROT_DisableDischargeHighCurrentInterrupt() (BPIMSK &= ~(1<<DHCIE))//!< Disable discharging high-current interrupt (Battery Protection Interrupt)
#define BATTPROT_EnableAllInterrupts() (BPIMSK = (1<<SCIE)|(1<<COCIE)|(1<<CHCIE)|(1<<DOCIE)|(1<<DHCIE)) //!< Enable all Battery Protection interrupts.
#define BATTPROT_DisableAllInterrupts() (BPIMSK = 0x00) //!< Disable all Battery Protection interrupts

// Enabling and disabling the Battery Protection features.
#define BATTPROT_EnableExternalInputProtection() (BPCR &= ~(1<<EPID)) //!< Enable the external protection input (default enabled).
#define BATTPROT_DisableExternalInputProtection() (BPCR |= (1<<EPID)) //!< Disable the external protection input.
#define BATTPROT_EnableShortCircuitProtection() (BPCR &= ~(1<<SCD)) //!< Enable the short-circuit protection (default enabled).
#define BATTPROT_DisableShortCircuitProtection() (BPCR |= (1<<SCD)) //!< Disable the short-circuit protection.
#define BATTPROT_EnableChargeOverCurrentProtection() (BPCR &= ~(1<<COCD)) //!< Enable the charging over-current protection (default enabled).
#define BATTPROT_DisableChargeOverCurrentProtection() (BPCR |= (1<<COCD)) //!< Disable the charging over-current.
#define BATTPROT_EnableChargeHighCurrentProtection() (BPCR &= ~(1<<CHCD)) //!< Enable the charging high-current protection (default enabled).
#define BATTPROT_DisableChargeHighCurrentProtection() (BPCR |= (1<<CHCD)) //!< Disable the charging high-current.
#define BATTPROT_EnableDischargeOverCurrentProtection() (BPCR &= ~(1<<DOCD)) //!< Enable the discharging over-current protection (default enabled).
#define BATTPROT_DisableDischargeOverCurrentProtection() (BPCR |= (1<<DOCD)) //!< Disable the discharging over-current.
#define BATTPROT_EnableDischargeHighCurrentProtection() (BPCR &= ~(1<<DHCD)) //!< Enable the discharging high-current protection (default enabled).
#define BATTPROT_DisableDischargeHighCurrentProtection() (BPCR |= (1<<DHCD)) //!< Disable the discharging high-current.

/******************************************************************************
 Function proto-types
******************************************************************************/
void BATTPROT_SetShortCircuitDetection( uint8_t dischargingLevel, uint8_t reacTime);
void BATTPROT_SetOverCurrentDetection( uint8_t dischargingLevel, uint8_t chargingLevel, uint8_t reacTime );
void BATTPROT_SetHighCurrentDetection( uint8_t dischargingLevel, uint8_t chargingLevel, uint8_t reacTime );

void BATTPROT_LockBatteryProtectionConfig( void );
bool BATTPROT_IsProtectionTriggered( void );

void BATTPROT_ClearChargeProtectionCounter( void );
uint8_t BATTPROT_GetChargeProtectionCounter( void );


#endif
