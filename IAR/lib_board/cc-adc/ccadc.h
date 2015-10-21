/*This file has been prepared for Doxygen automatic documentation generation.*/
/*! \file *********************************************************************
 *
 * \brief
 *      CCADC module header file
 *
 *      Module for configuration and sampling with the Coulomb Counter ADC 
 *      (CCADC).
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
 * $Revision: 5651 $
 * $Date: 2009-05-20 19:25:16 +0800 (Wed, 20 May 2009) $
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
 *****************************************************************************/

#ifndef CCADC_H
#define CCADC_H

/******************************************************************************
 Includes
******************************************************************************/
#include "common.h"
#include "battery_pack_parameters.h"

/******************************************************************************
 Defines
******************************************************************************/


/*! \brief	CC-ADC operation modes. */
typedef enum CCADC_modes_enum{
	CCADC_DISABLE = 0,   //!< To disable the CC-ADC, and disable and clear interrupts.
	CCADC_IAC = 1,       //!< CC_ADC is enabled and both Instantaneous Current and Accumulated Current Conversions generate interrupts.
	CCADC_ICC = 2,       //!< CC_ADC is enabled and Instantaneous Current Conversions generate interrupts.
	CCADC_ACC = 3,       //!< CC_ADC is enabled and Accumulated Current Conversions  generate interrupts.
	CCADC_RCC = 4        //!< Regular Current Condition is enabled - Note that accumulator is disabled and cleared by this
} CCADC_modes_t;

/*! \brief CC-ADC Accumulated Current Conversion Periods - in milliseconds*/
typedef enum CCADC_AccConversionPeriod_enum{
	ACCT_128 = ( (0<<CADAS1)|(0<<CADAS0) ),      //!< Equals to 16K (16384) ULP cycles
	ACCT_256 = ( (0<<CADAS1)|(1<<CADAS0) ),      //!< Equals to 32K (32768) ULP cycles
	ACCT_512 = ( (1<<CADAS1)|(0<<CADAS0) ),      //!< Equals to 64K (65536) ULP cycles
	ACCT_1024 = ( (1<<CADAS1)|(1<<CADAS0) )     //!< Equals to 128K (131072) ULP cycles
}CCADC_AccConvPeriod_t;

/*! \brief CC-ADC Regular Current Sampling Intervals - in milliseconds*/
typedef enum CCADC_RccConversionPeriod_enum{
	RCCI_256 = ( (0<<CADSI1)|(0<<CADSI0) ),   //!< Equals to 32K (32768) ULP cycles
	RCCI_512 = ( (0<<CADSI1)|(1<<CADSI0) ),   //!< Equals to 64K (65536) ULP cycles
	RCCI_1024 = ( (1<<CADSI1)|(0<<CADSI0) ),  //!< Equals to 128K (131072) ULP cycles
	RCCI_2048 = ( (1<<CADSI1)|(1<<CADSI0) )   //!< Equals to 256K (262144) ULP cycles
}CCADC_RccConvPeriod_t;

//Other defines
#define CCADC_POLARITY_POS	(0<<CADPOL)
#define CCADC_POLARITY_NEG	(1<<CADPOL)

/*! True if CCADC polarity is negative, false if it is positive */
#define CCADC_IS_POLARITY_NEGATIVE() (CADCSRA & (1<<CADPOL))

/******************************************************************************
 Function prototypes.
******************************************************************************/
void CCADC_Init( CCADC_AccConvPeriod_t accConversionTime, CCADC_RccConvPeriod_t rccSamplingInterval, uint16_t regularCurrentLevel );
void CCADC_SetMode( CCADC_modes_t mode );
uint8_t CCADC_GetMode( void );
int32_t CCADC_GetAccResult( void );
int32_t CCADC_GetIccResult( void );
bool CCADC_isAccResultReady( void );
bool CCADC_isIccResultReady( void );
bool CCADC_GetPolarityChange( void );

int16_t CCADC_GetRawAccOffset( void );
int16_t CCADC_GetAccOffset( void );
void CCADC_CalculateAccOffset( void );



#endif // CCADC_H
