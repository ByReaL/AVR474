/* This file has been prepared for Doxygen automatic documentation generation.*/
/*! \file *********************************************************************
 *
 * \brief
 *      Common headerfile.
 *
 *      Headerfile included in all
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
 * $Revision: 6090 $
 * $Date: 2009-10-05 21:40:12 +0800 (Mon, 05 Oct 2009) $  \n
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

#ifndef COMMON_H
#define COMMON_H


/* Includes. ***********
 */
#include <ioavr.h>
#include <inavr.h>
#include <stdint.h>
#include <stdbool.h>
#include <error.h>




/* Defines. ************
 */

//! System clock frequency in Hz.
#define SYSTEM_CLK_MHz     (1)             //!< System frequency in Hz. Only whole MHz currently allowed.
#define SYSTEM_CLK_HZ      (1000000UL)
#define SLOW_RC_CLK_HZ     (131000UL)
#define ULP_RC_CLK_HZ      (128000UL)
#define ULP_RC_CLK_HZ_MIN  (89000UL)

#define ZERO_KELVIN        (273)   //!< 0 K in degrees celcius.


/* Prototypes for intrinsic assembler functions. */
uint8_t  READ_SIGNATUREBYTE(uint8_t addr);
uint16_t READ_SIGNATUREWORD(uint8_t addr);
uint16_t CRC_16_Atmel(uint16_t crc, uint16_t data);
uint8_t SMBus_PEC(uint8_t lastCRC, uint8_t newByte);


#endif // COMMON_H

