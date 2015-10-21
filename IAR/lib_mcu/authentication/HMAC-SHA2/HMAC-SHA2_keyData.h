/* This file has been prepared for Doxygen automatic documentation generation.*/
/*! \file *********************************************************************
 *
 * \brief
 *      HMAC-SHA2 header file
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

#ifndef HMAC_SHA2_KEYDATA
#define HMAC_SHA2_KEYDATA

// Calculated from the following secret key:
// In ASCII:	This is a long key for HMAC-SHA2
// In hex:   	54686973 20697320 61206c6f 6e67206b 65792066 6f722048 4d41432d 53484132
// In hex:   	546869732069732061206c6f6e67206b657920666f7220484d41432d53484132

//! Data table containing the resulting hash from performing the inital SHA-2 operation on the secret key XOR 0x36
__flash uint32_t ful_inSHA[] = {
	0xb243ea78,
	0xad850388,
	0x86e93248,
	0xe4547e11,
	0x01e0293c,
	0xbdd54e11,
	0x8a8a1761,
	0x3695f1a5};

//! Data table containing the resulting hash from performing the inital SHA-2 operation on the secret key XOR 0x5c
__flash uint32_t ful_outSHA[] = {
	0x8a78e085,
	0xf4b1b9a1,
	0xff1da060,
	0x23a59f8c,
	0x32e02be0,
	0x0955bbd5,
	0x54a025de,
	0x57d89751};

#endif
