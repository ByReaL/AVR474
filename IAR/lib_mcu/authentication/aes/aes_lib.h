/* This file has been prepared for Doxygen automatic documentation generation.*/
/*! \file *********************************************************************
 *
 * \brief
 *      AES library
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
 * Copyright (c) 2006, Atmel Corporation All rights reserved.
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

#ifndef AES_LIB_H
#define AES_LIB_H

#include <stdint.h>

#include "battery_pack_parameters.h" // includes AES_KEY_SIZE

/******************************************************************************
 * Misc. constant defines.
 ******************************************************************************/

//! AES state block size in number of bytes.
#define AES_BLOCK_SIZE 16

// Calculate some info based on key bit count. Required by the AES_KEY_BUFFER_SIZE value.
#if (AES_KEY_SIZE == 16)
#  define AES_BLOCKS_IN_BUFFER 1 //!< Number of blocks that will fit in the schedule buffer.
#elif (AES_KEY_SIZE == 24)
#  define AES_BLOCKS_IN_BUFFER 3 //!< Number of blocks that will fit in the schedule buffer.
#elif (AES_KEY_SIZE == 32)
#  define AES_BLOCKS_IN_BUFFER 2 //!< Number of blocks that will fit in the schedule buffer.
#else
#  error Key must be 128, 192 or 256 bits!
#endif

//! Bytes needed for key schedule calculation, and the size of the 'keyBuffer' buffers.
#define AES_KEY_BUFFER_SIZE (AES_BLOCK_SIZE * AES_BLOCKS_IN_BUFFER)



/******************************************************************************
 * Function prototypes.
 ******************************************************************************/

//! Calculate starting point for key schedule to be used for encryption. Key must be preloaded.
void AES_PrepareEncrypt( uint8_t * keyBuffer );
//! Encrypt data block with on-the-fly calculation of key schedule in 'keyBuffer'.
void AES_Encrypt( uint8_t * block, uint8_t * keyBuffer );
//! Calculate starting point for key schedule to be used for decryption. Key must be preloaded.
void AES_PrepareDecrypt( uint8_t * keyBuffer );
//! Decrypt data block using prepared key schedule state from 'keyBuffer'.
void AES_Decrypt( uint8_t * block, uint8_t * keyBuffer );


#endif
// end of file

