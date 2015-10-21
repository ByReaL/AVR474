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

#include "battery_pack_parameters.h" // Included to get AUTH_USE_AES

#ifdef AUTH_USE_AES

#include "aes_lib.h"

#include "avrtypes.h"

/******************************************************************************
 * Misc. internal constant defines.
 ******************************************************************************/

// Calculate some info based on key bit count.
#if AES_KEY_SIZE == 16
#  define AES_ROUNDS 10 //!< Number of rounds/iterations in algorithm.
#  define AES_KEYS_IN_BUFFER 1 //!< Number of keys that fits in the schedule buffer.
#  define AES_LAST_RCON_VALUE 0x6C //!< Round constant taken from last encryption round.
#elif AES_KEY_SIZE == 24
#  define AES_ROUNDS 12 //!< Number of rounds/iterations in algorithm.
#  define AES_KEYS_IN_BUFFER 2 //!< Number of keys that fits in the schedule buffer.
#  define AES_LAST_RCON_VALUE 0x36 //!< Round constant taken from last encryption round.
#elif AES_KEY_SIZE == 32
#  define AES_ROUNDS 14 //!< Number of rounds/iterations in algorithm.
#  define AES_KEYS_IN_BUFFER 1 //!< Number of keys that fits in the schedule buffer.
#  define AES_LAST_RCON_VALUE 0x80 //!< Round constant taken from last encryption round.
#else
#  error Key must be 128, 192 or 256 bits!
#endif


//! Lower 8 bits of AES polynomial (x^8+x^4+x^3+x+1), ie. (x^4+x^3+x+1).
#define AES_BPOLY 0x1b



/******************************************************************************
 * AES substitution step lookup tables in Flash memory.
 ******************************************************************************/

//! S-Box lookup table.
static uint8_t const FLASH_DECLARE( AES_sBox[256] ) = {
	0x63, 0x7c, 0x77, 0x7b, 0xf2, 0x6b, 0x6f, 0xc5,
	0x30, 0x01, 0x67, 0x2b, 0xfe, 0xd7, 0xab, 0x76,
	0xca, 0x82, 0xc9, 0x7d, 0xfa, 0x59, 0x47, 0xf0,
	0xad, 0xd4, 0xa2, 0xaf, 0x9c, 0xa4, 0x72, 0xc0,
	0xb7, 0xfd, 0x93, 0x26, 0x36, 0x3f, 0xf7, 0xcc,
	0x34, 0xa5, 0xe5, 0xf1, 0x71, 0xd8, 0x31, 0x15,
	0x04, 0xc7, 0x23, 0xc3, 0x18, 0x96, 0x05, 0x9a,
	0x07, 0x12, 0x80, 0xe2, 0xeb, 0x27, 0xb2, 0x75,
	0x09, 0x83, 0x2c, 0x1a, 0x1b, 0x6e, 0x5a, 0xa0,
	0x52, 0x3b, 0xd6, 0xb3, 0x29, 0xe3, 0x2f, 0x84,
	0x53, 0xd1, 0x00, 0xed, 0x20, 0xfc, 0xb1, 0x5b,
	0x6a, 0xcb, 0xbe, 0x39, 0x4a, 0x4c, 0x58, 0xcf,
	0xd0, 0xef, 0xaa, 0xfb, 0x43, 0x4d, 0x33, 0x85,
	0x45, 0xf9, 0x02, 0x7f, 0x50, 0x3c, 0x9f, 0xa8,
	0x51, 0xa3, 0x40, 0x8f, 0x92, 0x9d, 0x38, 0xf5,
	0xbc, 0xb6, 0xda, 0x21, 0x10, 0xff, 0xf3, 0xd2,
	0xcd, 0x0c, 0x13, 0xec, 0x5f, 0x97, 0x44, 0x17,
	0xc4, 0xa7, 0x7e, 0x3d, 0x64, 0x5d, 0x19, 0x73,
	0x60, 0x81, 0x4f, 0xdc, 0x22, 0x2a, 0x90, 0x88,
	0x46, 0xee, 0xb8, 0x14, 0xde, 0x5e, 0x0b, 0xdb,
	0xe0, 0x32, 0x3a, 0x0a, 0x49, 0x06, 0x24, 0x5c,
	0xc2, 0xd3, 0xac, 0x62, 0x91, 0x95, 0xe4, 0x79,
	0xe7, 0xc8, 0x37, 0x6d, 0x8d, 0xd5, 0x4e, 0xa9,
	0x6c, 0x56, 0xf4, 0xea, 0x65, 0x7a, 0xae, 0x08,
	0xba, 0x78, 0x25, 0x2e, 0x1c, 0xa6, 0xb4, 0xc6,
	0xe8, 0xdd, 0x74, 0x1f, 0x4b, 0xbd, 0x8b, 0x8a,
	0x70, 0x3e, 0xb5, 0x66, 0x48, 0x03, 0xf6, 0x0e,
	0x61, 0x35, 0x57, 0xb9, 0x86, 0xc1, 0x1d, 0x9e,
	0xe1, 0xf8, 0x98, 0x11, 0x69, 0xd9, 0x8e, 0x94,
	0x9b, 0x1e, 0x87, 0xe9, 0xce, 0x55, 0x28, 0xdf,
	0x8c, 0xa1, 0x89, 0x0d, 0xbf, 0xe6, 0x42, 0x68,
	0x41, 0x99, 0x2d, 0x0f, 0xb0, 0x54, 0xbb, 0x16
};

#ifdef AES_INCLUDE_DECRYPT
//! Inverse S-Box lookup table.
static uint8_t const FLASH_DECLARE( AES_invSBox[256] ) = {
	0x52, 0x09, 0x6a, 0xd5, 0x30, 0x36, 0xa5, 0x38,
	0xbf, 0x40, 0xa3, 0x9e, 0x81, 0xf3, 0xd7, 0xfb,
	0x7c, 0xe3, 0x39, 0x82, 0x9b, 0x2f, 0xff, 0x87,
	0x34, 0x8e, 0x43, 0x44, 0xc4, 0xde, 0xe9, 0xcb,
	0x54, 0x7b, 0x94, 0x32, 0xa6, 0xc2, 0x23, 0x3d,
	0xee, 0x4c, 0x95, 0x0b, 0x42, 0xfa, 0xc3, 0x4e,
	0x08, 0x2e, 0xa1, 0x66, 0x28, 0xd9, 0x24, 0xb2,
	0x76, 0x5b, 0xa2, 0x49, 0x6d, 0x8b, 0xd1, 0x25,
	0x72, 0xf8, 0xf6, 0x64, 0x86, 0x68, 0x98, 0x16,
	0xd4, 0xa4, 0x5c, 0xcc, 0x5d, 0x65, 0xb6, 0x92,
	0x6c, 0x70, 0x48, 0x50, 0xfd, 0xed, 0xb9, 0xda,
	0x5e, 0x15, 0x46, 0x57, 0xa7, 0x8d, 0x9d, 0x84,
	0x90, 0xd8, 0xab, 0x00, 0x8c, 0xbc, 0xd3, 0x0a,
	0xf7, 0xe4, 0x58, 0x05, 0xb8, 0xb3, 0x45, 0x06,
	0xd0, 0x2c, 0x1e, 0x8f, 0xca, 0x3f, 0x0f, 0x02,
	0xc1, 0xaf, 0xbd, 0x03, 0x01, 0x13, 0x8a, 0x6b,
	0x3a, 0x91, 0x11, 0x41, 0x4f, 0x67, 0xdc, 0xea,
	0x97, 0xf2, 0xcf, 0xce, 0xf0, 0xb4, 0xe6, 0x73,
	0x96, 0xac, 0x74, 0x22, 0xe7, 0xad, 0x35, 0x85,
	0xe2, 0xf9, 0x37, 0xe8, 0x1c, 0x75, 0xdf, 0x6e,
	0x47, 0xf1, 0x1a, 0x71, 0x1d, 0x29, 0xc5, 0x89,
	0x6f, 0xb7, 0x62, 0x0e, 0xaa, 0x18, 0xbe, 0x1b,
	0xfc, 0x56, 0x3e, 0x4b, 0xc6, 0xd2, 0x79, 0x20,
	0x9a, 0xdb, 0xc0, 0xfe, 0x78, 0xcd, 0x5a, 0xf4,
	0x1f, 0xdd, 0xa8, 0x33, 0x88, 0x07, 0xc7, 0x31,
	0xb1, 0x12, 0x10, 0x59, 0x27, 0x80, 0xec, 0x5f,
	0x60, 0x51, 0x7f, 0xa9, 0x19, 0xb5, 0x4a, 0x0d,
	0x2d, 0xe5, 0x7a, 0x9f, 0x93, 0xc9, 0x9c, 0xef,
	0xa0, 0xe0, 0x3b, 0x4d, 0xae, 0x2a, 0xf5, 0xb0,
	0xc8, 0xeb, 0xbb, 0x3c, 0x83, 0x53, 0x99, 0x61,
	0x17, 0x2b, 0x04, 0x7e, 0xba, 0x77, 0xd6, 0x26,
	0xe1, 0x69, 0x14, 0x63, 0x55, 0x21, 0x0c, 0x7d
};
#endif


/******************************************************************************
 * Internal function prototypes.
 ******************************************************************************/

//! Add 'count' bytes from 'constant' into 'bytes' using XOR.
static void AES_AddConstant( uint8_t * bytes, uint8_t const * constant, uint8_t count );
//! Copy 'count' bytes from 'source' to 'destination'.
static void AES_CopyBytes( uint8_t * destination, uint8_t const * source, uint8_t count);
//! Cycle a 4-byte array left once.
static void AES_CycleLeft( uint8_t * row );
//! Perform an AES column mix operation on the 4 bytes in 'column' buffer.
static void AES_MixColumn( uint8_t * column );
//! Perform AES column mixing for the whole AES block/state.
static void AES_MixColumns( uint8_t * state );
//! Substitute 'count' bytes in the 'bytes' buffer in SRAM using the S-Box.
static void AES_SubBytes( uint8_t * bytes, uint8_t count );
//! Perform AES shift-rows operation for the whole AES block/state.
static void AES_ShiftRows( uint8_t * state );


//! XOR 'count' bytes from 'constant' buffer into 'bytes' and copy result to 'destination'.
#if AES_KEY_SIZE == 24
static void AES_AddConstAndCopy(
		uint8_t * bytes,
		uint8_t const * constant,
		uint8_t * destination,
		uint8_t count );
#endif
//! XOR 'count' bytes from 'buf1' and 'buf2' buffers and copy result to both buffers.
#if (AES_KEY_SIZE == 16 || AES_KEY_SIZE == 32)
static void AES_AddAndCopy( uint8_t * buf1, uint8_t * buf2, uint8_t count );
#endif

//! XOR 'count' bytes from 'buf1' and 'buf2' buffers and store result in 'destination'.
/* commented out because it is never used
static void AES_AddAndStore(
		uint8_t const * buf1,
		uint8_t const * buf2,
		uint8_t * destination,
		uint8_t count );*/
//! XOR 'count' bytes from 'constant' buffer into 'bytes' and substitute using S-Box.
static void AES_AddConstAndSubst( uint8_t * bytes, uint8_t const * constant, uint8_t count );
//! Calculates next round key from current round key in 'keyBuffer' and 'roundConstant'.
static void AES_DevelopKey( uint8_t * keyBuffer, uint8_t * roundConstant );

//! For configurations where more than one round key is calculated at a time,
//! calculate the first set of round keys given the initial key in 'keyBuffer'.
#if AES_KEY_SIZE == 24
static void AES_InitialKeyExpansion( uint8_t * keyBuffer, uint8_t * roundConstant );
#endif



// ***********
// Functions only used in decrypt
#ifdef AES_INCLUDE_DECRYPT
//! Perform an AES inverse column mix operation on the 4 bytes in 'column' buffer.
static void AES_InvMixColumn( uint8_t * column );
//! Perform AES inverse column mixing for the whole AES block/state.
static void AES_InvMixColumns( uint8_t * state );
//! Perform AES inverse shift rows operation for the whole AES block/state.
static void AES_InvShiftRows( uint8_t * state );

//! Substitute 'count' bytes from 'bytes' using inverse S-Box, XOR with 'constant' and store in 'bytes'.
static void AES_InvSubstAndAddConst( uint8_t * bytes, uint8_t const * constant, uint8_t count );

//! Calculate previous round key from current round key in 'keyBuffer' and 'roundConstant'.
static void AES_InvDevelopKey( uint8_t * keyBuffer, uint8_t * roundConstant );

#endif


/******************************************************************************
 * Function implementations.
 ******************************************************************************/

/*!
 * This function takes a preloaded encryption key, AES_KEY_SIZE bytes long,
 * in a buffer of AES_KEY_BUFFER_SIZE bytes and prepares the buffer contents
 * for encryption. This particular implementation does not touch the contents,
 * but this function is provided, and should be used, to be portable with other
 * implementations of AES that might need some preparation before encrypting.
 *
 * \param  keyBuffer  Buffer of AES_KEY_BUFFER_SIZE bytes containing encryption key.
 */
void AES_PrepareEncrypt( uint8_t * keyBuffer )
{
	keyBuffer=keyBuffer; // avoid compiler warning
	return; // Do nothing for this implementation.
}


/*!
 * This function encrypts the data in 'block' using a prepared 'keyBuffer'.
 * Use the AES_PrepareEncrypt() function to preapre the 'keyBuffer' before
 * encryption.
 *
 * Note that this function will change the buffer contents, actually to the
 * state required by the AES_Decrypt() function, but that fact only holds for
 * this implementation of AES.
 *
 * If you want to do several encryptions in a row, you need to call the
 * AES_PrepareEncrypt() function before every call to this function.
 *
 * Note that for this implementation, also the decryption process will result
 * in a 'keyBuffer' suitable for encryption.
 *
 * \param  block      The block of data to be encrypted. AES_BLOCK_SIZE bytes.
 * \param  keyBuffer  Buffer prepared by AES_PrepareEncrypt, or after decryption.
 */
void AES_Encrypt( uint8_t * block, uint8_t * keyBuffer )
{
	uint8_t roundConstant[4] = { 0x01, 0x00, 0x00, 0x00 };

#if (AES_KEYS_IN_BUFFER > 1)
	AES_InitialKeyExpansion( keyBuffer, roundConstant );
#endif

#if (AES_KEY_SIZE == 16)
	for (uint8_t round = 0; round < (AES_ROUNDS - 1); ++round) {
		AES_AddConstAndSubst( block, keyBuffer, AES_BLOCK_SIZE );
		AES_ShiftRows( block );
		AES_MixColumns( block );

		AES_DevelopKey( keyBuffer, roundConstant );
	}

	AES_AddConstAndSubst( block, keyBuffer, AES_BLOCK_SIZE );
	AES_ShiftRows( block );

	AES_DevelopKey( keyBuffer, roundConstant );
	AES_AddConstant( block, keyBuffer, AES_BLOCK_SIZE );

#elif (AES_KEY_SIZE == 24)
	for (uint8_t round = 0; round < (AES_ROUNDS - 3); round += 3) {
		AES_AddConstAndSubst( block, keyBuffer + (AES_BLOCK_SIZE * 0), AES_BLOCK_SIZE );
		AES_ShiftRows( block );
		AES_MixColumns( block );

		AES_AddConstAndSubst( block, keyBuffer + (AES_BLOCK_SIZE * 1), AES_BLOCK_SIZE );
		AES_ShiftRows( block );
		AES_MixColumns( block );

		AES_AddConstAndSubst( block, keyBuffer + (AES_BLOCK_SIZE * 2), AES_BLOCK_SIZE );
		AES_ShiftRows( block );
		AES_MixColumns( block );

		AES_DevelopKey( keyBuffer, roundConstant );
	}

	AES_AddConstAndSubst( block, keyBuffer + (AES_BLOCK_SIZE * 0), AES_BLOCK_SIZE );
	AES_ShiftRows( block );
	AES_MixColumns( block );

	AES_AddConstAndSubst( block, keyBuffer + (AES_BLOCK_SIZE * 1), AES_BLOCK_SIZE );
	AES_ShiftRows( block );
	AES_MixColumns( block );

	AES_AddConstAndSubst( block, keyBuffer + (AES_BLOCK_SIZE * 2), AES_BLOCK_SIZE );
	AES_ShiftRows( block );

	AES_DevelopKey( keyBuffer, roundConstant );
	AES_AddConstant( block, keyBuffer, AES_BLOCK_SIZE );

#elif (AES_KEY_SIZE == 32)
	for (uint8_t round = 0; round < (AES_ROUNDS - 2); round += 2 ) {
		AES_AddConstAndSubst( block, keyBuffer + (AES_BLOCK_SIZE * 0), AES_BLOCK_SIZE );
		AES_ShiftRows( block );
		AES_MixColumns( block );

		AES_AddConstAndSubst( block, keyBuffer + (AES_BLOCK_SIZE * 1), AES_BLOCK_SIZE );
		AES_ShiftRows( block );
		AES_MixColumns( block );

		AES_DevelopKey( keyBuffer, roundConstant );
	}

	AES_AddConstAndSubst( block, keyBuffer + (AES_BLOCK_SIZE * 0), AES_BLOCK_SIZE );
	AES_ShiftRows( block );
	AES_MixColumns( block );

	AES_AddConstAndSubst( block, keyBuffer + (AES_BLOCK_SIZE * 1), AES_BLOCK_SIZE );
	AES_ShiftRows( block );

	AES_DevelopKey( keyBuffer, roundConstant );
	AES_AddConstant( block, keyBuffer, AES_BLOCK_SIZE );

#else
  #error Unsupported key size.
#endif
}


/*!
 * This function takes a preloaded encryption key, AES_KEY_SIZE bytes long,
 * in a buffer of AES_KEY_BUFFER_SIZE bytes and prepares the buffer contents
 * for decryption. This particular implementation actually runs through all steps
 * of the encryption process involving the key schedule itself, and ends up with
 * the buffer containing an appropriate starting point for decryption.
 *
 * \param  keyBuffer  Buffer of AES_KEY_BUFFER_SIZE bytes containing encryption key.
 */
void AES_PrepareDecrypt( uint8_t * keyBuffer )
{
	uint8_t roundConstant[4] = { 0x01, 0x00, 0x00, 0x00 };

#if (AES_KEYS_IN_BUFFER > 1)
	AES_InitialKeyExpansion( keyBuffer, roundConstant );
#endif

	for (uint8_t round = 1; round < (AES_ROUNDS + 1); round += AES_BLOCKS_IN_BUFFER ) {
		AES_DevelopKey( keyBuffer, roundConstant );
	}
}


/*!
 * This function decrypts the data in 'block' using a prepared 'keyBuffer'.
 * Use the AES_PrepareDecrypt() function to preapre the 'keyBuffer' before
 * encryption.
 *
 * Note that this function will change the buffer contents, actually to the
 * state required by the AES_Encrypt() function, but that fact only holds for
 * this implementation of AES.
 *
 * If you want to do several encryptions in a row, you need to call the
 * AES_PrepareDecrypt() function before every call to this function.
 *
 * Note that for this implementation, also the encryption process will result
 * in a 'keyBuffer' suitable for decryption.
 *
 * \param  block      The block of data to be decrypted. AES_BLOCK_SIZE bytes.
 * \param  keyBuffer  Buffer prepared by AES_PrepareDecrypt, or after encryption.
 */
#ifdef AES_INCLUDE_DECRYPT
void AES_Decrypt( uint8_t * block, uint8_t * keyBuffer )
{
	uint8_t roundConstant[4] = { AES_LAST_RCON_VALUE, 0x00, 0x00, 0x00 };

#if (AES_KEY_SIZE == 16)
	AES_AddConstant( block, keyBuffer, AES_BLOCK_SIZE );

	for (uint8_t round = 0; round < (AES_ROUNDS - 1); ++round) {
		AES_InvDevelopKey( keyBuffer, roundConstant );

		AES_InvShiftRows( block );
		AES_InvSubstAndAddConst( block, keyBuffer, AES_BLOCK_SIZE );
		AES_InvMixColumns( block );
	}

	AES_InvDevelopKey( keyBuffer, roundConstant );

	AES_InvShiftRows( block );
	AES_InvSubstAndAddConst( block, keyBuffer, AES_BLOCK_SIZE );

#elif (AES_KEY_SIZE == 24)
	// Backtrace last update of round constant, since it is never
	// used, due to the use of two AES_KEY_SIZEs in schedule buffer.
	if ((roundConstant[0] ^ AES_BPOLY) == 0) {
		roundConstant[0] = 0x80;
	} else {
		roundConstant[0] >>= 1;
	}

	AES_AddConstant( block, keyBuffer, AES_BLOCK_SIZE );

	for (uint8_t round = 0; round < (AES_ROUNDS - 3); round += 3) {
		AES_InvDevelopKey( keyBuffer, roundConstant );

		AES_InvShiftRows( block );
		AES_InvSubstAndAddConst( block, keyBuffer + (AES_BLOCK_SIZE * 2), AES_BLOCK_SIZE );
		AES_InvMixColumns( block );

		AES_InvShiftRows( block );
		AES_InvSubstAndAddConst( block, keyBuffer + (AES_BLOCK_SIZE * 1), AES_BLOCK_SIZE );
		AES_InvMixColumns( block );

		AES_InvShiftRows( block );
		AES_InvSubstAndAddConst( block, keyBuffer + (AES_BLOCK_SIZE * 0), AES_BLOCK_SIZE );
		AES_InvMixColumns( block );
	}

	AES_InvDevelopKey( keyBuffer, roundConstant );

	AES_InvShiftRows( block );
	AES_InvSubstAndAddConst( block, keyBuffer + (AES_BLOCK_SIZE * 2), AES_BLOCK_SIZE );
	AES_InvMixColumns( block );

	AES_InvShiftRows( block );
	AES_InvSubstAndAddConst( block, keyBuffer + (AES_BLOCK_SIZE * 1), AES_BLOCK_SIZE );
	AES_InvMixColumns( block );

	AES_InvShiftRows( block );
	AES_InvSubstAndAddConst( block, keyBuffer + (AES_BLOCK_SIZE * 0), AES_BLOCK_SIZE );

#elif (AES_KEY_SIZE == 32)
	AES_AddConstant( block, keyBuffer, AES_BLOCK_SIZE );

	for (uint8_t round = 0; round < (AES_ROUNDS - 2); round += 2) {
		AES_InvDevelopKey( keyBuffer, roundConstant );

		AES_InvShiftRows( block );
		AES_InvSubstAndAddConst( block, keyBuffer + AES_BLOCK_SIZE, AES_BLOCK_SIZE );
		AES_InvMixColumns( block );

		AES_InvShiftRows( block );
		AES_InvSubstAndAddConst( block, keyBuffer, AES_BLOCK_SIZE );
		AES_InvMixColumns( block );
	}

	AES_InvDevelopKey( keyBuffer, roundConstant );

	AES_InvShiftRows( block );
	AES_InvSubstAndAddConst( block, keyBuffer + AES_BLOCK_SIZE, AES_BLOCK_SIZE );
	AES_InvMixColumns( block );

	AES_InvShiftRows( block );
	AES_InvSubstAndAddConst( block, keyBuffer, AES_BLOCK_SIZE );

#else
  #error Unsupported key size.
#endif
}
#endif


static void AES_AddConstant( uint8_t * bytes, uint8_t const * constant, uint8_t count )
{
	// Copy to temporary variables for optimization.
	uint8_t * tempBlock = bytes;
	uint8_t const * tempSource = constant;
	uint8_t tempCount = count;
	uint8_t tempValue;

	do {
		 // Add in GF(2), ie. XOR.
		tempValue = *tempBlock ^ *tempSource++;
		*tempBlock++ = tempValue;
	} while (--tempCount);
}


static void AES_CopyBytes( uint8_t * destination, uint8_t const * source, uint8_t count)
{
	// Copy to temporary variables for optimization.
	uint8_t * tempDest = destination;
	uint8_t const * tempSrc = source;
	uint8_t tempCount = count;

	do {
		*tempDest++ = *tempSrc++;
	} while (--tempCount);
}


static void AES_CycleLeft( uint8_t * row )
{
	// Cycle 4 bytes in an array left once.
	uint8_t temp = row[0];
	row[0] = row[1];
	row[1] = row[2];
	row[2] = row[3];
	row[3] = temp;
}


static void AES_MixColumn( uint8_t * column )
{
	uint8_t result0, result1, result2, result3;
	uint8_t column0, column1, column2, column3;
	uint8_t xor;

	// This generates more effective code, at least
	// with the IAR C compiler.
	column0 = column[0];
	column1 = column[1];
	column2 = column[2];
	column3 = column[3];

	// Partial sums (modular addition using XOR).
	result0 = column1 ^ column2 ^ column3;
	result1 = column0 ^ column2 ^ column3;
	result2 = column0 ^ column1 ^ column3;
	result3 = column0 ^ column1 ^ column2;

	// Multiply column bytes by 2 modulo AES_BPOLY.	
	xor = 0;
	if (column0 & 0x80) {
		xor = AES_BPOLY;
	}
	column0 <<= 1;
	column0  ^= xor;
	
	xor = 0;
	if (column1 & 0x80) {
		xor = AES_BPOLY;
	}
	column1 <<= 1;
	column1  ^= xor;
	
	xor = 0;
	if (column2 & 0x80) {
		xor = AES_BPOLY;
	}
	column2 <<= 1;
	column2  ^= xor;
	
	xor = 0;
	if (column3 & 0x80) {
		xor = AES_BPOLY;
	}
	column3 <<= 1;
	column3  ^= xor;

	// Final sums stored into original column bytes.
	column[0] = result0 ^ column0 ^ column1;
	column[1] = result1 ^ column1 ^ column2;
	column[2] = result2 ^ column2 ^ column3;
	column[3] = result3 ^ column0 ^ column3;
}


static void AES_MixColumns( uint8_t * state )
{
	AES_MixColumn( state + (0 * 4) );
	AES_MixColumn( state + (1 * 4) );
	AES_MixColumn( state + (2 * 4) );
	AES_MixColumn( state + (3 * 4) );
}


static void AES_SubBytes( uint8_t * bytes, uint8_t count )
{
	// Copy to temporary variables for optimization.
	uint8_t * tempPtr = bytes;
	uint8_t tempCount = count;

	do {
		*tempPtr = PGM_READ_BYTE( AES_sBox + *tempPtr ); // Substitute every byte in state.
		++tempPtr;
	} while (--tempCount);
}


static void AES_ShiftRows( uint8_t * state )
{
	uint8_t temp;

	// Note: State is arranged column by column.

	// Cycle second row left one time.
	temp = state[1 + (0 * 4)];
	state[1 + (0 * 4)] = state[1 + (1 * 4)];
	state[1 + (1 * 4)] = state[1 + (2 * 4)];
	state[1 + (2 * 4)] = state[1 + (3 * 4)];
	state[1 + (3 * 4)] = temp;

	// Cycle third row left two times.
	temp = state[2 + (0 * 4)];
	state[2 + (0 * 4)] = state[2 + (2 * 4)];
	state[2 + (2 * 4)] = temp;
	temp = state[2 + (1 * 4)];
	state[2 + (1 * 4)] = state[2 + (3 * 4)];
	state[2 + (3 * 4)] = temp;

	// Cycle fourth row left three times, ie. right once.
	temp = state[3 + (3 * 4)];
	state[3 + (3 * 4)] = state[3 + (2 * 4)];
	state[3 + (2 * 4)] = state[3 + (1 * 4)];
	state[3 + (1 * 4)] = state[3 + (0 * 4)];
	state[3 + (0 * 4)] = temp;
}

#ifdef AES_INCLUDE_DECRYPT
static void AES_InvMixColumn( uint8_t * column )
{
	uint8_t result0, result1, result2, result3;
	uint8_t column0, column1, column2, column3;
	uint8_t xor;
	
	// This generates more effective code, at least
	// with the IAR C compiler.
	column0 = column[0];
	column1 = column[1];
	column2 = column[2];
	column3 = column[3];

	// Partial sums (modular addition using XOR).
	result0 = column1 ^ column2 ^ column3;
	result1 = column0 ^ column2 ^ column3;
	result2 = column0 ^ column1 ^ column3;
	result3 = column0 ^ column1 ^ column2;

	// Multiply column bytes by 2 modulo AES_BPOLY.
	xor = 0;
	if (column0 & 0x80) {
		xor = AES_BPOLY;
	}
	column0 <<= 1;
	column0  ^= xor;
	
	xor = 0;
	if (column1 & 0x80) {
		xor = AES_BPOLY;
	}
	column1 <<= 1;
	column1  ^= xor;
	
	xor = 0;
	if (column2 & 0x80) {
		xor = AES_BPOLY;
	}
	column2 <<= 1;
	column2  ^= xor;
	
	xor = 0;
	if (column3 & 0x80) {
		xor = AES_BPOLY;
	}
	column3 <<= 1;
	column3  ^= xor;

	// More partial sums.
	result0 ^= column0 ^ column1;
	result1 ^= column1 ^ column2;
	result2 ^= column2 ^ column3;
	result3 ^= column0 ^ column3;

	// Multiply by 2.
	xor = 0;
	if (column0 & 0x80) {
		xor = AES_BPOLY;
	}
	column0 <<= 1;
	column0  ^= xor;
	
	xor = 0;
	if (column1 & 0x80) {
		xor = AES_BPOLY;
	}
	column1 <<= 1;
	column1  ^= xor;
	
	xor = 0;
	if (column2 & 0x80) {
		xor = AES_BPOLY;
	}
	column2 <<= 1;
	column2  ^= xor;
	
	xor = 0;
	if (column3 & 0x80) {
		xor = AES_BPOLY;
	}
	column3 <<= 1;
	column3  ^= xor;

	// More partial sums.
	result0 ^= column0 ^ column2;
	result1 ^= column1 ^ column3;
	result2 ^= column0 ^ column2;
	result3 ^= column1 ^ column3;

	// Multiply by 2.
	xor = 0;
	if (column0 & 0x80) {
		xor = AES_BPOLY;
	}
	column0 <<= 1;
	column0  ^= xor;
	
	xor = 0;
	if (column1 & 0x80) {
		xor = AES_BPOLY;
	}
	column1 <<= 1;
	column1  ^= xor;
	
	xor = 0;
	if (column2 & 0x80) {
		xor = AES_BPOLY;
	}
	column2 <<= 1;
	column2  ^= xor;
	
	xor = 0;
	if (column3 & 0x80) {
		xor = AES_BPOLY;
	}
	column3 <<= 1;
	column3  ^= xor;
	
	// Final partial sum.
	column0 ^= column1 ^ column2 ^ column3;

	// Final sums stored indto original column bytes.
	column[0] = result0 ^ column0;
	column[1] = result1 ^ column0;
	column[2] = result2 ^ column0;
	column[3] = result3 ^ column0;
}
#endif

#ifdef AES_INCLUDE_DECRYPT
static void AES_InvMixColumns( uint8_t * state )
{
	AES_InvMixColumn( state + (0 * 4) );
	AES_InvMixColumn( state + (1 * 4) );
	AES_InvMixColumn( state + (2 * 4) );
	AES_InvMixColumn( state + (3 * 4) );
}
#endif

#ifdef AES_INCLUDE_DECRYPT
static void AES_InvShiftRows( uint8_t * state )
{
	uint8_t temp;

	// Note: State is arranged column by column.

	// Cycle second row right one time.
	temp = state[1 + (3 * 4)];
	state[1 + (3 * 4)] = state[1 + (2 * 4)];
	state[1 + (2 * 4)] = state[1 + (1 * 4)];
	state[1 + (1 * 4)] = state[1 + (0 * 4)];
	state[1 + (0 * 4)] = temp;

	// Cycle third row right two times.
	temp = state[2 + (0 * 4)];
	state[2 + (0 * 4)] = state[2 + (2 * 4)];
	state[2 + (2 * 4)] = temp;
	temp = state[2 + (1 * 4)];
	state[2 + (1 * 4)] = state[2 + (3 * 4)];
	state[2 + (3 * 4)] = temp;

	// Cycle fourth row right three times, ie. left once.
	temp = state[3 + (0 * 4)];
	state[3 + (0 * 4)] = state[3 + (1 * 4)];
	state[3 + (1 * 4)] = state[3 + (2 * 4)];
	state[3 + (2 * 4)] = state[3 + (3 * 4)];
	state[3 + (3 * 4)] = temp;
}
#endif


#if AES_KEY_SIZE == 24
static void AES_AddConstAndCopy(
		uint8_t * bytes,
		uint8_t const * constant,
		uint8_t * destination,
		uint8_t count )
{
	// Copy to temporary variables for optimization.
	uint8_t * tempBlock = bytes;
	uint8_t const * tempSource = constant;
	uint8_t * tempDestination = destination;
	uint8_t tempCount = count;
	uint8_t tempValue;

	do {
		 // Add in GF(2), ie. XOR.
		tempValue = *tempBlock ^ *tempSource++;
		*tempBlock++ = tempValue;
		*tempDestination++ = tempValue;
	} while (--tempCount);
}
#endif

#if (AES_KEY_SIZE == 16 || AES_KEY_SIZE == 32)
static void AES_AddAndCopy( uint8_t * buf1, uint8_t * buf2, uint8_t count )
{
	// Copy to temporary variables for optimization.
	uint8_t * tempBuf1 = buf1;
	uint8_t * tempBuf2 = buf2;
	uint8_t tempCount = count;
	uint8_t tempValue;

	do {
		 // Add in GF(2), ie. XOR.
		tempValue = *tempBuf1 ^ *tempBuf2;
		*tempBuf1++ = tempValue;
		*tempBuf2++ = tempValue;
	} while (--tempCount);
}
#endif


/* Commented out because it is never used
static void AES_AddAndStore(
		uint8_t const * buf1,
		uint8_t const * buf2,
		uint8_t * destination,
		uint8_t count )
{
	// Copy to temporary variables for optimization.
	uint8_t const * tempBuf1 = buf1;
	uint8_t const * tempBuf2 = buf2;
	uint8_t * tempDestination = destination;
	uint8_t tempCount = count;
	uint8_t tempValue;

	do {
		 // Add in GF(2), ie. XOR.
		tempValue = *tempBuf1++ ^ *tempBuf2++;
		*tempDestination++ = tempValue;
	} while (--tempCount);
}
*/


static void AES_AddConstAndSubst( uint8_t * bytes, uint8_t const * constant, uint8_t count )
{
	// Copy to temporary variables for optimization.
	uint8_t * tempDestination = bytes;
	uint8_t const * tempSource = constant;
	uint8_t tempCount = count;
	uint8_t tempValue;

	do {
		// Add in GF(2), ie. XOR.
		tempValue = *tempDestination ^ *tempSource++;
		*tempDestination++ = PGM_READ_BYTE( AES_sBox + tempValue );
	} while (--tempCount);
}

#ifdef AES_INCLUDE_DECRYPT
static void AES_InvSubstAndAddConst( uint8_t * bytes, uint8_t const * constant, uint8_t count )
{
	// Copy to temporary variables for optimization.
	uint8_t * tempDestination = bytes;
	uint8_t const * tempSource = constant;
	uint8_t tempCount = count;
	uint8_t tempValue;

	do {
		// Add in GF(2), ie. XOR.
		tempValue = *tempDestination;
		*tempDestination++ = PGM_READ_BYTE( AES_invSBox + tempValue ) ^ *tempSource++;
	} while (--tempCount);
}
#endif

static void AES_DevelopKey( uint8_t * keyBuffer, uint8_t * roundConstant )
{
	uint8_t tempWord[4];
	uint8_t schedulePos = 0;
	uint8_t xor;
	
	// Get last word from previous schedule buffer.
	AES_CopyBytes( tempWord, keyBuffer + AES_KEY_BUFFER_SIZE - 4, 4 );

	// Transform it, since we are at a AES_KEY_SIZE boundary in the schedule.
	AES_CycleLeft( tempWord );
	AES_SubBytes( tempWord, 4 );
	AES_AddConstant( tempWord, roundConstant, 4 );

	// Update round constant.
	xor = 0;
	if (roundConstant[0] & 0x80) {
		xor = AES_BPOLY;
	}
	roundConstant[0] <<= 1;
	roundConstant[0]  ^= xor;
	

#if ((AES_KEY_SIZE == 16) || (AES_KEY_SIZE == 32))

#if (AES_KEY_SIZE == 32)
	do {
		// Add value from one AES_KEY_SIZE backwards, ie. in last schedule buffer.
		// Store in buffer, replacing old value, which was one AES_KEY_SIZE backwards.
		AES_AddAndCopy( tempWord, keyBuffer, 4 );

		// Move to next word in schedule buffer.
		keyBuffer += 4;
		schedulePos += 4;

	} while (schedulePos < AES_BLOCK_SIZE);

	// Substitute previous word when at AES_BLOCK_SIZE boundary with 256 bit keys.
	AES_SubBytes( tempWord, 4 );
#endif

	do {
		// Add value from one AES_KEY_SIZE backwards, ie. in last schedule buffer.
		// Store in buffer, replacing old value, which was one AES_KEY_SIZE backwards.
		AES_AddAndCopy( tempWord, keyBuffer, 4 );

		// Move to next word in schedule buffer.
		keyBuffer += 4;
		schedulePos += 4;

	} while (schedulePos < AES_KEY_BUFFER_SIZE);

#elif (AES_KEY_SIZE == 24)
	do {
		// Add value from one AES_KEY_SIZE backwards (wraps to end of buffer).
		// Store in buffer, replacing old value.
		AES_AddConstAndCopy( tempWord, keyBuffer + AES_KEY_SIZE, keyBuffer, 4 );

		// Move to next word in schedule buffer.
		keyBuffer += 4;
		schedulePos += 4;

	} while (schedulePos < AES_KEY_SIZE);

	// Transform previous word, since we are at a AES_KEY_SIZE boundary in the schedule.
	AES_CycleLeft( tempWord );
	AES_SubBytes( tempWord, 4 );
	AES_AddConstant( tempWord, roundConstant, 4 );

	// Update round constant.
	xor = 0;
	if (roundConstant[0] & 0x80) {
		xor = AES_BPOLY;
	}
	roundConstant[0] <<= 1;
	roundConstant[0]  ^= xor;
	
	do {
		// Add value from one AES_KEY_SIZE backwards.
		// Store in buffer, replacing old value.
		AES_AddConstAndCopy( tempWord, keyBuffer - AES_KEY_SIZE, keyBuffer, 4 );

		// Move to next word in schedule buffer.
		keyBuffer += 4;
		schedulePos += 4;

	} while (schedulePos < AES_KEY_BUFFER_SIZE);

#else
  #error Unsupported key size.
#endif
}


#if (AES_KEY_SIZE == 24) // Currently only required for 192-bit keys.
static void AES_InitialKeyExpansion( uint8_t * keyBuffer, uint8_t * roundConstant )
{
	uint8_t tempWord[4];
	uint8_t xor;
	uint8_t schedulePos = AES_KEY_BUFFER_SIZE / 2;
	keyBuffer += AES_KEY_BUFFER_SIZE / 2;

	// Get last word from previous key in schedule buffer.
	AES_CopyBytes( tempWord, keyBuffer - 4, 4 );

	// Transform it, since we are at a AES_KEY_SIZE boundary in the schedule.
	AES_CycleLeft( tempWord );
	AES_SubBytes( tempWord, 4 );
	AES_AddConstant( tempWord, roundConstant, 4 );

	// Update round constant.
	xor = 0;
	if (roundConstant[0] & 0x80) {
		xor = AES_BPOLY;
	}
	roundConstant[0] <<= 1;
	roundConstant[0]  ^= xor;

	do {
		// Add value from one AES_KEY_SIZE backwards.
		// Store in buffer.
		AES_AddConstAndCopy( tempWord, keyBuffer - AES_KEY_SIZE, keyBuffer, 4 );

		// Move to next word in schedule buffer.
		keyBuffer += 4;
		schedulePos += 4;

	} while (schedulePos < AES_KEY_BUFFER_SIZE);
}
#endif


#ifdef AES_INCLUDE_DECRYPT
static void AES_InvDevelopKey( uint8_t * keyBuffer, uint8_t * roundConstant )
{
	uint8_t tempWord[4];

#if ((AES_KEY_SIZE == 16) || (AES_KEY_SIZE == 32))
	uint8_t schedulePos = AES_KEY_BUFFER_SIZE - 4;
	keyBuffer += AES_KEY_BUFFER_SIZE - 4;

#if (AES_KEY_SIZE == 32)
	do {
		// Add with previous word.
		AES_AddConstant( keyBuffer, keyBuffer - 4, 4 );

		// Move to previous word in schedule.
		keyBuffer -= 4;
		schedulePos -= 4;

	} while (schedulePos > AES_BLOCK_SIZE);

	// Prepare substitution of previous word.
	AES_CopyBytes( tempWord, keyBuffer - 4, 4 );
	AES_SubBytes( tempWord, 4 );

	// Add substituted word to current word.
	AES_AddConstant( keyBuffer, tempWord, 4 );

	// Move to previous word in schedule.
	keyBuffer -= 4;
	schedulePos -= 4;
#endif

	do {
		// Add with previous word.
		AES_AddConstant( keyBuffer, keyBuffer - 4, 4 );

		// Move to previous word in schedule.
		keyBuffer -= 4;
		schedulePos -= 4;

	} while (schedulePos > 0);

	// Prepare round constant for transformation that follows.
	if ((roundConstant[0] ^ AES_BPOLY) == 0) {
		roundConstant[0] = 0x80;
	} else {
		roundConstant[0] >>= 1;
	}

	// Prepare transformation of previous word (which is now at end of buffer).
	AES_CopyBytes( tempWord, keyBuffer + AES_KEY_BUFFER_SIZE - 4, 4 );
	AES_CycleLeft( tempWord );
	AES_SubBytes( tempWord, 4 );
	AES_AddConstant( tempWord, roundConstant, 4 );

	// Apply transformation result to current word.
	AES_AddConstant( keyBuffer, tempWord, 4 );

#elif (AES_KEY_SIZE == 24)
	// Move to end of first AES_KEY_SIZE in schedule.
	uint8_t schedulePos = (AES_KEY_BUFFER_SIZE/2) - 4;
	keyBuffer += (AES_KEY_BUFFER_SIZE/2) - 4;

	do {
		// Get current word.
		// Add with previous word.
		// Store at position one AES_KEY_SIZE backwards (wraps to end of buffer).
		AES_AddAndStore( keyBuffer, keyBuffer - 4, keyBuffer + AES_KEY_SIZE, 4 );

		// Move to previous word in schedule.
		keyBuffer -= 4;
		schedulePos -= 4;

	} while (schedulePos > 0);

	// Prepare round constant for transformation that follows.
	if ((roundConstant[0] ^ AES_BPOLY) == 0) {
		roundConstant[0] = 0x80;
	} else {
		roundConstant[0] >>= 1;
	}

	// Prepare transformation of previous word (which is now at end of buffer).
	AES_CopyBytes( tempWord, keyBuffer + AES_KEY_BUFFER_SIZE - 4, 4 );
	AES_CycleLeft( tempWord );
	AES_SubBytes( tempWord, 4 );
	AES_AddConstant( tempWord, roundConstant, 4 );

	// Add current word to transformation result.
	AES_AddConstant( tempWord, keyBuffer, 4 );
	// Store at position one AES_KEY_SIZE backwards (wraps to end of buffer).
	AES_CopyBytes( keyBuffer + AES_KEY_SIZE, tempWord, 4 );

	// Move to last word in schedule.
	schedulePos = AES_KEY_BUFFER_SIZE - 4;
	keyBuffer += AES_KEY_BUFFER_SIZE - 4;

	do {
		// Get current word.
		// Add with previous word.
		// Store at position one AES_KEY_SIZE backwards.
		AES_AddAndStore( keyBuffer, keyBuffer - 4, keyBuffer - AES_KEY_SIZE, 4 );

		// Move to previous word in schedule.
		keyBuffer -= 4;
		schedulePos -= 4;

	} while (schedulePos > AES_KEY_SIZE);

	// Prepare round constant for transformation that follows.
	if ((roundConstant[0] ^ AES_BPOLY) == 0) {
		roundConstant[0] = 0x80;
	} else {
		roundConstant[0] >>= 1;
	}

	// Prepare transformation of previous word.
	AES_CopyBytes( tempWord, keyBuffer - 4, 4 );
	AES_CycleLeft( tempWord );
	AES_SubBytes( tempWord, 4 );
	AES_AddConstant( tempWord, roundConstant, 4 );

	// Add current word to transformation result.
	AES_AddConstant( tempWord, keyBuffer, 4 );
	// Store at position one AES_KEY_SIZE backwards.
	AES_CopyBytes( keyBuffer - AES_KEY_SIZE, tempWord, 4 );

#else
  #error Unsupported key size.
#endif
}
#endif

#endif // AUTH_USE_AES

// end of file
