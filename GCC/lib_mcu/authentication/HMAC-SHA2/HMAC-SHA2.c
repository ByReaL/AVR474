/*This file has been prepared for Doxygen automatic documentation generation.*/
/*! \file *********************************************************************
 *
 * \brief
 *     Routines for SHA-256 and HMAC-SHA256
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
 * $Revision: 6096 $
 * $Date: 2009-10-06 19:06:15 +0800 (Tue, 06 Oct 2009) $  \n
 *
 * Copyright (c) 2008, Atmel Corporation All rights reserved.
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

/* Includes. */
#include "battery_pack_parameters.h"

#ifdef AUTH_USE_HMAC_SHA2

#include "HMAC-SHA2.h"
#include "HMAC-SHA2_keyData.h"
#include "stdint.h"

/* Internal functions */
uint32_t RIGHTROTATE(uint32_t x, uint8_t n);
uint32_t LEFTROTATE(uint32_t x, uint8_t n);


/* File global variables. */

//! Data table containing the 64 K values for the SHA-2 implementation
const __flash uint32_t K[64] = {  0x428a2f98L, 0x71374491L, 0xb5c0fbcfL, 0xe9b5dba5L,
                            0x3956c25bL, 0x59f111f1L, 0x923f82a4L, 0xab1c5ed5L,
                            0xd807aa98L, 0x12835b01L, 0x243185beL, 0x550c7dc3L,
                            0x72be5d74L, 0x80deb1feL, 0x9bdc06a7L, 0xc19bf174L,
                            0xe49b69c1L, 0xefbe4786L, 0x0fc19dc6L, 0x240ca1ccL,
                            0x2de92c6fL, 0x4a7484aaL, 0x5cb0a9dcL, 0x76f988daL,
                            0x983e5152L, 0xa831c66dL, 0xb00327c8L, 0xbf597fc7L,
                            0xc6e00bf3L, 0xd5a79147L, 0x06ca6351L, 0x14292967L,

                            0x27b70a85L, 0x2e1b2138L, 0x4d2c6dfcL, 0x53380d13L,
                            0x650a7354L, 0x766a0abbL, 0x81c2c92eL, 0x92722c85L,
                            0xa2bfe8a1L, 0xa81a664bL, 0xc24b8b70L, 0xc76c51a3L,
                            0xd192e819L, 0xd6990624L, 0xf40e3585L, 0x106aa070L,
                            0x19a4c116L, 0x1e376c08L, 0x2748774cL, 0x34b0bcb5L,
                            0x391c0cb3L, 0x4ed8aa4aL, 0x5b9cca4fL, 0x682e6ff3L,
                            0x748f82eeL, 0x78a5636fL, 0x84c87814L, 0x8cc70208L,
                            0x90befffaL, 0xa4506cebL, 0xbef9a3f7L, 0xc67178f2L};


/* Defines */
#define		a		ul_working[0]  //!< Variable used to store 'a' during the SHA-2 calculations
#define		b		ul_working[1]  //!< Variable used to store 'b' during the SHA-2 calculations
#define		c		ul_working[2]  //!< Variable used to store 'c' during the SHA-2 calculations
#define		d		ul_working[3]  //!< Variable used to store 'd' during the SHA-2 calculations
#define		e		ul_working[4]  //!< Variable used to store 'e' during the SHA-2 calculations
#define		f		ul_working[5]  //!< Variable used to store 'f' during the SHA-2 calculations
#define		g		ul_working[6]  //!< Variable used to store 'g' during the SHA-2 calculations
#define		h		ul_working[7]  //!< Variable used to store 'h' during the SHA-2 calculations


#define ROTR(x, n) RIGHTROTATE(x, n) //!< Defining the rotate right function (other implementations may cut code size/RAM usage/increase speed
#define ROTL(x, n) LEFTROTATE(x, n)  //!< Defining the rotate left function (other implementations may cut code size/RAM usage/increase speed

/**** Functions. */


/*! \brief Perform a SHA-2 operation on input data
 *
 * \param  ul_message  Data array containing the input data to the SHA-2 routine, 16 32-bit words
 * \param  ul_working  Data array for the result. 8 32-bit words
 * \param  ul_initial  Data array containing the initial values of hash, 8 32-bit works
 */

void SHA_2(uint32_t *ul_message, uint32_t *ul_working, const uint32_t __flash *ul_initial)
{
	uint8_t uc_temp, s;   //!< Variables used as temporary datastorage/counter
 	static uint32_t t1 , t2;    //!< Temporary variables, static to avoid using the stack
	
	// Set the initial values
	for(uc_temp = 0; uc_temp < 8; ++uc_temp) {
		ul_working[uc_temp] = ul_initial[uc_temp];
	}
	
	// Loop over all the 16 input values and expand on the fly to save RAM
	for(uc_temp = 0; uc_temp < 64; uc_temp++)											
	{
		s = uc_temp & 0x0F;   // Isolate the lower 4 bits for use during look up to save RAM space
		if(uc_temp >= 16)									
		{
			// From Wikipedia pseudocode: "s0 := (w[i-15] rightrotate 7) xor (w[i-15] rightrotate 18) xor (w[i-15] rightshift 3)"
			t1 = ROTR(ul_message[(s - 15) & 0x0F] , 7) ^ ROTL(ul_message[(s - 15) & 0x0F] , (32-18)) ^ (ul_message[(s - 15) & 0x0F] >> 3);
			// From Wikipedia pseudocode: "s1 := (w[i-2] rightrotate 17) xor (w[i-2] rightrotate 19) xor (w[i-2] rightshift 10)"
			t2 = ROTL(ul_message[(s - 2) & 0x0F] , (32-17)) ^ ROTL(ul_message[(s - 2) & 0x0F] , (32-19)) ^ (ul_message[(s - 2) & 0x0F] >> 10);
			// From Wikipedia pseudocode: "w[i] := w[i-16] + s0 + w[i-7] + s1"
			ul_message[s] = ul_message[s] + t1 + ul_message[(s - 7) & 0x0F] + t2;		
		}
		
		t2 = ROTR(a , 2) ^ ROTR(a , 13) ^ ROTL(a , (32-22));  // From Wikipedia pseudocode: "s0 := (a rightrotate 2) xor (a rightrotate 13) xor (a rightrotate 22)"
		t2 += (a & b) ^ (a & c) ^ (b & c);                    // From Wikipedia pseudocode: "maj := (a and b) xor (a and c) xor (b and c)"
		t1 = ROTR(e , 6) ^ ROTR(e , 11) ^ ROTL(e , (32-25));  // From Wikipedia pseudocode: "s1 := (e rightrotate 6) xor (e rightrotate 11) xor (e rightrotate 25)"
		t1 += ((e & f) ^ (~e & g));                           // From Wikipedia pseudocode: "ch := (e and f) xor ((not e) and g)"
		t1 += h + K[uc_temp] + ul_message[s];                 // From Wikipedia pseudocode: "t1 := h + s1 + ch + k[i] + w[i]"
	
		h = g;       // From Wikipedia pseudocode: "h := g"
		g = f;       // From Wikipedia pseudocode: "g := f"
		f = e;       // From Wikipedia pseudocode: "f := e"
		e = d + t1;  // From Wikipedia pseudocode: "e := d + t1"
		d = c;       // From Wikipedia pseudocode: "d := c"
		c = b;       // From Wikipedia pseudocode: "c := b"
		b = a;       // From Wikipedia pseudocode: "b := a2"
		a = t1 + t2; // From Wikipedia pseudocode: "a := t1 + t2"
	}

	// From Wikipedia pseudocode: "Hj(i)= a..h + Hj(i-1)"
	for(uc_temp = 0; uc_temp < 8; uc_temp++)
	{			
		ul_working[uc_temp] += ul_initial[uc_temp];
	}
}

/*! \brief Perform a HMAC operation on input data using SHA-2
 *
 * \param  ul_message  Data array containing the input data to the SHA-2 routine, 16 32-bit words
 * \param  ul_working  Data array containing the initial values for a,b,,h values, 8 32-bit words
 *                        Will also contain the finished digest
 *
 * Both data arrays will be altered
 */
void HMAC_SHA2(uint32_t *ul_message, uint32_t *ul_working)
{
	uint8_t uc_temp;  // Temporary variable

	// If + denotes concatenation and K the key, the HMAC is calculated this way:
	// HMAC(m) = SHA2( (K^outpad) + SHA2( (K^inpad) + message ) )
	
	// Since the key is one block size (512 bits) the digest from hashing only
	// K^inpad/outpad can be precalculated and used as initial values when hashing
	// with the message. They are stored in ful_inSHA and ful_outSHA and can be
	// generated with a utility in the "tools" folder.
	
	ul_message[SHA_INPUT_BLOCK_WORDS] = 0x80000000; // Load first padding value
	for(uc_temp = SHA_INPUT_BLOCK_WORDS+1; uc_temp < 15; uc_temp++)       // Pad remaining words with 0 (only one word needed to store length of message)
	{
		ul_message[uc_temp] = 0;
	}
	ul_message[15] = 512+8*SHA_INPUT_BLOCK_SIZE;		// Load message length, 512 bits from the key
	
	SHA_2(ul_message , ul_working, ful_inSHA);      // Perform a SHA-2 operation on the data
	
	
	for(uc_temp = 0; uc_temp < 8; uc_temp++)        // Copy result of first SHA-2 conversion
	{
		ul_message[uc_temp] = ul_working[uc_temp];    // Copy outdata from the SHA-2 into the data array before second SHA-2 operation
	}
	
	ul_message[8] = 0x80000000;   // Load first padding value
	for(uc_temp = 9; uc_temp < 15; uc_temp++)       // Pad remaining words with 0 (only one word needed to store length of message)
	{
		ul_message[uc_temp] = 0;
	}
	ul_message[15] = 512+256;         // Load message length, previous digest(256bits) + 512 bits from key
	
	SHA_2(ul_message , ul_working, ful_outSHA);   // Perform final SHA-2 operation on the data
}

#endif //AUTH_USE_HMAC_SHA2
