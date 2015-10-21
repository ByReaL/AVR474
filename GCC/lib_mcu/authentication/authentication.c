/* This file has been prepared for Doxygen automatic documentation generation.*/
/*! \file *********************************************************************
 *
 * \brief
 *      Functions for taking care of the authentication command
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

#include <stdint.h>
#include "authentication.h"
#include "sbs_commands.h"
#include "battery_pack_parameters.h"

#if defined(AUTH_USE_HMAC_SHA2)
  #include "HMAC-SHA2/HMAC-SHA2.h"

#elif defined(AUTH_USE_AES)
// AES_KEY_SIZE needs to be defined before importing the AES functions, because they
// are somewhat different depending on key size
  #include "aes/aes_lib.h"

#endif

/************************
Static functions
*************************/
#ifdef AUTH_USE_HMAC_SHA2
static void HMAC_SHA2_swapEndianess(uint32_t* block, uint8_t size);
#endif

/************************
Internal variables
************************/
//! Transformed data is saved here, including status byte
SBS_command_t authenticationResponse;

//**********
// AES
#if defined(AUTH_USE_AES)
//! Key saved in sram at startup instead of eeprom
uint8_t key[AES_KEY_SIZE];

//! The key changes during encryption so it has to be moved to a buffer
uint8_t key_buffer[AES_KEY_BUFFER_SIZE]; // if AES_KEY_SIZE = 16, AES_KEY_BUFFER_SIZE = 16. But if key size is 24 or 32, buffer size is 72 or 64

//***********
// HMAC-SHA2
#elif defined(AUTH_USE_HMAC_SHA2)
union{
	uint32_t ul_message[16];
	uint8_t uc_message[64];
} u_message;
#endif


/*****************
 Function implemenentations
 *****************/


/*! \brief Copy key from eeprom to sram
 *
 */
#if defined(AUTH_USE_AES)
void AUTH_CopyKeyToSram()
{
	for(uint8_t i = 0; i < AES_KEY_SIZE; ++i) {
		key[i] = AUTH_aes_key[i];
	}
}
#endif

/*! \brief Return pointer to authentication response
 *
 *	\return Pinter to response
 *
 */
SBS_command_t* AUTH_GetResponse( void )
{
	return &authenticationResponse;
}

#ifdef AUTH_USE_HMAC_SHA2
/*! \brief Swap endianess for HMAC-SHA2
 *
 * \param block Pointer to data array consisting of "size" 32-bit variables
 * \param size  Size of block in 32-bit variables
 */
static void HMAC_SHA2_swapEndianess(uint32_t * block, uint8_t size)
{
	uint8_t * block8 = (uint8_t*)block;
	
	uint8_t temp;
	uint8_t i;
	for(i = 0; i < size; ++i) {
		temp = block8[4*i + 3];
		block8[4*i + 3] = block8[4*i];
		block8[4*i] = temp;
		
		temp = block8[4*i + 2];
		block8[4*i + 2] = block8[4*i + 1];
		block8[4*i + 1] = temp;
	}
}
#endif


/*! \brief Run the chosen authentication algorithm
 *
 * Runs the authentication algorithm on the data in sbsCmd and stores the result
 * in authenticationResponse to be retrieved by AUTH_GetResponse().
 *
 * The data in sbsCmd will be destroyed
 *
 * Only runs the algorithm if the status byte is AUTH_Status_Start.
 *
 * \param sbsCmd Pointer to SBS command parameters
 */
void AUTH_Execute( SBS_command_t* sbsCmd )
{	
	uint8_t i;
	
	// Only run if status byte (the first byte) says so
	if(AUTH_Status_Start == sbsCmd->payload[0]) {
		
//******************************************************************************
// AES Encryption Start
//******************************************************************************
#		if defined(AUTH_USE_AES)

		
		// Copy the key  to the key buffer
		for(i = 0; i < AES_KEY_SIZE; ++i) {
			key_buffer[i] = key[i];
		}
		
		// Do the encryption
		AES_PrepareEncrypt( key_buffer ); // Does nothing, but should be called anyway if that changes in the future
		AES_Encrypt( &sbsCmd->payload[1], key_buffer );
		
		sbsCmd->payload[0] = AUTH_Status_Finished;
		
		// Store the encrypted message internally for later use
		// The variable "i" is used later to fill the rest of the payload with zeros
		for( i = 0; i <= AES_BLOCK_SIZE; i++)
		{
			authenticationResponse.payload[i] = sbsCmd->payload[i];
		}
		
		for( ; i < SBSDATA_BLOCK_LENGTH; ++i)
		{
			authenticationResponse.payload[i] = 0x00;
		}	
		
//******************************************************************************
// AES Encryption End
//******************************************************************************
		
//******************************************************************************
// HMAC-SHA2 Encryption Start
//******************************************************************************
#		elif defined(AUTH_USE_HMAC_SHA2)
		
		// Copy the input data to the message buffer
		// If SHA_INPUT_BLOCK_SIZE is SBSDATA_BLOCK_LENGTH include
		// the status byte, otherwise skip it
		for( i = 0; i < SHA_INPUT_BLOCK_SIZE; ++i) {
#			if SHA_INPUT_BLOCK_SIZE == SBSDATA_BLOCK_LENGTH
				u_message.uc_message[i] = sbsCmd->payload[i];
#			else
				u_message.uc_message[i] = sbsCmd->payload[i+1];
#			endif
		}
		
		// Swap endianess of the message to comply with the standard
		HMAC_SHA2_swapEndianess(u_message.ul_message, SHA_INPUT_BLOCK_WORDS);
		
		// Run the algorithm
		HMAC_SHA2( u_message.ul_message, (uint32_t*)sbsCmd->payload );
		
		// Swap endianess of the result to comply with the standard
		// 32-bytes (256-bits, 8 32-bit words) is always the result from the HMAC-SHA2 algorithm
		HMAC_SHA2_swapEndianess((uint32_t*)sbsCmd->payload, 8);
		
		// Store the response internally for later use
		// Only store the 31 last bytes, as the first byte is a status byte.
		for( i = 1; i < 32; i++)
		{
			authenticationResponse.payload[i] = sbsCmd->payload[i-1];
		}
		
		// Mark it as finished
		authenticationResponse.payload[0] = AUTH_Status_Finished;
		
#		endif // If AUTH_USE_AES/AUTH_USE_HMAC_SHA2
//******************************************************************************
// HMAC-SHA2 Encryption End
//******************************************************************************

		
	} else {
		// Set payload to only return the status, which is "error"
		authenticationResponse.payload[0] = AUTH_Status_Error;
		for(i = 1 ; i < SBSDATA_BLOCK_LENGTH; ++i)
		{
			authenticationResponse.payload[i] = 0x00;
		}	
	}
}
