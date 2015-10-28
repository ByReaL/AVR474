/* This file has been prepared for Doxygen automatic documentation generation.*/
/*! \file *********************************************************************
 *
 * \brief
 *      Contains a function for calculating the CRC for polynomial:
 *      x^16 + x^15 + x^2 + 1
 *
 * \par Application note:
 *      AVR474: SB202 Firmware.
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
#include "crc16.h"
#include "ATmega32HVB_signature.h"
#include "iar_compat.h"
/****************************
 * CRC-16 calculation
 *
 * Polynomial: x^16 + x^15 + x^2 + 1
 *
 * \param  crc  CRC checksum to update
 * \param data  Data to update CRC with
 *
 * \return The new checksum
 */
uint16_t crc16_update(uint16_t crc, uint16_t data)
{
	crc ^= data;
	for (uint8_t i = 0; i < 8; ++i)
	{
		if (crc & 1) {
			crc = (crc >> 1) ^ 0xA001;
		} else {
			crc >>= 1;
		}
	}
	
	return crc;
}

/*!
 * CRC16 implementation from Israel Echevarria that is used for the signature checksum
 */
uint16_t crc16_from_atmel(uint16_t crc, uint16_t data)
{
	uint8_t data_bit;
	uint8_t crc_msb;
	
	for(int8_t bit = 15; bit >= 0; --bit) {
		data_bit = (data >> bit) & 1;
		crc_msb = (crc >> 15) & 1;
		crc = (((crc <<1)+ data_bit) ^ (crc_msb << 15) ^ (crc_msb << 2) ^ (crc_msb << 0));
	}
	
	return crc;
}

/*! \brief Check if the CRC of the data in the signature row is correct
 *
 * \retval  true  CRC is correct
 * \retval false  CRC is not correct
 */
bool CheckSignatureCRC()
{
	uint16_t crc = 0; // Start value is zero for this implementation
	
	uint8_t addr;
	uint16_t data;
	
	uint8_t interrupt_status = __save_interrupt();  // Disable interrupts.
	__disable_interrupt();
	
	for(addr = 0; addr <= 0x1D; ++addr) {
		data = READ_SIGNATUREWORD((addr*2));
		crc = CRC_16_Atmel(crc, data);
	}
	
	uint16_t correct_crc = READ_SIGNATUREWORD(SIG_CRC16_2ND_CAL_L);
	
	__restore_interrupt(interrupt_status);
	
	if(correct_crc == crc) {
		return true;
	} else {
		return false;
	}
}
