/* This file has been prepared for Doxygen automatic documentation generation.*/
/*! \file *********************************************************************
 *
 * \brief
 *      Data structures and access methods for smart battery data set.
 *
 * \par Application note:
 *      AVR456: SB201 Firmware.
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

// Includes
#include "common.h"
#include "sbs_commands.h"
#include <avr/pgmspace.h>

const uint8_t SBSDATA_sbsReservedCommands[] PROGMEM = {0x1D, 0x1E, 0x1F,
                                        0x25, 0x26, 0x27, 0x28, 0x29, 0x2A,
                                        0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x3A, 0x3B};

const uint8_t SBSDATA_sbsExtendedCommands[] PROGMEM = {0x24, 0x2B, 0x2C, 0x2D, 0x2E};


/******************************************************************************/
/*! \brief Determines if a SBS standard command is of static string type (if not it is word type).
 *
 *	\param cmd The command that should be evaluated.
 *
 *	\return True or false - true if the command is string type.
 */
bool SBS_IsCommandStaticStringType( SBS_commands_t cmd )
{
	return ( (SBSCMD_manufacturerName == cmd) || 
	         (SBSCMD_deviceName == cmd)       || 
	         (SBSCMD_deviceChemistry == cmd)  || 
	         (SBSCMD_manufacturerData == cmd) );
}

/******************************************************************************/
/*! \brief Determines if a SBS standard command is of block data type (if not it is word type).
 *
 *	\param cmd The command that should be evaluated.
 *
 *	\return True of false - true if the command is string type.
 */
bool SBS_IsCommandBlockDataType( SBS_commands_t cmd )
{
	return ( SBSCMD_authentication == cmd );
}

/******************************************************************************/
/*! \brief Determines if a SBS standard command is writable.
 *
 *	\param cmd The command that should be evaluated.
 * 
 *	\return True of false - true if the command is writable.
 */
bool SBS_IsCommandWritable( SBS_commands_t cmd )
{
	return ( (cmd <= SBSCMD_atRate) ||			// All commands before atRate are writable
	         (SBSCMD_authentication == cmd) );  // so are the authentication command.
}

/******************************************************************************/
/*! \brief Test if a command is reserved.
 *
 *	\param cmd This is the SBS command that will be verified.
 * 
 *	\return Returns true if the command is an reserved command.
 */
bool SBS_IsCommandReserved( SBS_commands_t cmd )
{
	bool status = false;
	
	uint8_t i = 0;
	while( (i < sizeof(SBSDATA_sbsReservedCommands)) && ( false == status ) )
	{
		if( cmd == SBSDATA_sbsReservedCommands[i] )
		{
			status = true;   // Yes - the command is a reserved command!
			break;
		}
		i++;
	}
	
	return status;
}

/******************************************************************************/
/*! \brief Test if a command is an extended command.
 *
 *	\param cmd This is the SBS command that will be verified.
 *
 *	\return Returns true if the command is an extended command.
 */
bool SBS_IsCommandExtended( SBS_commands_t cmd )
{
	bool status = false;
	
	uint8_t i = 0;
	while( (i < sizeof(SBSDATA_sbsExtendedCommands)) && ( false == status ) )
	{
		if( cmd == SBSDATA_sbsExtendedCommands[i] )
		{
			status = true;   // Yes - the command is an extended command!
			break;
		}
		i++;
	}
	return status;
}

