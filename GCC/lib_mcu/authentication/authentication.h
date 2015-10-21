/* This file has been prepared for Doxygen automatic documentation generation.*/
/*! \file *********************************************************************
 *
 * \brief
 *      Headerfile for authentication.c.
 *
 *      Contains defines, typedefinitions and functions prototypes for authentication.c.
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

#ifndef AUTHENTICATION_H
#define AUTHENTICATION_H

#include "common.h"
#include "sbs_commands.h"
#include "battery_pack_parameters.h"

/********************
 Types
 ********************/
enum AUTH_Status {
	/*! 
	 * Should be the value of the status flag when sent from host, and is 
	 * returned if the battery hasn't started the authentication algorithm yet.
	 */
	AUTH_Status_Start = 0x01,
	
	/*!
	 * Status during execution
	 */
	AUTH_Status_Executing = 0x02,
	
	/*!
	 * The authentication algorithm is finished and the data ready to send to host
	 */
	AUTH_Status_Finished = 0x04,
	
	/*!
	 * Status if any error occurs (depends on crypto used)
	 */
	AUTH_Status_Error = 0x08
};


/******************************************************************************
 Proto types
 ******************************************************************************/
#if defined(AUTH_USE_AES)
void AUTH_CopyKeyToSram();
#endif
SBS_command_t* AUTH_GetResponse( void );
void AUTH_Execute( SBS_command_t* sbsCmd );

#endif
