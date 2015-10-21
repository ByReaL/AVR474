/* This file has been prepared for Doxygen automatic documentation generation.*/
/*! \file *********************************************************************
 *
 * \brief
 *      This file contains the Communication definitions.
 *
 *
 * \par Application note:
 *      AVR456: SB201 HVA one two cell smart battery firmware 
 *
 * \par Documentation:
 *      For comprehensive code documentation, supported compilers, compiler
 *      settings and supported devices see readme.html
 *
 * \author
 *      Atmel Corporation: http://www.atmel.com \n
 *      Support email: avr@atmel.com \n
 *      Original author: \n
 *
 * $Name: $
 * $Revision: 5627 $
 * $Date: 2009-05-15 14:49:21 +0800 (Fri, 15 May 2009) $\n
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

#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include "SBS_commands.h"

//byte position in message
#define SYNC_POS 0
#define TO_POS 1
#define FROM_POS 2
#define CDE_POS 3
#define DATA_LENGTH_POS 4
#define FIRST_DATA_POS 5

//List of address/sender (using in EXP/FROM bytes)
#define ALL_GROUP 'A'
#define CHARGER 'C'
#define DB101_DISPLAY 'D'
#define ELECTRONIC_LOAD 'E'
#define MAIN_CONTROLLER 'M'
#define PC_USB 'P' 
#define SMART_BATTERY 'S'

//block message
#define BLOCK_CDE_POS   0
#define BLOCK_SIZE_POS  1

//word message
#define WORD_FIRST_DATA_POS 1

#define MSK_RW_BIT_CDE 0x7F
#define SBS_WRITE (0x00)
#define SBS_READ  (0x80)

//definition of SYNC byte
#define SYNC	'U'	 // 0x55

#define COMM_NACK 0x00
#define COMM_ACK 0xFF



void    Comm_Init(void);
bool Comm_IsIdle(void);
void Comm_SbsBlockCommandReply( SBS_command_t* sbsReply );
void Comm_SbsWordCommandReply( SBS_command_t* sbsReply );
void Comm_SendACK();
bool Comm_Handle( SBS_command_t* sbsCmdDestination );
uint8_t Comm_Flag(void);

#endif //COMMUNICATION_H
