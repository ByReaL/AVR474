// This file has been prepared for Doxygen automatic documentation generation.
/*! \file ********************************************************************
*
* Atmel Corporation
*
* - File              : SMBslave.h
* - Compiler          : IAR EWAAVR 4.10b
*
* - Support mail      : avr@atmel.com
*
* - Supported devices : All AVR devices with a TWI module can be used.
*                       The example is written for ATmega32
*
* - AppNote           : AVR316 - SMBus slave
*
* - Description       : Header file for SMBExample.c.
*
* $Revision: 5627 $
* $Date: 2009-05-15 14:49:21 +0800 (Fri, 15 May 2009) $
*****************************************************************************/

#ifndef __SMB_EXAMPLE_H__
#define __SMB_EXAMPLE_H__

#include "SMBSlave.h"

// Command codes used in this example implementation.
#define SMB_COMMAND_RETURN_VENDOR_STRING    0x10
#define SMB_COMMAND_RETURN_SWITCHES         0x20
#define SMB_COMMAND_SET_EE_POINTER          0x30
#define SMB_COMMAND_READ_EE_BYTE            0x40
#define SMB_COMMAND_READ_EE_WORD            0x41
#define SMB_COMMAND_WRITE_LED_BYTE          0x50
#define SMB_COMMAND_WRITE_LED_WORD          0x51
#define SMB_COMMAND_WRITE_LED_BLOCK         0x52
#define SMB_COMMAND_MULTIPLY_BY_TWO         0x60
#define SMB_COMMAND_SUM_OF_BYTES            0x70

void ProcessReceiveByte(SMBData *smb);
void ProcessMessage(SMBData *smb);

void ReturnVendorString(SMBData *smb);
void ReturnSwitchesPressed(SMBData *smb);
void SetEepromPointer(SMBData *smb);
void ReadEepromByte(SMBData *smb);
void ReadEepromWord(SMBData *smb);
void OutputLedByte(SMBData *smb);
void OutputLedWord(SMBData *smb);
void OutputLedBlock(SMBData *smb);
void ReturnParameterTimesTwo(SMBData *smb);
void ReturnSumOfBytes(SMBData *smb);
void UndefinedCommand(SMBData *smb);


#endif
