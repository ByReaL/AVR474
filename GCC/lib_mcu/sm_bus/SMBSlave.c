// This file has been prepared for Doxygen automatic documentation generation.
/*! \file ********************************************************************
*
* Atmel Corporation
*
* - File              : SMBslave.c
* - Compiler          : IAR EWAAVR 4.10b
*
* - Support mail      : avr@atmel.com
*
* - Supported devices : All AVR devices with a TWI module can be used.
*                       The example is written for ATmega32
*
* - AppNote           : AVR316 - SMBus slave
*
* - Description       : SMBus general driver implementation. Takes care of
*                       transmission and reception of data.
*
* $Revision: 5627 $
* $Date: 2009-05-15 14:49:21 +0800 (Fri, 15 May 2009) $
*****************************************************************************/

#include <avr/io.h>
#include <avr/interrupt.h>
#include "SMBSlave.h"
//#include "SMBExample.h"
#include "device_specific.h"
#include <avr/pgmspace.h>

static void SMBcrc(unsigned char data);

SMBData smb;                        //!< SMBus driver data

/*! \brief Initialize TWI module
 *
 *  This function initializes the TWI module for SMBus operation.
 */
void SMBusInit()
{
    // Set own slave address
    TWAR = SMB_OWN_ADDRESS << 1;
    // Enable TWI-interface, enable ACK, enable interrupt, clear interrupt flag
    TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWIE) | (1 << TWEA);
}


/*! \brief Enable the SMBus driver
 *
 * This function enables communications involving this device on the SMBus.
 */
void SMBEnable(void)
{
    smb.enable = TRUE;
}


/*! \brief Disable the SMBus driver
 *
 * This function disables communications involving this device on the SMBus.
 * Note that the slave address will still be ACKed according to the SMBus
 * 2.0 specification.
 */
void SMBDisable(void)
{
    smb.enable = FALSE;
}


/*! \brief Returns the error flag for the last message
 *
 *  Returns the error flag for the last SMBus communication.
 *
 *  \retval 1  There was an error in the last communication
 *  \retval 0  The last SMBus communication was successful.
 */
unsigned char SMBError(void)
{
    return smb.error;
}


 /*! \brief TWI interrupt routine
  *
  * The TWI interrupt routine
  */
#pragma vector = TWI_VECT
__interrupt void SMBus_ISR(void)
{
    unsigned char enableACK;
    unsigned char temp;

    // Enable ACKing if not explicitly disabled later in this ISR.
    enableACK = TRUE;

    // Is this the start of a protocol?
    if (smb.state == SMB_STATE_IDLE)
    {
        // Reset SMBus variables.
        smb.txLength = 0;
        smb.txCount = 0;
        smb.rxCount = 0;
        smb.error = FALSE;
#ifdef SMB_SUPPORT_PEC
        smb.pec = 0;
#endif //SMB_SUPPORT_PEC

    }

    // Use the TWI status information to make desicions.
    switch (TWSR & 0xf8)
    {
        case 0x60:      // SLA + W received, ACK returned
        {
            // State should be IDLE when SLA+W is received. If SLA+W is received
            // and state is not IDLE, an error has probably occured in an earlier
            // transmission that could not be detected at the time. Nothing can be
            // done to rescue the last transmission, but the SMBus driver variables
            // should be reset so the ongoing transmission can complete correctly.
          if (smb.state != SMB_STATE_IDLE)
          {
              smb.txLength = 0;
              smb.txCount = 0;
              smb.rxCount = 0;
              smb.error = FALSE;
#ifdef SMB_SUPPORT_PEC
              smb.pec = 0;
#endif //SMB_SUPPORT_PEC
          }
          smb.state = SMB_STATE_WRITE_REQUESTED;

#ifdef SMB_SUPPORT_PEC
          SMBcrc(SMB_OWN_ADDRESS_W);
#endif //SMB_SUPPORT_PEC
          break;
        }

        case 0x80:      // Previously addressed with own SLA+W, data received, ACK returned
        {
            // Store data received in receive buffer and increase receive count.
            temp = TWDR;
            smb.rxBuffer[smb.rxCount] = temp;
            smb.rxCount++;

#ifdef SMB_SUPPORT_PEC
            // Calculate PEC.
            SMBcrc(temp);
#endif
            // If the receive buffer is full, disable ACKing of the next data byte.
            if (smb.rxCount == SMB_RX_BUFFER_LENGTH)
            {
                enableACK = FALSE;
            }
            break;
        }

        case 0x88:      // Previously addressed with own SLA+W, data received, NACK returned.
        {
            smb.error = TRUE;
            smb.state = SMB_STATE_IDLE;
            break;
        }

        case 0xa0:   // P or Sr received while still addressed.
        {
            ProcessMessage(&smb);
            break;
        }

        case 0xa8:  // Own SLA +R received, ACK returned.
        {
            // Calculate PEC of own address + R.
#ifdef SMB_SUPPORT_PEC
            SMBcrc(SMB_OWN_ADDRESS_R);
#endif
            if (smb.state == SMB_STATE_IDLE) // Receive byte.
            {
                ProcessReceiveByte(&smb);
                smb.state = SMB_STATE_READ_REQUESTED;
            }

            // Make the first byte of txBuffer ready for transmission.
            temp = smb.txBuffer[0];
            TWDR = temp;
            smb.txCount++;

#ifdef SMB_SUPPORT_PEC
            // Calculate PEC of sent byte.
            SMBcrc(temp);
#endif

            break;
        }

        case 0xb8:      // Data byte in TWDR transmitted, ACK received.
        {
            if (smb.txCount == smb.txLength)
            {
#ifdef SMB_SUPPORT_PEC
                TWDR = smb.pec;     // If PEC is enabled, an ACK is OK here.
#else
                smb.error = TRUE;   // If PEC is disabled, an ACK here is an error.
#endif //SMB_SUPPORT_PEC
                enableACK = FALSE;
            }
            else
            {
                temp = smb.txBuffer[smb.txCount];
                TWDR = temp;
                smb.txCount++;
#ifdef SMB_SUPPORT_PEC
                SMBcrc(temp);
#endif //SMB_SUPPORT_PEC

            }
            break;
        }

        case 0xc0:      // Data byte in TWDR transmitted, NACK received.
        {
            if (smb.txCount != smb.txLength)
            {
                // Error, NACK is only expected after last data byte or PEC.
                smb.error = TRUE;
            }
            smb.state = SMB_STATE_IDLE;
            break;
        }

        case 0xc8:      // Last data byte in TWDR transmitted, ACK received.
        {
            smb.error = TRUE;
            smb.state = SMB_STATE_IDLE;
            break;
        }

        case 0x00:      // Bus error due to an illegal START or STOP condition.
        {
            TWCR |= (1 << TWSTO);
            smb.error = TRUE;
            smb.state = SMB_STATE_IDLE;
            break;
        }

        default:        // Unexpected TWSR value, error.
        {

            smb.error = TRUE;
            smb.state = SMB_STATE_IDLE;
            break;
        }
    }

    // Issue next TWI command.
    if (enableACK)
    {
        // Set TWEA flag and don't clear TWINT (if set) at the same time.
        temp = TWCR;
        temp |= (1 << TWEA);
        temp &= ~(1 << TWINT);
        TWCR = temp;
    }
    else
    {
        // Clear TWEA flag and make sure that TWINT is not cleared at the same time.
        TWCR &= ~((1 << TWEA) | (1 << TWINT));
    }

    TWCR |= (1 << TWEN) | (1 << TWIE) | (1 << TWINT);
}


#ifdef SMB_USE_PEC_LOOKUP
//! Table of crc values stored in flash.
unsigned const char crcTable[256] PROGMEM =	{	
	0x00, 0x07, 0x0e, 0x09, 0x1c, 0x1b, 0x12, 0x15,
	0x38, 0x3f, 0x36, 0x31, 0x24, 0x23, 0x2a, 0x2d,
	0x70, 0x77, 0x7e, 0x79, 0x6c, 0x6b, 0x62, 0x65,
	0x48, 0x4f, 0x46, 0x41, 0x54, 0x53, 0x5a, 0x5d,
	0xe0, 0xe7, 0xee, 0xe9, 0xfc, 0xfb, 0xf2, 0xf5,
	0xd8, 0xdf, 0xd6, 0xd1, 0xc4, 0xc3, 0xca, 0xcd,
	0x90, 0x97, 0x9e, 0x99, 0x8c, 0x8b, 0x82, 0x85,
	0xa8, 0xaf, 0xa6, 0xa1, 0xb4, 0xb3, 0xba, 0xbd,
	0xc7, 0xc0, 0xc9, 0xce, 0xdb, 0xdc, 0xd5, 0xd2,
	0xff, 0xf8, 0xf1, 0xf6, 0xe3, 0xe4, 0xed, 0xea,
	0xb7, 0xb0, 0xb9, 0xbe, 0xab, 0xac, 0xa5, 0xa2,
	0x8f, 0x88, 0x81, 0x86, 0x93, 0x94, 0x9d, 0x9a,
	0x27, 0x20, 0x29, 0x2e, 0x3b, 0x3c, 0x35, 0x32,
	0x1f, 0x18, 0x11, 0x16, 0x03, 0x04, 0x0d, 0x0a,
	0x57, 0x50, 0x59, 0x5e, 0x4b, 0x4c, 0x45, 0x42,
	0x6f, 0x68, 0x61, 0x66, 0x73, 0x74, 0x7d, 0x7a,
	0x89, 0x8e, 0x87, 0x80, 0x95, 0x92, 0x9b, 0x9c,
	0xb1, 0xb6, 0xbf, 0xb8, 0xad, 0xaa, 0xa3, 0xa4,
	0xf9, 0xfe, 0xf7, 0xf0, 0xe5, 0xe2, 0xeb, 0xec,
	0xc1, 0xc6, 0xcf, 0xc8, 0xdd, 0xda, 0xd3, 0xd4,
	0x69, 0x6e, 0x67, 0x60, 0x75, 0x72, 0x7b, 0x7c,
	0x51, 0x56, 0x5f, 0x58, 0x4d, 0x4a, 0x43, 0x44,
	0x19, 0x1e, 0x17, 0x10, 0x05, 0x02, 0x0b, 0x0c,
	0x21, 0x26, 0x2f, 0x28, 0x3d, 0x3a, 0x33, 0x34,
	0x4e, 0x49, 0x40, 0x47, 0x52, 0x55, 0x5c, 0x5b,
	0x76, 0x71, 0x78, 0x7f, 0x6a, 0x6d, 0x64, 0x63,
	0x3e, 0x39, 0x30, 0x37, 0x22, 0x25, 0x2c, 0x2b,
	0x06, 0x01, 0x08, 0x0f, 0x1a, 0x1d, 0x14, 0x13,
	0xae, 0xa9, 0xa0, 0xa7, 0xb2, 0xb5, 0xbc, 0xbb,
	0x96, 0x91, 0x98, 0x9f, 0x8a, 0x8d, 0x84, 0x83,
	0xde, 0xd9, 0xd0, 0xd7, 0xc2, 0xc5, 0xcc, 0xcb,
	0xe6, 0xe1, 0xe8, 0xef, 0xfa, 0xfd, 0xf4, 0xf3
};


 /* \brief PEC CRC lookup function
  *
  * This function uses a table stored in flash to look up the
  * PEC value of one byte, using the PEC calculated so far as a
  * starting point.
  *
  * \param  data    Value to calculate PEC of.
  * \param  pec     Pointer to current PEC.
  */
#pragma inline
static void SMBcrc(unsigned char data)
{
    smb.pec ^= data;
    smb.pec = crcTable[smb.pec];
}
#endif //SMB_USE_PEC_LOOKUP


#ifdef SMB_USE_PEC_CALCULATION
 /* \brief PEC CRC lookup function
  *
  * This function calculates the PEC value of one byte, using the
  * PEC calculated so far as a starting point.
  *
  * \param  data    Value to calculate PEC of.
  * \param  pec     Pointer to current PEC.
  */
// PEC calculation routine
static void SMBcrc(unsigned char data)
{
    unsigned char i;	// Counter for 8 shifts

    smb.pec ^= data;        // Initial XOR

    i = 8;
    do
    {
        if (smb.pec & 0x80)
        {
            smb.pec <<= 1;
            smb.pec ^= SMB_CRC_POLYNOME;
        }
        else
        {
            smb.pec <<= 1;
        }
    }
    while(--i);
}
#endif //SMB_USE_PEC_CALCULATION
