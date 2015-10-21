// This file has been prepared for Doxygen automatic documentation generation.
/*! \file ********************************************************************
*
* Atmel Corporation
*
* - File              : SMBExample.c
* - Compiler          : IAR EWAAVR 4.10b
*
* - Support mail      : avr@atmel.com
*
* - Supported devices : All AVR devices with a TWI module can be used.
*                       The example is written for ATmega32
*
* - AppNote           : AVR316 - SMBus slave
*
* - Description       : Application specific part of SMBus slave
*                       implementation. Actions based o
*
* $Revision: 5627 $
* $Date: 2009-05-15 14:49:21 +0800 (Fri, 15 May 2009) $
*****************************************************************************/

#include <avr/io.h>
#include <avr/interrupt.h>
#include "SMBExample.h"
#include "SMBSlave.h"


unsigned char __eeprom vendor[] = "ACME SMBus Devices";     //!< Vendor ID string.
unsigned char __eeprom * e2p = (unsigned char __eeprom *)0; //!< EEPROM pointer, initialized to 0.

extern unsigned volatile char leds[32];      //!< Array of values to display on LEDs.
extern unsigned volatile char ledLength;     //!< Number of values to display on LEDs.
extern unsigned volatile char ledIndex;      //!< Index of the currently displayed LED value.


/*! \brief Prepares the response to a Receive Byte protocol..
 *
 *  This function gets called whenever a SLA+R has been received
 *  right after a Start condition. If the Receive byte protocol is
 *  supported, data should be put in smb->txBuffer[0] and
 *  smb->txLength should be set to 1.
 *
 *  \param smb Pointer to the SMBus driver's SMBData struct.
 *
 *  \note While this runs the SMBCLK is held low, and it is thus important to keep the functions
 *  short enough not to violate the SMBus T_LOW:SEXT of 25ms. If this happens, the master will time
 *  out and drop the lines, and since the AVR's TWI module is I2C compatible there is no timeout
 *  in the TWI module. The AVR can then hold the SMBDAT line low waiting for clock, and thus block
 *  the bus indefinitely.
 */
void ProcessReceiveByte(SMBData *smb)
{
    smb->txBuffer[0] = ~PIND;
    smb->txLength = 1;
}


/*! \brief Prepares response to all supported protocols except Receive Byte
 *
 *  This function gets called whenever a Write byte, Write word, Read byte,
 *  Read word, Process call, Block write, Block read or Block write, block
 *  read process call protocol is in progress. The function is called rigth
 *  after a Stop or Repeated start condition is received.
 *
 *  This function, along with ProcessReceiveByte implements the application
 *  specific parts of the SMBus slave. For this reason, it can get very large.
 *  To remedy this, it has been broken up into one inlined function per
 *  command code.
 *
 *  Since the TWI module does not diffrentiate between a Stop condition
 *  and a repeated start condition, it is important that smb->state is set
 *  in this function, to reflect the state of the SMBus driver.
 *
 *  When this function is called, the data receieved is available through
 *  smb->rxBuffer array. All data received (excluding SLA+W) is available.
 *  The total number of valid bytes in smb->rxBuffer is available in
 *  smb->rxLength. The command code is always available in smb->rxBuffer[0].
 *  The rest of the array is filled according to the SMBus specification.
 *
 *  For Write byte, Write word and Block write, data must simply be copied
 *  to their destination. smb->state should then be set to SMB_STATE_IDLE
 *  before returning.
 *
 *  For Read byte, Read word, Process call, Block read and Block write, block
 *  read process call, a response must be made ready and put in smb-txBuffer.
 *  The data in smb->txBuffer includes all data to be sent back by the SMBus
 *  driver (excluding PEC). The smb->txLength variable must be set to the
 *  total number of bytes to be transmitted (excluding PEC). When the response
 *  is complete, the smb-> state variable should be set to
 *  SMB_STATE_WRITE_READ_REQUESTED before returning.
 *
 *  In the following examples, it is explained how to prepare a response to
 *  these protocols.
 *
 *  Read byte:                                                  <br> <code>
 *  smb->txBuffer[0] = Data byte                                <br>
 *  smb->txLength = 1                                           <br></code>
 *
 *
 *  Read word / Process call:                                   <br> <code>
 *  smb->txBuffer[0] = Data byte low                            <br>
 *  smb->txBuffer[1] = Data byte high                           <br>
 *  smb->txLength = 2                                           <br></code>
 *
 *
 *  Block read / Block write, block read process call:          <br><code>
 *  smb->txBuffer[0] = Byte count (n)                           <br>
 *  smb->txBuffer[1] = Data byte 1                              <br>
 *                :                                             <br>
 *                :                                             <br>
 *  smb->txBuffer[n] = Data byte n                              <br>
 *  smb->txLength = n + 1                                       <br></code>
 *
 *
 *  \param smb Pointer to the SMBus driver's SMBData struct.
 *
 *  \note While this runs the SMBCLK is held low, and it is thus important to keep the functions
 *  short enough not to violate the SMBus T_LOW:SEXT of 25ms. If this happens, the master will time
 *  out and drop the lines, and since the AVR's TWI module is I2C compatible there is no timeout
 *  in the TWI module. The AVR can then hold the SMBDAT line low waiting for clock, and thus block
 *  the bus indefinitely.
 */
void ProcessMessage(SMBData *smb)
{
    if (smb->state == SMB_STATE_WRITE_REQUESTED)
    {
        switch (smb->rxBuffer[0]) // Command code.
        {
            case SMB_COMMAND_RETURN_VENDOR_STRING:  // Block read, vendor ID.
                ReturnVendorString(smb);
                break;

            case SMB_COMMAND_RETURN_SWITCHES:  // Read byte, buttons.
                ReturnSwitchesPressed(smb);
                break;

            case SMB_COMMAND_SET_EE_POINTER:  // Write word, EEPROM pointer.
                SetEepromPointer(smb);
                break;

            case SMB_COMMAND_READ_EE_BYTE:  // Read byte, EEPROM data.
                ReadEepromByte(smb);
                break;

            case SMB_COMMAND_READ_EE_WORD:  // Read word, EEPROM data.
                ReadEepromWord(smb);
                break;

            case SMB_COMMAND_WRITE_LED_BYTE:  // Write byte, LEDs.
                OutputLedByte(smb);
                break;

            case SMB_COMMAND_WRITE_LED_WORD:  // Write word, LEDs.
                OutputLedWord(smb);
                break;

            case SMB_COMMAND_WRITE_LED_BLOCK:  // Block write, LEDs.
                OutputLedBlock(smb);
                break;

            case SMB_COMMAND_MULTIPLY_BY_TWO: // Process call, Multiply by two.
                ReturnParameterTimesTwo(smb);
                break;

            case SMB_COMMAND_SUM_OF_BYTES:  // Block write, block read process call. Sum of bytes.
                ReturnSumOfBytes(smb);
                break;
            default:
                UndefinedCommand(smb);
                break;
        }
    }
    else
    {
        smb->state = SMB_STATE_IDLE;
    }
}


/*! \brief Returns a vendor string in response to a Block read.
 *
 *  This function demonstrates how to set up a response to a Block read
 *  request. A vendor ID string will be transmitted when this function is invoked.
 *
 *  \param smb Pointer to the SMBus driver's SMBData struct.
 */
#pragma inline=forced
static void ReturnVendorString(SMBData *smb)
{
    unsigned char i;
    unsigned char temp;

    i = 0;
    // Copy vendor ID string from EEPROM.
    while ((temp = vendor[i]) != '\0')
    {
        i++;
        smb->txBuffer[i] = temp;
    }
    smb->txBuffer[0] = i; // Byte count.
    smb->txLength = i + 1; // Number of bytes to be transmitted including byte count.

    smb->state = SMB_STATE_WRITE_READ_REQUESTED;
}


/*! \brief Returns switches pressed on STK500 as a number.
 *
 *  This function demonstrates how to set up a response to a Read byte
 *  request. The switches pressed on the STK500 will be transmitted as
 *  a byte-sized number.
 *
 *  \param smb Pointer to the SMBus driver's SMBData struct.
 */
#pragma inline=forced
static void ReturnSwitchesPressed(SMBData *smb)
{
    smb->txBuffer[0] = ~PIND;
    smb->txLength = 1;

    smb->state = SMB_STATE_WRITE_READ_REQUESTED;
}


/*! \brief Sets the EEPROM pointer.
 *
 *  This function demonstrates how to use data received in a Write word
 *  protocol. The data received is converted to an EEPROM pointer and
 *  used to fetch EEPROM data for other commands.
 *
 *  \param smb Pointer to the SMBus driver's SMBData struct.
 */
#pragma inline=forced
static void SetEepromPointer(SMBData *smb)
{
// If the largest EEPROM address is 0xff or less, a 1 byte EEPROM pointer is sufficient.
// Otherwise, a two byte pointer is used.
#if E2END <= 0xff
    unsigned char tempData;
#else
    unsigned int tempData;
#endif

    if ( RX_COUNT_AND_PEC_CHECK(3) )
    {
#if E2END <= 0xff
        // Cast received low byte to EEPROM pointer.
        tempData = smb->rxBuffer[1] & E2END;
        e2p = (unsigned char __eeprom *)(tempData);
#else
        // Merge received bytes into one word and cast to EEPROM pointer.
        tempData = ((smb->rxBuffer[2] << 8) | smb->rxBuffer[1]);
        tempData &= E2END;
        e2p = (unsigned char __eeprom *)(tempData);
#endif
    }
    else
    {
        smb->error = TRUE;
    }

    smb->state = SMB_STATE_IDLE;
}


/*! \brief Returns one byte from EEPROM at the current location of the EEPROM pointer.
 *
 *  This function demonstrates how to set up a response to a Read byte
 *  request. The data byte pointed to by the EEPROM pointer will be transmitted
 *  when this function is invoked.
 *
 *  \param smb Pointer to the SMBus driver's SMBData struct.
 */
#pragma inline=forced
static void ReadEepromByte(SMBData *smb)
{
    smb->txBuffer[0] = *e2p;
    smb->txLength = 1;

    smb->state = SMB_STATE_WRITE_READ_REQUESTED;
}


/*! \brief Returns two bytes from EEPROM at the current location of the EEPROM pointer.
 *
 *  This function demonstrates how to set up a response to a Read word
 *  request. The data word pointed to by the EEPROM pointer will be transmitted
 *  when this function is invoked.
 *
 *  \param smb Pointer to the SMBus driver's SMBData struct.
 */
#pragma inline=forced
static void ReadEepromWord(SMBData *smb)
{
    smb->txBuffer[0] = *e2p;
    smb->txBuffer[1] = *(e2p + 1);
    smb->txLength = 2;

    smb->state = SMB_STATE_WRITE_READ_REQUESTED;
}


/*! \brief Output 1 byte to LEDS on STK500.
 *
 *  This function demonstrates how to use data received in a Write byte
 *  protocol. The received byte is output to the LEDS of the STK500 (PORTB).
 *
 *  \param smb Pointer to the SMBus driver's SMBData struct.
 */
#pragma inline=forced
static void OutputLedByte(SMBData *smb)
{
    if (RX_COUNT_AND_PEC_CHECK(2))
    {
        leds[0] = smb->rxBuffer[1];
        ledLength = 1;
        ledIndex = 0;
    }
    else
    {
        smb->error = TRUE;
    }

    smb->state = SMB_STATE_IDLE;
}


/*! \brief Output 2 bytes to LEDS on STK500.
 *
 *  This function demonstrates how to use data received in a Write word
 *  protocol. The received data is output as  a sequence to the LEDS of the
 *  STK500 (PORTB).
 *
 *  \param smb Pointer to the SMBus driver's SMBData struct.
 */
#pragma inline=forced
static void OutputLedWord(SMBData *smb)
{
    if (RX_COUNT_AND_PEC_CHECK(3))
    {
        leds[0] = smb->rxBuffer[1];
        leds[1] = smb->rxBuffer[2];
        ledLength = 2;
        ledIndex = 0;
    }
    else
    {
        smb->error = TRUE;
    }

    smb->state = SMB_STATE_IDLE;
}


/*! \brief Output data block to LEDS on STK500.
 *
 *  This function demonstrates how to use data received in a Block write
 *  protocol. The received data is output as  a sequence to the LEDS of the
 *  STK500 (PORTB).
 *
 *  \param smb Pointer to the SMBus driver's SMBData struct.
 */
#pragma inline=forced
static void OutputLedBlock(SMBData *smb)
{
    unsigned char i;
    unsigned char byteCount;
    byteCount = smb->rxBuffer[1];
    if (RX_COUNT_AND_PEC_CHECK(byteCount+2))
    {
        for (i = 0; i  < byteCount; i++)
        {
            leds[i] = smb->rxBuffer[i + 2];
        }
        ledLength = byteCount;
        ledIndex = 0;
    }
    else
    {
        smb->error = TRUE;
    }

    smb->state = SMB_STATE_IDLE;
}


/*! \brief Treat the incoming parameter as a word and return the parameter * 2.
 *
 *  This function demonstrates how to use the Process call
 *  protocol. The incoming data is treated as one word-sized parameter.
 *  This incoming parameter is multiplied by 2 and transmitted as a
 *  word-sized value.
 *
 *  \param smb Pointer to the SMBus driver's SMBData struct.
 */
#pragma inline=forced
static void ReturnParameterTimesTwo(SMBData *smb)
{
    unsigned int temp;
    if (smb->rxCount == 3)
    {
        // Multiply incoming data by 2
        temp = smb->rxBuffer[1] | (smb->rxBuffer[2] << 8);
        temp *= 2;
        // Send result back
        smb->txBuffer[0] = (unsigned char)temp;        // Low byte
        smb->txBuffer[1] = (unsigned char)(temp >> 8); // High byte
        smb->txLength = 2;

        smb->state = SMB_STATE_WRITE_READ_REQUESTED;
    }
    else
    {
        smb->error = TRUE;

        smb->state = SMB_STATE_IDLE;
    }
}


/*! \brief Sums each byte received and returns the sum as a word.
 *
 *  This function demonstrates how to use the Block write, block read
 *  process call protocol. The received data is treated as a stream
 *  of byte-sized values. The sum of these values are calculated and
 *  transmitted back as a word-sized (2 bytes) value.
 *
 *  \param smb Pointer to the SMBus driver's SMBData struct.
 */
#pragma inline=forced
static void ReturnSumOfBytes(SMBData *smb)
{
    unsigned int sum;
    unsigned char i;
    unsigned char byteCount;

    byteCount = smb->rxBuffer[1];
    if (smb->rxCount == (byteCount + 2))
    {
        sum = 0;

        // Calculate sum of received bytes
        for (i = 2; i < byteCount + 2; i++)
        {
            sum += smb->rxBuffer[i];
        }

        // Send result back
        smb->txBuffer[0] = 2;                          // Number of bytes
        smb->txBuffer[1] = (unsigned char)sum;        // Low byte
        smb->txBuffer[2] = (unsigned char)(sum >> 8); // High byte
        smb->txLength = 3;

        smb->state = SMB_STATE_WRITE_READ_REQUESTED;
    }
    else
    {
        smb->error = TRUE;
        smb->state = SMB_STATE_IDLE;
    }
}


/*! \brief Handles undefined commands.
 *
 *  This function will be called whenever an unrecognized command code
 *  is received.
 *
 *  \param smb Pointer to the SMBus driver's SMBData struct.
 */
#pragma inline=forced
static void UndefinedCommand(SMBData *smb)
{
    // Handle undefined requests here.
    smb->error = TRUE;
    smb->state = SMB_STATE_IDLE;
}
