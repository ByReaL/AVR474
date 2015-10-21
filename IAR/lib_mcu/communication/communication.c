/*This file has been prepared for Doxygen automatic documentation generation.*/
/*! \file *********************************************************************
 *
 * \brief 
 *     The communication file reads a command from the SMB bus and sends
 *     r/w, data and length information to SBS layer.
 *
 * \par Application note:
 *      AVR456: SB201 HVA one two cell smart battery firmware 
 *
 * \par Documentation
 *      For comprehensive code documentation, supported compilers, compiler
 *      settings and supported devices see readme.html
 *
 * \author
 *      Atmel Corporation: http://www.atmel.com \n
 *      Support email: avr@atmel.com
 *
 * $Name:  $
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

#include "common.h"

#define MODULE_SMBUS		/* ensure that we instantiate our variables in smbus.h */

#include "smbus.h"
#include "communication.h"
#include "SBS_commands.h"
#include "interpreter.h"
#include "crc8.h"



//---------------------------------------------------------------------
unsigned char TEST50US = 0;
unsigned char SMLOCK   = 0;	      //prevents Master bus grab attempts WHILE BUS IS IN MASTER MODE.

unsigned char TW_MTxBuf[8];	      //Master-mode TX buffer
unsigned char TW_MTxBufCnt   = 0; //how many valid bytes are in the buffer
unsigned char TW_MTxBufIndex = 0;

// Note for the buffers below, these must be able to contain:
// Slave Address, SMBus Command, Byte Count (if Block-mode), up to 32 bytes, plus PEC.
unsigned char TW_TxBuf[36];	      //must be long enough for any outbound strings
unsigned char TW_TxBufCnt    = 0; //how many valid bytes are in the buffer
unsigned char TW_TxBufIndex  = 0;

unsigned char TW_RxBuf[10];	      //In Application mode (non-ISP mode), only receive WORD commands.
signed char TW_RxBufCnt      = 0;
unsigned char TW_RxBufIndex  = 0;


//This byte contains flags from the TWI ISR to tell the Foreground code what to do.
//If this byte is ever non-zero, the foreground code will act on its contents.
//Although it is written by both the ISR and the Foreground code, it does not
//  need to be declared VOLATILE because the SMBus is halted until the foreground
//  code finishes processing the associated command and has cleared this flag byte.
unsigned char TWI_CmdFlags;
  #define SMB_GenBusTimeout 1	/* Tell Foreground to generate a bus timeout, as we saw an error! */
  #define SMB_SetUpReply    2	/* Have Foreground set up TW_TxBuf[]. */
  #define SMB_GotCmdData    4	/* Have Foreground interpret the complete received command. */

unsigned char CurrentCmd = 0xFF;
unsigned char UsePEC = 0;	//PEC usage is disabled by default.



/* *************************************************************************
 *
 *   SMBus Initialization routine
 *
 ************************************************************************* */
/*! \brief Initialize TWI module
 *
 *  This function initializes the TWI module for SMBus operation.
 */

void Comm_Init(void)
{
	SMB_RestoreBus();
	TWBCSR = (1<<TWBCIF) | (1<<TWBCIE) | (1<<TWBDT1) | (1<<TWBDT0) | (0<<TWBCIP);
}




/* *************************************************************************
 *
 *   SMBus Wakeup Interrupt
 *
 ************************************************************************* */


//This wakes up a battery from sleep mode into the "On" state, per sbdat110, 4.4.2

#pragma vector = TWI_BUS_CD_vect
__interrupt void TWICD_ISR(void)
{
	//clear bits per sbdat110, 4.4.2
	SMBvariables[SMBV_BattMode][hibyte] &= ~(0xE3);	
	
	if(TWBCSR & (1<<TWBCIP))	//this int was caused by bus DISCONNECT event.
	{
		TWBCSR &= ~(1<<TWBCIP);
//SH		ChangePowerMode(POWERMODE_IDLE,0);
	}
	else	//this int occurred due to a bus CONNECT event.
	{
		TWBCSR |= (1<<TWBCIP);
//SH		ChangePowerMode(POWERMODE_ACTIVE,0);
	}
}






/*! \brief  Reply to a SBS read block command.
 *
 *
 *	\param sbsReply Is a pointer to the #SBS_command_t struct containing the data to transmit.
 *
 */	
void Comm_SbsBlockCommandReply( SBS_command_t* sbsReply )
{
/*	// Send the Data Payload and update PEC
	for( uint8_t i=0; i < SBSDATA_BLOCK_LENGTH; i++)
	{
		SW_UART_Transmit( sbsReply->payload[i] );
		CRC8_Update(&COMM_CalculatedPEC, sbsReply->payload[i] );
	}
	// Send the PEC
	SW_UART_Transmit( COMM_CalculatedPEC );
	*/
}

/*! \brief  Reply to a SBS read word command.
 *
 *	\param sbsReply Is a pointer to the #SBS_command_t struct containing the data to transmit.
 */	
void Comm_SbsWordCommandReply( SBS_command_t* sbsReply )
{
/*	// Send the Data Payload
	SW_UART_Transmit( sbsReply->payload[0] );
	SW_UART_Transmit( sbsReply->payload[1] );
	// Calculate the PEC and send it
	CRC8_Update( &COMM_CalculatedPEC, sbsReply->payload[0] );
	CRC8_Update( &COMM_CalculatedPEC, sbsReply->payload[1] );
	SW_UART_Transmit( COMM_CalculatedPEC );
	*/
}

/*! \brief  Parse command.
 *
 *  Check the command and send message to SBS 
 *  for the protocol layer 
 * 
 *  It is important that if this function return true (receive is complete) that
 *  it is handles before this function is called again
 *
 *  \param  sbsCmdDestination  Where to save the incoming data
 *
 *  \return True if a new command has been received and transferred to the destination buffer.
*/	
bool Comm_Handle( SBS_command_t* sbsCmdDestination )
{
	bool receiveComplete = false;
	/*
	// Clear the flag as early as possible to allow it to be set again and this
	// function called again
	CLEAR_FLAG(SW_UART_status, SW_UART_RX_COMPLETE );
	
	if( READ_FLAG(SW_UART_status, SW_UART_FRAME_ERROR) ) {
		// If a frame error has been received, treat it as a break and wait for 
		// next command
		CLEAR_FLAG(SW_UART_status, SW_UART_FRAME_ERROR);
		CLEAR_FLAG(SW_UART_status, SW_UART_RX_BUFFER_OVERFLOW);
		COMM_State = COMM_WAITING_FOR_COMMAND;
		
	} else if( READ_FLAG(SW_UART_status, SW_UART_RX_BUFFER_OVERFLOW) ) {
		// If a buffer overflow occured, wait for a break
		CLEAR_FLAG(SW_UART_status, SW_UART_RX_BUFFER_OVERFLOW);
		COMM_State = COMM_WAITING_FOR_BREAK;
	} else {
		
		// Handle received data if any
		while( SW_UART_Data_Available() ) {
			if(receiveComplete) {
				// If receive is complete, but there is still data in the RX buffer, 
				// something went wrong so reset everything and wait for a break
				CLEAR_FLAG(SW_UART_status, SW_UART_FRAME_ERROR);
				CLEAR_FLAG(SW_UART_status, SW_UART_RX_BUFFER_OVERFLOW);
				receiveComplete = false;
				SW_UART_Flush_Buffers();
				COMM_State = COMM_WAITING_FOR_BREAK;
				
			} else if( READ_FLAG(SW_UART_status, SW_UART_FRAME_ERROR) ) {
				// If a frame error has been received, treat it as a break and wait for 
				// next command
				CLEAR_FLAG(SW_UART_status, SW_UART_FRAME_ERROR);
				CLEAR_FLAG(SW_UART_status, SW_UART_RX_BUFFER_OVERFLOW);
				COMM_State = COMM_WAITING_FOR_COMMAND;
				
			} else if( READ_FLAG(SW_UART_status, SW_UART_RX_BUFFER_OVERFLOW) ) {
				// If a buffer overflow occured, wait for a break
				CLEAR_FLAG(SW_UART_status, SW_UART_RX_BUFFER_OVERFLOW);
				COMM_State = COMM_WAITING_FOR_BREAK;
				
			} else {
				// Get the received data
				uint8_t data = SW_UART_Getchar();
				
				switch( COMM_State ) {
				case COMM_WAITING_FOR_BREAK:
					{
						// Do nothing. Break is handled outside the case switch
						break;
					}
				case COMM_WAITING_FOR_COMMAND:
					{
						// Allow to transmit data
						CLEAR_FLAG(SW_UART_status, SW_UART_DECLINE_NEW_TX_DATA);
						// Save command
						sbsCmdDestination->command = (SBS_commands_t)data;
						if( data & 0x80 ) {
							// Read command
							receiveComplete = true;
							COMM_State = COMM_WAITING_FOR_COMMAND;
						} else {
							// Write command
							COMM_State = COMM_RECEIVING_DATA;
							COMM_ReceivedBytes = 0;
							
							// Since only write commands are handled here, no need to remove
							// the read/write bit when checking for block
							if(   SBS_IsCommandStaticStringType((SBS_commands_t)(data))
								 || SBS_IsCommandBlockDataType((SBS_commands_t)(data)) )
							{
								// Block command
								COMM_BytesToReceive = SBSDATA_BLOCK_LENGTH ;
							} else {
								// Word command
								COMM_BytesToReceive = 2;
							}
						}
						//Send command back to the host and initialize the PEC
						SW_UART_Transmit( data );
						COMM_CalculatedPEC = 0;
						CRC8_Update( &COMM_CalculatedPEC, data);
						break;
					}
					
				case COMM_RECEIVING_DATA:
					{
						if(COMM_ReceivedBytes == COMM_BytesToReceive) {
							// All data bytes have been received, this is the PEC
							if( data == COMM_CalculatedPEC ) {
								receiveComplete = true;
								// The function that handles receiveComplete has to send ACK
								//  for write commands
							} else {
								// If the PEC was wrong, send NACK
								SW_UART_Transmit(COMM_NACK);
								COMM_State = COMM_WAITING_FOR_COMMAND;
							}
						} else {
							// Add received byte to receive buffer and update PEC
							sbsCmdDestination->payload[COMM_ReceivedBytes++] = data;
							CRC8_Update(&COMM_CalculatedPEC, data);
						}
						break;
					}
				} // switch(COMM_state)
			}
		} // while( SW_UART_Data_Available() )
	}
	*/
	return receiveComplete;
}


/*! \brief  Check Communication State
 *
 *  This function returns true, if it is no ongoing comm.
 *  Then it is safe to enter psave
 *  
 *  \return true if idle, false if busy
 */
bool Comm_IsIdle(void)
{
	return 1;
}


/*! \brief  Check Communication Status
 *
 *  This function returns 0, if it is nothing to handle.
 *  If it is > 0, then execute Comm_Handle()
 *  
 *  \return 0 if there is nothing to handle, 1 is there is
 */
uint8_t Comm_Flag(void)
{
	return 0;
}


unsigned char TXmsg[4][4];    //leave room for PEC
volatile unsigned char TXmsgHead = 0;
volatile unsigned char TXmsgTail = 0;
volatile unsigned char TXmsgQty = 0;

#define TXmsgEmpty (TXmsgQty == 0)
#define TXmsgFull (TXmsgQty == 4)
#define TXmsgDelete {++TXmsgTail; TXmsgTail &= (4-1);  TXmsgQty--;}

//We will compute the PEC for the message as well.
void MasterInsertMsg(unsigned char addr, unsigned char cmd, unsigned int data)
{ //Note that the Charger address is ALWAYS 0x12! (sbsm100.pdf, 5.2)
	//The Host address is always 0x16.
	//The addr always assumes WRITE.
	//We only do a WORD WRITE.
	
	unsigned char * ptr = TXmsg[TXmsgHead];
	
	*ptr++ = addr;
	*ptr++ = cmd;
	*ptr++ = (unsigned char) data;
	*ptr = (unsigned char)(data>>8);
	
	if(TXmsgFull)
		return;
	
	
	++TXmsgHead;
	TXmsgHead &= (4-1);
	TXmsgQty++;
}





//TWI Interrupt Service Routine
//  This ISR contains a state machine for handling the low-level bus communications.
//  It uses the variable TWI_CmdFlags to communicate the need for message processing
//    to the foreground-based command interpreter.  Whenever it passes control off
//    to the foreground routine, the SMBus is halted.

/* Changes to the operation of the SMBus State Machine as of 6/17/2005 (rgf):

   Since there can be multiple Masters on the bus addressing a Smart Battery,
   namely either a Host or a Charger, it is entirely possible that one Master
   may support PEC while the other does not.  For maximum reliability, we must
   be able to support the reception of PEC on-the-fly regardless of whether it
   was used in the past or not.

   To do so, we determine an expected byte count WITHOUT PEC.  While receiving,
   we decrement this counter and store received bytes until either (a) the counter
   drops below a value of (-1), or (b) we receive a STOP.  If we receive a STOP
   and the counter is not either 0 or -1, this is an error and is flagged as such.
   (Note that AFTER receiving a STOP, it is not possible to flag an error to the
   Host or Charger except through the Battery Status flags.)  If the counter is
   zero, this indicates that PEC is disabled.  If the counter is -1, this indicates
   that PEC is enabled.  The message will be processed with this knowledge.

   However, since the SCL line is not being held low due to TWINT being left asserted,
   a different mechanism is needed to hold off incoming messages to the battery
   until the previous message has been processed.  This is done by turning off
   the generation of ACK on any subsequent bytes and/or messages until the foreground
   code has finished processing the prior message.

   It is crucial therefore that the foreground code does not change the
   value of the TWI_CmdFlags variable from being equal to 'SMB_GotCmdData'
   until after it is completely done with processing of the prior message.

   This mechanism also is valid if a Master attempts to perform a Master Read
   while the battery is busy.  In this case, the second byte from the Master,
   specifically the SMBus Command byte, will not be acknowledged.  The Master
   is thereby required to generate a STOP condition.

*/

//State Machine states
enum /*TWISR_State*/ {TW_IDLE=0, TW_Wait4Stop, TW_Wait4Cmd, TW_Wait4RW, TW_Wait4Data, TW_ReplyData, TW_MSLA_W, TW_MCMD_W, TW_MDATA_W };

unsigned char TWISR_state = TW_IDLE;	//state variable


#pragma vector = TWI_vect
__interrupt void TWI_ISR(void)
{
	static unsigned char TWISR_CmdFeatures = 0;	//Command-related feature flags
	unsigned char Status;
	unsigned char tmp;
	
	Status = TWSR & 0xF8;		//This identifies what caused the interrupt to fire.
	
	switch(TWISR_state)
	{
    default:
    case TW_IDLE:	//If not SLA_W or RSTOP, is an error!
		if(TWS_SLA_W == Status)	// saw Slave address match with a Write bit
		{
			if(TWI_CmdFlags == SMB_GotCmdData)
			{
				//Assert that we're 'busy' to SMBus Master by leaving TWEA *off* until we get a STOP.
				//Note that this is 'legal' because we have ALREADY sent an ACK to our Slave Address,
				// as is required by the SMBus specification.
				TWISR_state = TW_Wait4Stop;
				TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWIE);	//must NOT re-enable ACKing!
				return;
			}
			else
				TWISR_state = TW_Wait4Cmd;
		}
		else if(TWS_RSTOP == Status)	//Saw a Stop, possibly left over from previous cmd.
		{
			;			//Everything is probably OK.  Take no action.
		}
		else if(TWS_START == Status)	//we have successfully sent a START bit. Handle MASTER mode.
		{
			TWDR = TW_MTxBuf[TW_MTxBufIndex++];
			TWISR_state = TW_MSLA_W;
		}
		else //had some type of error!
		{
			SMBvariables[SMBV_BattStatus][lobyte] |= SMBerr_UnknownError;
			TWI_CmdFlags = SMB_GenBusTimeout;	//generate a bus timeout.
			TWCR = (1<<TWEA) | (1<<TWEN);		//disable int, and DON'T clear the TWINT flag!
			TWISR_state = TW_IDLE;			//Reset the state machine.
			return;
		}
		TWCR = (1<<TWINT) | (1<<TWEA) | (1<<TWEN) | (1<<TWIE);	//must re-enable ACKing
		break;
		
    case TW_Wait4Stop:
		if(TWS_RSTOP == Status)	//Saw a Stop, possibly left over from previous cmd.
		{
			TWCR = (1<<TWINT) | (1<<TWEA) | (1<<TWEN) | (1<<TWIE);	//must re-enable ACKing
			TWISR_state = TW_IDLE;			//Reset the state machine.
		}
		else
			TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWIE);	//must NOT re-enable ACKing yet!
		break;
		
		
		//SLAVE-mode states follow.
		
    case TW_Wait4Cmd:	//upon entry, we expect to have received a Cmd byte.
		if(TWS_RCMD == Status)		//It appears that we have received a Command byte now.
		{
			tmp = TWDR;
			if(tmp <= HIGHEST_SMB_CMD)	//Is the Cmd within valid range?
			{
				CurrentCmd = tmp;		//Save a copy.
				tmp = SM_Cmd_Table[tmp][0];	//Grab the Command Characteristics/Features flags.
				if(tmp & SMBslave)		//Is the Command valid for Slaves?
				{				//The command appears to be valid.
					TWISR_CmdFeatures = tmp;	//Save the Feature flags for use in Wait4RW state.
					TWISR_state = TW_Wait4RW;	//set up next state
					TWCR = (1<<TWINT) | (1<<TWEA) | (1<<TWEN) | (1<<TWIE);	//enable ACKing
					return;
				}
			}
		}
		//In all cases except those that 'return' (above), it's an error.
		SMBvariables[SMBV_BattStatus][lobyte] |= SMBerr_UnknownError;
		TWI_CmdFlags = SMB_GenBusTimeout;	//generate a bus timeout.
		TWCR = (1<<TWEA) | (1<<TWEN);     //disable int, and DON'T clear the TWINT flag!
		TWISR_state = TW_IDLE;		//Reset the state machine.
		return;
		//    break;
		
		
    case TW_Wait4RW:	//We will now find out if we will RX more, or we need to TX a reply instead.
		if(TWS_RDATA == Status)		//It is a WRITE-type command. Prep the RX buffer to accept more data.
		{	//NOTE: except for OptionalMfgFunction5, all WRITE cmds are 2-byte, plus optional PEC.
			//Place all bytes of the transaction into the buffer so we can do a PEC on it if needed.
			TW_RxBuf[0] = TWAR & 0xFE;	//store everything incl. the slave address for computing PEC.
			TW_RxBuf[1] = CurrentCmd;	//store the previously-send Command.
			TW_RxBuf[2] = TWDR;		//store this first DATA byte
			TW_RxBufIndex = 3;		//use RxBufIndex as the index to store data in the buffer.
			if(TWISR_CmdFeatures & SCWW)	//is it a Write-WORD command type?
			{
				TW_RxBufCnt = 1;		//We expect 1 more data byte, and possibly PEC after that.
			}
			else if(TWISR_CmdFeatures & SCWG)	//is it a write-BLOCK command (must be OptionalMfgFunction5 then)
			{
				tmp = TWDR;
				if((tmp >= 1) && (tmp <= 32))
					TW_RxBufCnt = TWDR;
				else
				{
					SMBvariables[SMBV_BattStatus][lobyte] |= SMBerr_BadSize;
					TWI_CmdFlags = SMB_GenBusTimeout;	//generate a bus timeout.
					TWCR = (1<<TWEA) | (1<<TWEN);       //disable int, and DON'T clear the TWINT flag!
					TWISR_state = TW_IDLE;		//Reset the state machine.
					return;
				}
			}
			else	//this Command doesn't allow EITHER word OR group/block Writes! It's Read-only!
			{
				SMBvariables[SMBV_BattStatus][lobyte] |= SMBerr_AccessDenied;
				TWI_CmdFlags = SMB_GenBusTimeout;	//Not a WRITE-type cmd, so generate a bus timeout.
				TWCR = (1<<TWEA) | (1<<TWEN);         //disable int, and DON'T clear the TWINT flag!
				TWISR_state = TW_IDLE;		//Reset the state machine.
				return;
			}
			TWISR_state = TW_Wait4Data;
			TWCR = (1<<TWINT) | (1<<TWEA) | (1<<TWEN) | (1<<TWIE);	//enable ACKing
		}
		else if(TWS_REPEAT == Status)	//We saw a re-Start, so must be getting ready for a Read cmd.
		{	//Must now interpret previously-sent CurrentCmd & set up Reply data.
			if(TWISR_CmdFeatures & (SCRW | SCRG))	//Is it a 'ReadWord' or 'ReadGroup' command type?
			{
				TWI_CmdFlags = SMB_SetUpReply;	//Foreground decoder will set up TWCR.
				TWISR_state = TW_ReplyData;		//Move to next state.
			}
			else
			{
				SMBvariables[SMBV_BattStatus][lobyte] |= SMBerr_UnknownError;
				TWI_CmdFlags = SMB_GenBusTimeout;	//Not a READ-type cmd, so generate a bus timeout.
				TWCR = (1<<TWEA) | (1<<TWEN);		//disable int, and DON'T clear the TWINT flag!
				TWISR_state = TW_IDLE;		//Reset the state machine.
				return;
			}
			TWCR = (1<<TWEA) | (1<<TWEN);         //disable int, and DON'T clear the TWINT flag!
			return;					
		}
		else  //some type of error!
		{
			SMBvariables[SMBV_BattStatus][lobyte] |= SMBerr_UnknownError;
			TWI_CmdFlags = SMB_GenBusTimeout;	//Generate a bus timeout.
			TWCR = (1<<TWEA) | (1<<TWEN);		//disable int, and DON'T clear the TWINT flag!
			TWISR_state = TW_IDLE;			//Reset the state machine.
			return;
		}
		break;
		
		
    case TW_Wait4Data:	//We are in Slave Receive operating mode.
		if(TWS_RDATA == Status)			//received a data byte
		{
			tmp = TWDR;
			if(--TW_RxBufCnt < -1)			//Are we past the PEC byte and still getting more data?
			{
				SMBvariables[SMBV_BattStatus][lobyte] |= SMBerr_BadSize;	//throw away the data & flag error
				TWI_CmdFlags = SMB_GenBusTimeout;	//Generate a bus timeout.
				TWCR = (1<<TWEA) | (1<<TWEN);		//disable int, and DON'T clear the TWINT flag!
				TWISR_state = TW_IDLE;			//Reset the state machine.
				return;
			}
			TW_RxBuf[TW_RxBufIndex++] = TWDR;	//store the byte
		}
		
		else if(TWS_RSTOP == Status)			//got a STOP; all done RXing data now.
		{ //Note: if we get a STOP prematurely, then we simply ignore the command,
			//  since it is too late to inform the Master of the error.
			if((TW_RxBufCnt > 0) || (TW_RxBufCnt < -1))	//We got a premature STOP or too much data; ERROR!
			{
				SMBvariables[SMBV_BattStatus][lobyte] |= SMBerr_BadSize;	//throw away the data.
			}
			else
			{
				if(0 == TW_RxBufCnt)
					UsePEC = 0;				//there is no PEC coming for this packet.
				else if(-1 == TW_RxBufCnt)
					UsePEC = 1;				//PEC was included.
				
				TW_RxBufCnt = TW_RxBufIndex;		//re-use the Write Index's value as the Valid Byte Count.
				TW_RxBufIndex = 0;			//clear the index in preparation for interpreting it.
				TWI_CmdFlags = SMB_GotCmdData;	//tell Foreground to process this now.
				//Note that when (TWI_CmdFlags == SMB_GotCmdData), TWI ISR will respond with BUSY condition.
			}
			TWISR_state = TW_IDLE;		//Reset the state machine in all cases.
		}
		
		else  //some type of error during transmission!
		{
			SMBvariables[SMBV_BattStatus][lobyte] |= SMBerr_UnknownError;
			TWI_CmdFlags = SMB_GenBusTimeout;	//Not a WRITE-type cmd, so generate a bus timeout.
			TWCR = (1<<TWEA) | (1<<TWEN);		//disable int, and DON'T clear the TWINT flag!
			TWISR_state = TW_IDLE;			//Reset the state machine.
			return;
		}
		
		TWCR = (1<<TWINT) | (1<<TWEA) | (1<<TWEN) | (1<<TWIE);	//enable ACKing
		break;
		
		
    case TW_ReplyData:	//We are now in Slave Transmit operating mode.
		//The foreground code has set up the response that we are now sending.
		//Note: TW_TxBufCnt *always* includes the PEC byte! Since we don't
		// know whether the Master actually WANTS the PEC byte or not, we will
		// always TRY to send it, regardless of the state of the UsePEC flag.
		// If the Master does NOT want it, we will get a NAK while the PEC
		// byte is still in the buffer.  In the rare case where we send it all,
		// including the PEC byte, but we still get an ACK back, the TWI module
		// will be off-line anyway due to not setting the TWEA bit after sending PEC,
		// and we will therefore be unable to flag that an error has occurred.
		if((TWS_SLA_R == Status) || (TWS_RACK == Status))	//send out Reply data
		{
			TWDR = TW_TxBuf[TW_TxBufIndex++];	//send data out
			if(--TW_TxBufCnt == 0)			//Have we emptied the buffer, incl. PEC?
				TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWIE);		// Yes, so don't set TWEA.
			else
				TWCR = (1<<TWINT) | (1<<TWEA) | (1<<TWEN) | (1<<TWIE);	// No, so assert TWEA.
		}
		else if(TWS_RNAK == Status)	//We may have gotten this validly or as an Error.
		{
			if(TW_TxBufCnt == 1)	//Not an error. Master didn't want PEC; clear UsePEC flag!
			{
				TW_TxBufCnt = 0;	//clear the buffer too.
				UsePEC = 0;
			}
			else if(TW_TxBufCnt == 0)	//Not an error. Master wanted PEC (and got it); assert UsePEC.
				UsePEC = 1;
			else			//some kind of error occurred; we got NAK too early!
			{ //Note: the TWI module is now OFF-LINE, so we can't inform Host of this error!!
				SMBvariables[SMBV_BattStatus][lobyte] |= SMBerr_UnknownError;	//flag it later
			}
			TWISR_state = TW_IDLE;			//In all cases, go back to IDLE.
			TWCR = (1<<TWINT) | (1<<TWEA) | (1<<TWEN) | (1<<TWIE);
		}
		else
			//    if(TWS_FINAL == Status)	//ERROR: we got an ACK but we have no more data!
		{ //Since the TWI module is now in "Not Addressed Slave" mode, we can't flag the error
			// back to the Master DURING this transaction; we WILL assert an error status though.
			SMBvariables[SMBV_BattStatus][lobyte] |= SMBerr_BadSize;
			TWISR_state = TW_IDLE;	//Reset the state machine.
			TWCR = (1<<TWINT) | (1<<TWEA) | (1<<TWEN) | (1<<TWIE);
		}
		break;
		
		
		
		//MASTER-mode states follow.
		
    case TW_MSLA_W:	//we just tried to send the SLAVE ADDRESS in Master Mode.
		if(TWS_WRITE_ACK == Status) 	// we got an ACK back from the desired SLA+W transmission
		{
			TWDR = TW_MTxBuf[TW_MTxBufIndex++];	//send out Cmd
			TWISR_state = TW_MCMD_W;
			TWCR = (1<<TWINT) | (1<<TWEA) | (1<<TWEN) | (1<<TWIE);
		}
		else  //anything else is Unexpected or an Error result.
		{ //We simply delete msgs that couldn't be sent as they will be resent in 10secs anyway.
			TWCR = (1<<TWINT) | (1<<TWEA) | (1<<TWSTO) | (1<<TWEN) | (1<<TWIE);
			TWISR_state = TW_IDLE;	//Reset the state machine.
			TW_MTxBufCnt = 0;
			TW_MTxBufIndex = 0;
			TXmsgDelete;                  //Delete this just-sent msg from the TX buffer.
			SMLOCK = 0;
		}
		break;
		
    case TW_MCMD_W:	//we just sent a Master COMMAND byte or a Data byte
		if(TWS_TXDATA_ACK == Status) 	// we got an ACK from data we sent.
		{
			if(TW_MTxBufCnt > TW_MTxBufIndex)
			{
				TWDR = TW_MTxBuf[TW_MTxBufIndex++];	//send out Data byte
				TWISR_state = TW_MCMD_W;
				TWCR = (1<<TWINT) | (1<<TWEA) | (1<<TWEN) | (1<<TWIE);
			}
			else  //we've sent everything in the buffer, so send STOP now.
			{
				TWCR = (1<<TWINT) | (1<<TWEA) | (1<<TWSTO) | (1<<TWEN) | (1<<TWIE);
				TWISR_state = TW_IDLE;
				TW_MTxBufCnt = 0;
				TW_MTxBufIndex = 0;
				TXmsgDelete;                  //Delete this just-sent msg from the TX buffer.
				SMLOCK = 0;
			}
		}
		else  //Unexpected or Error response.
		{
			TWCR = (1<<TWINT) | (1<<TWEA) | (1<<TWSTO) | (1<<TWEN) | (1<<TWIE);
			TWISR_state = TW_IDLE;
			TW_MTxBufCnt = 0;
			TW_MTxBufIndex = 0;
			TXmsgDelete;                  //Delete this just-sent msg from the TX buffer.
			SMLOCK = 0;
		}
		break;
		
	} // end of switch(TWISR_state)
}


/* *************************************************************************
 *
 *   Foreground SMBus Command Interpreter
 *
 ************************************************************************* */


void SMB_CmdInterpreter(void)
{
	unsigned char temp;
	
	if(SMB_GenBusTimeout == TWI_CmdFlags)	//The ISR detected an error condition.
	{
		TWI_CmdFlags = 0;			//clear the flag that brought us here.
		//start the 26mS timer.  When it is done, the Timer handler will re-init the TWI peripheral.
		SetGenericTimer(SMBfaultTimer, 26);
		return;
	}
	else if(SMB_SetUpReply == TWI_CmdFlags)	//interpret a 'Read' Command.
	{
		TWI_CmdFlags = 0;			//clear the flag that brought us here.
		TW_TxBufIndex = 0;			//initialize
		TW_TxBufCnt = 0;			//initialize
		if(0 != SMB_ReadCmd[CurrentCmd]())	//After interpreting, was there an error??
		{
			SMBvariables[SMBV_BattStatus][lobyte] |= SMBerr_UnsuptdCommand;
			SetGenericTimer(SMBfaultTimer, 26); //generate a bus timeout error.
			TW_TxBufIndex = 0;		  //Wipe out anything in the buffer, just in case.
			TW_TxBufCnt = 0;			
			return;
		}
		else //generate PEC now for the *entire* transaction, including the original request!
		{
			//Assume (TW_TxBufIndex == 0) and (TW_TxBufCtr == # of bytes of data, not incl PEC).
			temp = FastCRC(0, (TWAR & 0xFE));	//use the SLA+W address
			temp = FastCRC(temp, CurrentCmd);
			temp = FastCRC(temp, (TWAR | 1));	//use the SLA+R address
			
			do {temp = FastCRC(temp, TW_TxBuf[TW_TxBufIndex++]);}
			while(TW_TxBufIndex != TW_TxBufCnt);
			
			TW_TxBuf[TW_TxBufIndex] = temp;	//append the CRC value on the end.
			TW_TxBufCnt++;			//increase the byte count for the PEC.
			TW_TxBufIndex = 0;		//Reset the buffer pointer.
			TWCR = (1<<TWINT) | (1<<TWEA) | (1<<TWEN) | (1<<TWIE);	//have TWI continue from where it stalled.
		}
		return;
	}
	else if(SMB_GotCmdData == TWI_CmdFlags)	//process some received command+data.
	{
		//NOTE: as of 6/17/2005, TWI_CmdFlags should NOT be cleared until we are
		// completely done processing this message, else the TWI ISR will overwrite
		// the RX buffer and won't respond with BUSY as it should.  Note also that
		// the TWI is fully re-enabled when entering here, so we MUST NOT write
		// to any of the TWI h/w registers or we could create havoc.  -rgf
		
		if(UsePEC)				//check the CRC of the received packet.
		{
			temp = 0;				//use this as our CRC value.
			
			do { temp = FastCRC(temp, TW_RxBuf[TW_RxBufIndex++]); }
			while(TW_RxBufCnt != TW_RxBufIndex);
			
			if(temp)	//The result of a CRC check SHOULD be =0 if all was ok.
			{
				SMBvariables[SMBV_BattStatus][lobyte] |= SMBerr_UnknownError;
				
				//start the 26mS timer to generate a bus timeout error.
				SetGenericTimer(SMBfaultTimer, 26);
				TWI_CmdFlags = 0;		//clear the flag that brought us here.
				return;
			}
		}
		
		//The rcvd message is valid enough to warrant calling the command's handler now.
		
		
		//Note that none of the regular SMBus commands use Block Mode to send info
		// *TO* the Smart Battery, so TW_RxBuf[2] will NEVER be a byte count, but
		// will always be DATA.  For OptionalMfgFunction(5), the associated command
		// function itself is aware that [2] is the byte count and that the actual
		// data starts at offset=[3].
		TW_RxBufIndex = 2;			//point to the first byte of Received Data.
		
		
		if(0 != SMB_WriteCmd[CurrentCmd]())	//After interpreting, was there an error??
		{
			TW_RxBufIndex = 0;		//Wipe out anything in the buffer, just in case.
			TW_RxBufCnt = 0;			
			
			//start the 26mS timer to generate a bus timeout error.
			SetGenericTimer(SMBfaultTimer, 26);
			TWI_CmdFlags = 0;			//clear the flag that brought us here.
			return;
		}
		//At this point it looks like everything went OK.
		TWI_CmdFlags = 0;			//clear the flag that brought us here.
		return;
	}
}


//This function restores the SMBus & the TWI_ISR state machine to normal after
//  we have deliberately generated a bus timeout error (in order to tell the
//  Master that something was wrong with his last command).
void SMB_RestoreBus(void)
{
	TWCR = 0;				//shut down the peripheral
	TWISR_state = TW_IDLE;	//force an init of the state machine
	TWAR = BATTERY_ADDR<<1;			
	TWCR = (1<<TWINT) | (1<<TWEA) | (1<<TWEN) | (1<<TWIE);	// | (1<<TWSTO)  re-enable
	
	//Note that we must be careful not to generate some kind of bus error
	// as a result of waking back up, or we will get into an endless loop
	// by generating another bus timeout if the IDLE state doesn't like
	// something it sees when it comes back to life.
}

