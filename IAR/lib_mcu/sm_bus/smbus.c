/*This file has been prepared for Doxygen automatic documentation generation.*/
/*! \file *********************************************************************
 *
 * \brief
 *     The communication file reads a command from the SMB bus and sends
 *     r/w, data and length information to SBS layer.
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

#include "common.h"

#define MODULE_SMBUS		/* ensure that we instantiate our variables in smbus.h */

#include "smbus.h"
#include "SBS_commands.h"
#include "interpreter.h"
#include "timer.h"
#include "battery_pack_parameters.h"
#include "gas_gauging.h"

//---------------------------------------------------------------------
uint8_t TEST50US = 0;
uint8_t SMLOCK   = 0;	      //prevents Master bus grab attempts WHILE BUS IS IN MASTER MODE.

//uint8_t TW_MTxBuf[8];	      //Master-mode TX buffer
//uint8_t TW_MTxBufCnt   = 0; //how many valid bytes are in the buffer
//uint8_t TW_MTxBufIndex = 0;

// Note for the buffers below, these must be able to contain:
// Slave Address, SMBus Command, Byte Count (if Block-mode), up to 32 bytes, plus PEC.
uint8_t TW_TxBuf[36];	      //must be long enough for any outbound strings
uint8_t TW_TxBufCnt    = 0; //how many valid bytes are in the buffer
uint8_t TW_TxBufIndex  = 0;

uint8_t TW_RxBuf[36];	      //In Application mode (non-ISP mode), only receive WORD commands.
int8_t TW_RxBufCnt    = 0;
uint8_t TW_RxBufIndex  = 0;


//This byte contains flags from the TWI ISR to tell the Foreground code what to do.
//If this byte is ever non-zero, the foreground code will act on its contents.
//Although it is written by both the ISR and the Foreground code, it does not
//  need to be declared VOLATILE because the SMBus is halted until the foreground
//  code finishes processing the associated command and has cleared this flag byte.
volatile uint8_t TWI_CmdFlags;
  #define SMB_GenBusTimeout 1	/* Tell Foreground to generate a bus timeout, as we saw an error! */
  #define SMB_SetUpReply    2	/* Have Foreground set up TW_TxBuf[]. */
  #define SMB_GotCmdData    4	/* Have Foreground interpret the complete received command. */

uint8_t CurrentCmd = 0xFF;
uint8_t UsePEC = 0;	//PEC usage is disabled by default.



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
}




/* *************************************************************************
 *
 *   SMBus Wakeup Interrupt
 *
 ************************************************************************* */


//This wakes up a battery from sleep mode into the "On" state, per sbdat110, 4.4.2

#pragma vector = TWI_BUS_C_D_vect
__interrupt void TWICD_ISR(void)
{
	//clear bits per sbdat110, 4.4.2
	//SMBvariables[SMBV_BattMode][hibyte] &= ~(0xE3);	
	
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
	uint8_t temp = 0;
	
	temp = SMB_PEC(temp, (TWAR & 0xFE));	//use the SLA+W address
	temp = SMB_PEC(temp, CurrentCmd);
	temp = SMB_PEC(temp, (TWAR | 1));	//use the SLA+R address
	
	TW_TxBuf[0] = SBSDATA_BLOCK_LENGTH;//Block Length
	temp = SMB_PEC(temp, TW_TxBuf[0]);
	
	TW_TxBufIndex = 1;
	TW_TxBufCnt = SBSDATA_BLOCK_LENGTH+1;
	
	do {
		TW_TxBuf[TW_TxBufIndex] = (uint8_t) sbsReply->payload[TW_TxBufIndex-1];
		temp = SMB_PEC(temp, TW_TxBuf[TW_TxBufIndex++]);
	}while(TW_TxBufIndex != TW_TxBufCnt);
	
	TW_TxBuf[TW_TxBufIndex] = temp;	//append the CRC value on the end.
	TW_TxBufCnt++;			//increase the byte count for the PEC.
	TW_TxBufIndex = 0;		//Reset the buffer pointer.
	TWCR = (1<<TWINT) | (1<<TWEA) | (1<<TWEN) | (1<<TWIE);	//have TWI continue from where it stalled.
	
	
}

/*! \brief  Reply to a SBS read word command.
 *
 *	\param sbsReply Is a pointer to the #SBS_command_t struct containing the data to transmit.
 */	
void Comm_SbsWordCommandReply( SBS_command_t* sbsReply )
{
	uint8_t temp = 0;
	
	TW_TxBuf[0] = (uint8_t) sbsReply->payload[0];
	TW_TxBuf[1] = (uint8_t) sbsReply->payload[1];;
	
	temp = SMB_PEC(temp, (TWAR & 0xFE));	//use the SLA+W address
	temp = SMB_PEC(temp, CurrentCmd);
	temp = SMB_PEC(temp, (TWAR | 1));	//use the SLA+R address

	//Calculate PEC for Word
	temp = SMB_PEC(temp, TW_TxBuf[0]);
	temp = SMB_PEC(temp, TW_TxBuf[1]);
	
	TW_TxBuf[2] = temp;		//Append the CRC value on the end.
	TW_TxBufCnt = 3;		//Set the byte count for the Word + PEC.
	TW_TxBufIndex = 0;		//Reset the buffer pointer.
	
	TWCR = (1<<TWINT) | (1<<TWEA) | (1<<TWEN) | (1<<TWIE);	//have TWI continue from where it stalled.

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
	uint8_t temp = 0;
	
	switch(TWI_CmdFlags)
	{
	    case SMB_GenBusTimeout:	//The ISR detected an error condition.
		
		TWI_CmdFlags = 0;			//clear the flag that brought us here.
		sbsCmdDestination->command = (SBS_commands_t) 0xFF;         //clear command register
		//start the 26mS timer.  When it is done, the Timer handler will re-init the TWI peripheral.
		SetGenericTimer(SMBfaultTimer, 26);
		break;
		
	    case SMB_SetUpReply:       //interpret a 'Read' Command.
		TWI_CmdFlags = 0;			//clear the flag that brought us here.
		TW_TxBufIndex = 0;			//initialize
		TW_TxBufCnt = 0;			//initialize
		sbsCmdDestination->command = (SBS_commands_t) (CurrentCmd | SBS_READ);
		receiveComplete = true;
		break;
		
	    case SMB_GotCmdData:	//process some received command+data.
		//NOTE: as of 6/17/2005, TWI_CmdFlags should NOT be cleared until we are
		// completely done processing this message, else the TWI ISR will overwrite
		// the RX buffer and won't respond with BUSY as it should.  Note also that
		// the TWI is fully re-enabled when entering here, so we MUST NOT write
		// to any of the TWI h/w registers or we could create havoc.  -rgf
		if(UsePEC)				//check the CRC of the received packet.
		{
			temp = 0;				//use this as our CRC value.
			
			do{
				temp = SMB_PEC(temp, TW_RxBuf[TW_RxBufIndex++]);
			}while(TW_RxBufCnt != TW_RxBufIndex);
			
			if(temp)	//The result of a CRC check SHOULD be =0 if all was ok.
			{
				//start the 26mS timer to generate a bus timeout error.
				SetGenericTimer(SMBfaultTimer, 26);
				receiveComplete = false;
				TWI_CmdFlags = 0;			//clear the flag that brought us here.
				break;
			}
		}
		
		//If more bytes than a word, a block has been received and omit length byte
		// Addr + Cmd + DataWord = 4 bytes
		if(TW_RxBufCnt > 4){
			TW_RxBufIndex = 3;		//Start of data in block
		}else{
			TW_RxBufIndex = 2;		//Start of data in word
		}
		
		temp = 0;
		
		do{
			sbsCmdDestination->payload[temp++] = TW_RxBuf[TW_RxBufIndex++];
		}while(TW_RxBufCnt != TW_RxBufIndex);
		
		sbsCmdDestination->command = (SBS_commands_t) (CurrentCmd | SBS_WRITE);
		receiveComplete = true;

		//At this point it looks like everything went OK.
		TW_RxBufIndex = 0;		//Wipe out anything in the buffer, just in case.
		TW_RxBufCnt = 0;			
		TWI_CmdFlags = 0;			//clear the flag that brought us here.
		break;
	    default:
		receiveComplete = false;
	}
	
	return receiveComplete;
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
enum /*TWISR_State*/ {TW_IDLE=0, TW_Wait4Stop, TW_Wait4Cmd, TW_Wait4RW, TW_Wait4Data, TW_ReplyData };

uint8_t TWISR_state = TW_IDLE;	//state variable


#pragma vector = TWI_vect
__interrupt void TWI_ISR(void)
{
	//Command-related feature flags
	static uint8_t TWISR_CmdFeatures = 0;
	
	uint8_t Status;
	uint8_t tmp;
	
	//This identifies what caused the interrupt to fire.
	Status = TWSR & 0xF8;
	
	switch(TWISR_state){
	    default:

	    //If not SLA_W or RSTOP, is an error!
	    case TW_IDLE:
		
		// Saw Slave address match with a Write bit
		if(TWS_SLA_W == Status)
		{
			if(TWI_CmdFlags == SMB_GotCmdData)
			{
				SetGenericTimer(SMBfaultTimer, 26);
				
				//Assert that we're 'busy' to SMBus Master by leaving TWEA *off* until we get a STOP.
				//Note that this is 'legal' because we have ALREADY sent an ACK to our Slave Address,
				// as is required by the SMBus specification.
				
				TWISR_state = TW_Wait4Stop;
				TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWIE);	//must NOT re-enable ACKing!
				break;
			}
			
			// Still waiting for command data.
			else
			{
				TWISR_state = TW_Wait4Cmd;
			}
		}
		//Saw a Stop, possibly left over from previous cmd.
		else if(TWS_RSTOP == Status)
		{
			//Everything is probably OK.  Take no action.
		}
		
		// Had some type of error!
		else
		{
			TWI_CmdFlags = SMB_GenBusTimeout;  //Generate a bus timeout.
			TWCR = (1<<TWEA) | (1<<TWEN);      //Disable int, and DON'T clear the TWINT flag!
			TWISR_state = TW_IDLE;             //Reset the state machine.
			break;
		}
		TWCR = (1<<TWINT) | (1<<TWEA) | (1<<TWEN) | (1<<TWIE);	//must re-enable ACKing
		break;
		

	    case TW_Wait4Stop:
		//Saw a Stop, possibly left over from previous cmd.
		if(TWS_RSTOP == Status)
		{
			TWCR = (1<<TWINT) | (1<<TWEA) | (1<<TWEN) | (1<<TWIE);	//must re-enable ACKing
			TWISR_state = TW_IDLE;			//Reset the state machine.
		}
		//Error! Did not see a stop as expected. NACKing the rest.
		else
		{
			TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWIE);	//must NOT re-enable ACKing yet!
		}
		break;
		
		
		//SLAVE-mode states follow.
		
	    // Upon entry, we expect to have received a Cmd byte.
	    case TW_Wait4Cmd:

		//It appears that we have received a Command byte now.
		if(TWS_RCMD == Status)
		{
			tmp = TWDR;
			
			//Is the Cmd within valid range?
			if(tmp <= HIGHEST_SMB_CMD)
			{
				CurrentCmd = tmp;		//Save a copy.
				tmp = SM_Cmd_Table[tmp][0];	//Grab the Command Characteristics/Features flags.
				if(tmp & SMBslave)		//Is the Command valid for Slaves?
				{				//The command appears to be valid.
					TWISR_CmdFeatures = tmp;	//Save the Feature flags for use in Wait4RW state.
					TWISR_state = TW_Wait4RW;	//set up next state
					TWCR = (1<<TWINT) | (1<<TWEA) | (1<<TWEN) | (1<<TWIE);	//enable ACKing
					break;
				}
			}
		}
		//In all cases except those that 'break' (above), it's an error.
		TWI_CmdFlags = SMB_GenBusTimeout;  //Generate a bus timeout.
		TWCR = (1<<TWEA) | (1<<TWEN);      //Disable int, and DON'T clear the TWINT flag!
		TWISR_state = TW_IDLE;             //Reset the state machine.
		break;
		
	    //We will now find out if we will RX more, or we need to TX a reply instead.
	    case TW_Wait4RW:
		
		//It is a WRITE-type command. Prep the RX buffer to accept more data.
		if(TWS_RDATA == Status)
		{
			//NOTE: except for OptionalMfgFunction5, all WRITE cmds are 2-byte, plus optional PEC.
			//Place all bytes of the transaction into the buffer so we can do a PEC on it if needed.
			
			TW_RxBuf[0] = TWAR & 0xFE;	//store everything incl. the slave address for computing PEC.
			TW_RxBuf[1] = CurrentCmd;	//store the previously-send Command.
			TW_RxBuf[2] = TWDR;		//store this first DATA byte
			TW_RxBufIndex = 3;		//use RxBufIndex as the index to store data in the buffer.
			
			//Is it a Write-WORD command type?
			if(TWISR_CmdFeatures & SCWW)
			{
				TW_RxBufCnt = 1;		//We expect 1 more data byte, and possibly PEC after that.
			}
			
			//Is it a write-BLOCK command (must be OptionalMfgFunction5 then)
			else if(TWISR_CmdFeatures & SCWG)
			{
				tmp = TWDR;
				if((tmp >= 1) && (tmp <= 32))
					TW_RxBufCnt = TWDR;
				else
				{
					TWI_CmdFlags = SMB_GenBusTimeout;  //Generate a bus timeout.
					TWCR = (1<<TWEA) | (1<<TWEN);      //Disable int, and DON'T clear the TWINT flag!
					TWISR_state = TW_IDLE;             //Reset the state machine.
					break;
				}
			}

			//Error! This Command doesn't allow EITHER word OR group/block Writes! It's Read-only!
			else
			{
				TWI_CmdFlags = SMB_GenBusTimeout;  //Generate a bus timeout.
				TWCR = (1<<TWEA) | (1<<TWEN);      //Disable int, and DON'T clear the TWINT flag!
				TWISR_state = TW_IDLE;             //Reset the state machine.
				break;
			}
			TWISR_state = TW_Wait4Data;
			TWCR = (1<<TWINT) | (1<<TWEA) | (1<<TWEN) | (1<<TWIE);	//enable ACKing
		}
		
		//We saw a re-Start, so must be getting ready for a Read cmd.
		else if(TWS_REPEAT == Status)
		{	
			//Must now interpret previously-sent CurrentCmd & set up Reply data.
			
			//Is it a 'ReadWord' or 'ReadGroup' command type?
			if(TWISR_CmdFeatures & (SCRW | SCRG))
			{
				TWI_CmdFlags = SMB_SetUpReply;    //Foreground decoder will set up TWCR.
				TWISR_state = TW_ReplyData;       //Move to next state.
			}

			//Error! Not a valid read command.
			else
			{
				TWI_CmdFlags = SMB_GenBusTimeout;  //Generate a bus timeout.
				TWCR = (1<<TWEA) | (1<<TWEN);      //Disable int, and DON'T clear the TWINT flag!
				TWISR_state = TW_IDLE;             //Reset the state machine.
				break;
			}
			TWCR = (1<<TWEA) | (1<<TWEN);         //disable int, and DON'T clear the TWINT flag!
		}
		
		// Some type of error!
		else
		{
			TWI_CmdFlags = SMB_GenBusTimeout;  //Generate a bus timeout.
			TWCR = (1<<TWEA) | (1<<TWEN);      //Disable int, and DON'T clear the TWINT flag!
			TWISR_state = TW_IDLE;             //Reset the state machine.
			break;
		}
		break;
		
		
	    //We are in Slave Receive operating mode.
	    case TW_Wait4Data:

		// Received a data byte.
		if(TWS_RDATA == Status)	
		{
			tmp = TWDR;
			
			//Are we past the PEC byte and still getting more data?
			if(--TW_RxBufCnt < -1)
			{
				TWI_CmdFlags = SMB_GenBusTimeout;  //Generate a bus timeout.
				TWCR = (1<<TWEA) | (1<<TWEN);      //Disable int, and DON'T clear the TWINT flag!
				TWISR_state = TW_IDLE;             //Reset the state machine.
				break;
			}
			//Store the byte in the buffer.
			TW_RxBuf[TW_RxBufIndex++] = tmp;
		}
	
		//Got a STOP; All done RXing data now.
		else if(TWS_RSTOP == Status)
		{
			//Note: If we get a STOP prematurely, then we simply ignore the command,
			//since it is too late to inform the Master of the error.
			if((TW_RxBufCnt > 0) || (TW_RxBufCnt < -1))
			{
				// We got a premature STOP or too much data; ERROR!
				// Error handling if needed.
			}
			else
			{
				if(0 == TW_RxBufCnt){
					UsePEC = 0;            //there is no PEC coming for this packet.
				}else if(-1 == TW_RxBufCnt){
					UsePEC = 1;            //PEC was included.
				}
				
				TW_RxBufCnt = TW_RxBufIndex;      //re-use the Write Index's value as the Valid Byte Count.
				TW_RxBufIndex = 0;                //clear the index in preparation for interpreting it.
				TWI_CmdFlags = SMB_GotCmdData;    //tell Foreground to process this now.
				//Note that when (TWI_CmdFlags == SMB_GotCmdData), TWI ISR will respond with BUSY condition.
			}
			TWISR_state = TW_IDLE;		//Reset the state machine in all cases.
		}

		//Some type of error during transmission!
		else
		{
			TWI_CmdFlags = SMB_GenBusTimeout;  //Generate a bus timeout.
			TWCR = (1<<TWEA) | (1<<TWEN);      //Disable int, and DON'T clear the TWINT flag!
			TWISR_state = TW_IDLE;             //Reset the state machine.
			break;
		}
		
		TWCR = (1<<TWINT) | (1<<TWEA) | (1<<TWEN) | (1<<TWIE);	//enable ACKing
		break;
		

	    // We are now in Slave Transmit operating mode.
	    // The foreground code has set up the response that we are now sending.
	    // Note: TW_TxBufCnt *always* includes the PEC byte! Since we don't
	    // know whether the Master actually WANTS the PEC byte or not, we will
	    // always TRY to send it, regardless of the state of the UsePEC flag.
	    // If the Master does NOT want it, we will get a NAK while the PEC
	    // byte is still in the buffer.  In the rare case where we send it all,
	    // including the PEC byte, but we still get an ACK back, the TWI module
	    // will be off-line anyway due to not setting the TWEA bit after sending PEC,
	    // and we will therefore be unable to flag that an error has occurred.
	    case TW_ReplyData:
		
		//Send out Reply data
		if((TWS_SLA_R == Status) || (TWS_RACK == Status))
		{
			//Get data next byte from buffer and send data out.
			TWDR = TW_TxBuf[TW_TxBufIndex++];
			
			//Have we emptied the buffer, incl. PEC?
			if(--TW_TxBufCnt == 0)
			{	// Yes, so don't set TWEA.
				TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWIE);
			}
			else
			{	// No, so assert TWEA.
				TWCR = (1<<TWINT) | (1<<TWEA) | (1<<TWEN) | (1<<TWIE);
			}
		}
		
		//We may have gotten this validly or as an Error.
		else if(TWS_RNAK == Status)
		{
			//Not an error. Master didn't want PEC; clear UsePEC flag!
			if(TW_TxBufCnt == 1)
			{
				TW_TxBufCnt = 0;	//clear the buffer too.
				UsePEC = 0;
			}
			//Not an error. Master wanted PEC (and got it); assert UsePEC.
			else if(TW_TxBufCnt == 0)
			{
				UsePEC = 1;
			}
			
			//Some kind of error occurred; we got NAK too early!
			else
			{
				//Note: the TWI module is now OFF-LINE, so we can't inform Host of this error!!
			}
			
			//In all cases, go back to IDLE.
			TWISR_state = TW_IDLE;
			TWCR = (1<<TWINT) | (1<<TWEA) | (1<<TWEN) | (1<<TWIE);
		}
		
		// if(TWS_FINAL == Status)	//ERROR: we got an ACK but we have no more data!
		// Since the TWI module is now in "Not Addressed Slave" mode, we can't flag the error
		// back to the Master DURING this transaction; we WILL assert an error status though.
		else
		{
			TWISR_state = TW_IDLE;	//Reset the state machine.
			TWCR = (1<<TWINT) | (1<<TWEA) | (1<<TWEN) | (1<<TWIE);
		}
		break;
	} // end of switch(TWISR_state)
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
	return (TW_IDLE == TWISR_state);
}


/*! \brief  Check Communication Status
 *
 *  This function returns 0, if it is nothing to handle.
 *  If it is > 0, then execute Comm_Handle()
 *
 *  \return 0 if there is nothing to handle, > 0  there is
 */
uint8_t Comm_Flag(void)
{
	return TWISR_state;  // TW_IDLE=0
}



//This function restores the SMBus & the TWI_ISR state machine to normal after
//  we have deliberately generated a bus timeout error (in order to tell the
//  Master that something was wrong with his last command).
void SMB_RestoreBus(void)
{
	TWCR = 0;				//shut down the peripheral
	TWISR_state = TW_IDLE;	//force an init of the state machine
	TWAR = BATTERY_ADDR<<1;			
	TWCR = (1<<TWINT) | (1<<TWEA) | (1<<TWEN) | (1<<TWIE);// | (1<<TWSTO); // re-enable
	
	//Note that we must be careful not to generate some kind of bus error
	// as a result of waking back up, or we will get into an endless loop
	// by generating another bus timeout if the IDLE state doesn't like
	// something it sees when it comes back to life.
}


/* *************************************************************************
 *
 *   Utilities for SMBus commmunications
 *
 ************************************************************************* */

#ifdef SMB_PEC_ROUTINE_SLOW

/* \brief PEC CRC lookup function
 *
 * This function calculates the PEC value of one byte, using the
 * PEC calculated so far as a starting point.
 *
 * \param  newByte    Value to calculate PEC of.
 * \param  lastCRC    The current PEC.
 */
uint8_t SMB_PEC(uint8_t lastCRC, uint8_t newByte)
{
	// Initial XOR
	lastCRC ^= newByte;
	
	for(uint8_t i = 0; i < 8; i++){
		if (lastCRC & 0x80){
			lastCRC = (lastCRC << 1) ^ SMB_PEC_CRC_POLYNOME;
		}else{
			lastCRC <<= 1;
		}
	}
	return lastCRC;
}

#elif defined(SMB_PEC_ROUTINE_MEDIUM)

//! Tables of crc values stored in flash.
__flash uint8_t crcTable1[16] = {0x00,0x07,0x0E,0x09, 0x1c,0x1b,0x12,0x15, 0x38,0x3F,0x36,0x31, 0x24,0x23,0x2A,0x2D};
__flash uint8_t crcTable2[16] = {0x00,0x70,0xE0,0x90, 0xC1,0xB1,0x21,0x51, 0x83,0xF3,0x63,0x13, 0x42,0x32,0xA2,0xD2};

/* \brief PEC CRC lookup function
 *
 * This function uses a table stored in flash to look up the
 * PEC value of one byte, using the PEC calculated so far as a
 * starting point.
 *
 * \param  newByte    Value to calculate PEC of.
 * \param  lastCRC    The current PEC.
 */
uint8_t SMB_PEC(uint8_t lastCRC, uint8_t newByte)
{
	uint8_t index;
	
	index = newByte;
	index ^= lastCRC;
	index >>= 4;
	lastCRC &= 0x0F;
	lastCRC ^= crcTable2[index];
	
	index = lastCRC;
	index ^= newByte;
	index &= 0x0F;
	lastCRC &= 0xF0;
	lastCRC ^= crcTable1[index];
	
	return lastCRC;
}

#elif defined(SMB_PEC_ROUTINE_FAST)

//! Table of crc values stored in flash.
__flash uint8_t crcTable[256] =	{	
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
 * \param  newByte    Value to calculate PEC of.
 * \param  lastCRC    The current PEC.
 */
uint8_t SMB_PEC(uint8_t lastCRC, uint8_t newByte)
{
	lastCRC ^= newByte;
	lastCRC = crcTable[lastCRC];
	
	return lastCRC;
}


#else
#error One SMB_PEC_ROUTINE must be defined.
#endif


/* Some important notes from the SMBus specs:

Insertion or removal of a Smart Battery may be detected when the ‘Safety Signal?transitions from or to an
open-circuit value (>100k.)

When an SMBus device acting as the bus master detects an error, it must attempt to return the bus to the idle
state by generating a STOP condition.

The Smart Battery must ALWAYS acknowledge its own address. Failure to do so might cause the SMBus
Host or Smart Battery Charger to incorrectly assume the Smart Battery is NOT present in the system,
although the ‘Safety Signal?can be used to detect the presence of a Smart Battery in a system. Note
however that the Smart Battery may choose not to acknowledge any byte following its address if it is
busy or otherwise unable to respond. If this occurs, the requestor should re-try the data request.

After each SMBus transaction directed to the Smart Battery device address, the Smart Battery must place
the appropriate error code in the lower nibble of the BatteryStatus() register. If the transaction completed
successfully, the error codes should be cleared to signify that no error was detected. Timeout and other
errors not described by one of the error code types may be signaled with an Unknown Error.

Another device may try to interrogate the battery immediately after a transaction from the Host.
The safest method to insure that the read of BatteryStatus() corresponds to the most recently read data value
is to perform this read of BatteryStatus() immediately after the read of the initial data value. This may be
accomplished by issuing a SMBus START condition after the SMBus STOP condition from the previous
transmission.

A bus master is required to check for bus idle time of 50 us.

The Smart Battery enters the “On State whenever it detects that the SMBus Clock and Data lines go high.
The battery should be active and able to communicate via the SMBus within 1 ms of detecting these SMBus
lines going high.

The Smart Battery may not begin *broadcasting* ChargingVoltage(), ChargingCurrent() or AlarmWarning()
messages to either the SMBus Host or Smart Battery Charger for at least 10 seconds after entering the “On
State.?
When the Smart Battery enters the “On State?the following values must be reinitialized:
Function (Data Value) 		Initial Value 		
---------------------	----------------------------	
BatteryMode() 		Bit 15: CAPACITY_MODE=0
			Bit 14: CHARGER_MODE=0
			Bit 13: ALARM MODE=0
			Bit 9: PRIMARY_BATTERY=0
			Bit 8: CHARGE_CONTROLLER_ENABLED=0


The Smart Battery may enter the “Off State?whenever the SMBus Clock and Data lines both remain low
for greater than 2.5 seconds.

There is no limit to the speed (other than SMBus limits) or rate at which data may be requested from the
Smart Battery. Continuous data polling at high rates is permitted and allowed, though not encouraged due
to limitations on SMBus bandwidth and availability. The Smart Battery may delay any data request by
holding the CLOCK line low for up to 25 ms. This may be done in order to re-calculate the requested data
value or to retrieve data from a storage device.


*/
