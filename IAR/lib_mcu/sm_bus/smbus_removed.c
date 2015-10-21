enum /*TWISR_State*/ {TW_IDLE=0, TW_Wait4Stop, TW_Wait4Cmd, TW_Wait4RW, TW_Wait4Data, TW_ReplyData, TW_MSLA_W, TW_MCMD_W, TW_MDATA_W };

#pragma vector = TWI_vect
__interrupt void TWI_ISR(void)
{
	static uint8_t TWISR_CmdFeatures = 0;	//Command-related feature flags
	uint8_t Status;
	uint8_t tmp;

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
//		SMBvariables[SMBV_BattStatus][lobyte] |= SMBerr_UnknownError;
//		TWI_CmdFlags = SMB_GenBusTimeout;	//generate a bus timeout.
//		TWCR = (1<<TWEA) | (1<<TWEN);     //disable int, and DON'T clear the TWINT flag!
//		TWISR_state = TW_IDLE;		//Reset the state machine.
		TWI_CmdFlags = 0;	//generate a bus timeout.
		TWCR = (1<<TWINT) | (0<<TWEA) | (1<<TWEN) | (1<<TWIE);
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
	uint8_t temp;

	if(SMB_GenBusTimeout == TWI_CmdFlags)	//The ISR detected an error condition.
	{
		TWI_CmdFlags = 0;			//clear the flag that brought us here.
		//start the 26mS timer.  When it is done, the Timer handler will re-init the TWI peripheral.
		//SetGenericTimer(SMBfaultTimer, 26);
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




void FillResponseInt(uint16_t info)
{
	TW_TxBuf[0] = (uint8_t) info;
	TW_TxBuf[1] = (uint8_t) (info >> 8);

	TW_TxBufIndex = 0;
	TW_TxBufCnt = 2;
}


void FillResponseStr(char __flash * source)
{
	uint8_t * dest = TW_TxBuf;
	uint8_t ctr = 0;

	for(;;)
	{
		if(*dest++ = *source++)
			ctr++;
		else
			break;
	}

	TW_TxBufIndex = 0;
	TW_TxBufCnt = ctr;
}




/* ************************************************** */

uint8_t SMBR_MfrAccess(void)	// Cmd # 0
{
  FillResponseInt(SMBvar_int[SMBV_MfrAccess]);
  return 0;
}



uint8_t SMBR_RemCapAlm(void)	// 1
{
  FillResponseInt(SMBvar_int[SMBV_RemCapAlm]);
  return 0;
}


uint8_t SMBR_RemTimeAlm(void)	// 2
{
  FillResponseInt(SMBvar_int[SMBV_RemTimeAlm]);
  return 0;
}


uint8_t SMBR_BattMode(void)	// 3
{
  FillResponseInt(SMBvar_int[SMBV_BattMode]);
  return 0;
}


uint8_t SMBR_AtRate(void)		// 4
{
  FillResponseInt((uint16_t)battParams_sram.atRate);
  return 0;
}


uint8_t SMBR_AtRateTTF(void)	// 5
{
  uint16_t temp = GASG_TimeToFull(battParams_sram.atRate); //AtRateTTF();
  SMBvar_int[SMBV_AtRateTTF] = temp;	//save local copy of result for DEBUG PURPOSES ONLY
  FillResponseInt(temp);
  return 0;
}


uint8_t SMBR_AtRateTTE(void)	// 6
{
  uint16_t temp = GASG_TimeToEmpty(battParams_sram.atRate); //AtRateTTE();
  SMBvar_int[SMBV_AtRateTTE] = temp;
  FillResponseInt(temp);
  return 0;
}


uint8_t SMBR_AtRateOK(void)	// 7
{
  uint16_t temp = GASG_AtRateOK(battParams_sram.atRate); //AtRateOK();
  SMBvar_int[SMBV_AtRateOK] = temp;
  FillResponseInt(temp);
  return 0;
}

extern uint16_t GetTemperature(void);

uint8_t SMBR_Temperature(void)	//  8
{
  uint16_t temp = GetTemperature();
  SMBvar_int[SMBV_Temperature] = temp;
  FillResponseInt(temp);
  return 0;
}


uint8_t SMBR_Voltage(void)	//  9
{
  uint16_t volt = VADC_readCellVoltage( CELL4 )+
	              VADC_readCellVoltage( CELL3 )+
	              VADC_readCellVoltage( CELL2 )+
	              VADC_readCellVoltage( CELL1 ); //GetVoltage();
  SMBvar_int[SMBV_Voltage] = volt;
  FillResponseInt(volt);
  return 0;
}


uint8_t SMBR_Current(void)	// 10
{
  signed int current = BATTCUR_Ticks2mA( BATTCUR_GetCurrent() ); //Current1Sec();

  SMBvar_int[SMBV_Current] = (uint16_t) current;
  FillResponseInt((uint16_t) current);
  return 0;
}


uint8_t SMBR_AvgCurrent(void)	// 11
{
  signed int current = BATTCUR_Ticks2mA( BATTCUR_GetAverageCurrent() ); //CCarray_Average();

  SMBvar_int[SMBV_AvgCurrent] = (uint16_t) current;
  FillResponseInt((uint16_t) current);
  return 0;
}


uint8_t SMBR_MaxError(void)	// 12
{
  FillResponseInt(SMBvar_int[SMBV_MaxError] = 0);
  return 0;
}


uint8_t SMBR_RelSOC(void)		// 13
{
  uint16_t temp = GASG_StateOfCharge(BATTCUR_GetAverageCurrent(), battParams_sram.capacityInCCAccumulated); //RelativeSOC();

  SMBvar_int[SMBV_RelSOC] = temp;
  FillResponseInt(temp);
  return 0;
}


uint8_t SMBR_AbsSOC(void)		// 14
{
  uint16_t temp = GASG_StateOfCharge(BATTCUR_GetAverageCurrent(), CCGASG_mAh2Acc(battParams.designCapacity)); //AbsoluteSOC();
  SMBvar_int[SMBV_AbsSOC] = temp;
  FillResponseInt(temp);
  return 0;
}


uint8_t SMBR_RemCap(void)		// 15
{
  uint16_t cap = CCGASG_Acc2mAh(GASG_RemainingCapacity(BATTCUR_GetAverageCurrent())); //RemainingCap();
  FillResponseInt(cap);
  SMBvar_int[SMBV_RemCap] = cap;
  return 0;
}


uint8_t SMBR_FullChgCap(void)	// 16
{
  uint16_t cap = battParams.fullChargeCapacity; //FullChgCap();
  FillResponseInt(cap);
  SMBvar_int[SMBV_FullChgCap] = cap;
  return 0;
}


uint8_t SMBR_RunTTE(void)		// 17
{
	uint16_t tte;
	int32_t temp = BATTCUR_GetCurrent();
	temp = BATTCUR_Ticks2mA(temp);

  tte = GASG_TimeToEmpty( (int16_t)temp ); //TimeToEmpty(0);
  FillResponseInt(tte);
  SMBvar_int[SMBV_RunTTE] = tte;
  return 0;
}


uint8_t SMBR_AvgTTE(void)		// 18
{
	uint16_t tte;
	int32_t temp = BATTCUR_GetAverageCurrent();
	temp = BATTCUR_Ticks2mA(temp);

  tte = GASG_TimeToEmpty( (int16_t)temp ); //TimeToEmpty(0);
  FillResponseInt(tte);
  SMBvar_int[SMBV_AvgTTE] = tte;
  return 0;
}


uint8_t SMBR_AvgTTF(void)		// 19
{
  	uint16_t ttf;
	int32_t temp = BATTCUR_GetAverageCurrent();
	temp = BATTCUR_Ticks2mA(temp);

  ttf = GASG_TimeToFull( (int16_t)temp ); //AvgTimeToFull();
  FillResponseInt(ttf);
  SMBvar_int[SMBV_AvgTTF] = ttf;
  return 0;
}



/* ********************************************** */

// These two messages are sent from either a Charger or a Host
uint8_t SMBR_ChgCurrent(void)	// 20
{
  SMBvariables[SMBV_BattStatus][lobyte] &= 0xF0;  //since this cmd is OK, clear Error bits per sbdat110, 4.3.2
  FillResponseInt(SMBvar_int[SMBV_ChgCurrent]);
  return 0;
}


uint8_t SMBR_ChgVoltage(void)	// 21
{
  SMBvariables[SMBV_BattStatus][lobyte] &= 0xF0;  //since this cmd is OK, clear Error bits per sbdat110, 4.3.2
  FillResponseInt(SMBvar_int[SMBV_ChgVoltage]);
  return 0;
}


/* ********************************************** */


uint8_t SMBR_BattStatus(void)	// 22
{
  FillResponseInt(SMBvar_int[SMBV_BattStatus]);
  SMBvariables[SMBV_BattStatus][lobyte] &= 0xF0;
  return 0;
}


uint8_t SMBR_CycleCount(void)	// 23
{
  FillResponseInt(SMBvar_int[SMBV_CycleCount]);
  return 0;
}


uint8_t SMBR_DesignCap(void)	// 24
{
  unsigned long temp;

  if(SMBvariables[SMBV_BattMode][hibyte] & CAPACITY_MODE)	//use mW in calculations
    temp = PACK_DESIGNCAPMW;
  else
    temp = PACK_DESIGNCAPC5;

  FillResponseInt(temp);
  SMBvar_int[SMBV_DesignCap] = (uint16_t)temp;
  return 0;
}


uint8_t SMBR_DesignVolt(void)	// 25
{
  FillResponseInt(PACK_NOMINALV);
  SMBvar_int[SMBV_DesignVolt] = PACK_NOMINALV;
  return 0;
}


uint8_t SMBR_SpecInfo(void)	// 26
{
  FillResponseInt(SMBvar_int[SMBV_SpecInfo]);	//! \todo  this value is filled in as const by init.
  return 0;
}


uint8_t SMBR_MfrDate(void)	// 27
{
  FillResponseInt(SMBvar_int[SMBV_MfrDate]);	//! \todo  this value is filled in as const by init.
  return 0;
}


uint8_t SMBR_SerialNo(void)	// 28
{
  FillResponseInt(SMBvar_int[SMBV_SerialNo]);	//! \todo  this value is filled in as const by init.
  return 0;

}


uint8_t SMBR_MfrName(void)	// 32
{
  FillResponseStr(str_MfrName);			//! \todo  Modify as needed. __flash char defined in smbus.h
  return 0;
}



uint8_t SMBR_DeviceName(void)	// 33
{
  FillResponseStr(str_DeviceName);		//! \todo  Modify as needed. __flash char defined in smbus.h
  return 0;
}


uint8_t SMBR_DeviceChem(void)	// 34
{
  FillResponseStr(str_DeviceChem);		//! \todo  Modify as needed. __flash char defined in smbus.h
  return 0;
}


uint8_t SMBR_MfrData(void)	// 35
{
  FillResponseStr(str_MfrData);			//! \todo  Modify as needed. __flash char defined in smbus.h
  return 0;
}


uint8_t SMBR_Opt5(void)		// 0x2F
{
  FillResponseInt(12345);			//! \todo  this value is defined as a constant here.
  return 0;
//  return SMBerr_ReservedCommand;
}


uint8_t SMBR_Opt4(void)		// 0x3C
{
  FillResponseInt( 54321 ); //! \todo  this value is defined as a constant here.
  return 0; // Return "OK, there are data to transmitted".

}


uint8_t SMBR_Opt3(void)		// 0x3D
{

  return SMBerr_ReservedCommand;	//unused
}


uint8_t SMBR_Opt2(void)		// 0x3E
{

  return SMBerr_ReservedCommand;	//unused
}


uint8_t SMBR_Opt1(void)		// 0x3F
{

  return SMBerr_ReservedCommand;	//unused
}


uint8_t SMBR_invalid(void)	//This should never execute, if error is caught early!
{
  return SMBerr_UnsuptdCommand;
}



//typedef uint8_t (*ptr2funcUC_V)(void);

//Table of pointers to functions, indexed from the received SMBus Command byte.
ptr2funcUC_V SMB_ReadCmd[HIGHEST_SMB_CMD+1] =
{
  SMBR_MfrAccess,	//  0
  SMBR_RemCapAlm,	//  1
  SMBR_RemTimeAlm,	//  2
  SMBR_BattMode,  	//  3
  SMBR_AtRate,     	//  4
  SMBR_AtRateTTF,  	//  5
  SMBR_AtRateTTE,  	//  6
  SMBR_AtRateOK,   	//  7
  SMBR_Temperature,	//  8
  SMBR_Voltage,    	//  9
  SMBR_Current,    	// 10
  SMBR_AvgCurrent, 	// 11
  SMBR_MaxError,   	// 12
  SMBR_RelSOC,     	// 13
  SMBR_AbsSOC,     	// 14
  SMBR_RemCap,     	// 15
  SMBR_FullChgCap, 	// 16
  SMBR_RunTTE,     	// 17
  SMBR_AvgTTE,     	// 18
  SMBR_AvgTTF,     	// 19
  SMBR_ChgCurrent, 	// 20
  SMBR_ChgVoltage, 	// 21
  SMBR_BattStatus, 	// 22
  SMBR_CycleCount, 	// 23
  SMBR_DesignCap,  	// 24
  SMBR_DesignVolt, 	// 25
  SMBR_SpecInfo,   	// 26
  SMBR_MfrDate,    	// 27
  SMBR_SerialNo,   	// 28
  SMBR_invalid,
  SMBR_invalid,
  SMBR_invalid,
  SMBR_MfrName,    	// 32
  SMBR_DeviceName, 	// 33
  SMBR_DeviceChem, 	// 34
  SMBR_MfrData,    	// 35
  SMBR_invalid,
  SMBR_invalid,
  SMBR_invalid,
  SMBR_invalid,
  SMBR_invalid,
  SMBR_invalid,
  SMBR_invalid,
  SMBR_invalid,
  SMBR_invalid,
  SMBR_invalid,
  SMBR_invalid,
  SMBR_Opt5,       	// 0x2F
  SMBR_invalid,
  SMBR_invalid,
  SMBR_invalid,
  SMBR_invalid,
  SMBR_invalid,
  SMBR_invalid,
  SMBR_invalid,
  SMBR_invalid,
  SMBR_invalid,
  SMBR_invalid,
  SMBR_invalid,
  SMBR_invalid,
  SMBR_Opt4,       	// 0x3C
  SMBR_Opt3,       	// 0x3D
  SMBR_Opt2,       	// 0x3E
  SMBR_Opt1       	// 0x3F
};



/* *************************************************************************
 *
 *   Individual handlers for SMBus WRITE-type commands
 *
 ************************************************************************* */


uint8_t SMBW_MfrAccess(void)	//  0
{
  uint8_t temp = TW_RxBuf[TW_RxBufIndex++];
  SMBvar_int[SMBV_MfrAccess] = temp | (TW_RxBuf[TW_RxBufIndex]<<8);
  SMBvariables[SMBV_BattStatus][lobyte] &= 0xF0;  //since this cmd is OK, clear Error bits per sbdat110, 4.3.2
  return 0;
}


uint8_t SMBW_RemCapAlm(void)	//  1
{
  uint8_t temp = TW_RxBuf[TW_RxBufIndex++];
  SMBvar_int[SMBV_RemCapAlm] = temp | (TW_RxBuf[TW_RxBufIndex]<<8);
  SMBvariables[SMBV_BattStatus][lobyte] &= 0xF0;  //since this cmd is OK, clear Error bits per sbdat110, 4.3.2
  return 0;
}


uint8_t SMBW_RemTimeAlm(void)	//  2
{
  uint8_t temp = TW_RxBuf[TW_RxBufIndex++];
  SMBvar_int[SMBV_RemTimeAlm] = temp | (TW_RxBuf[TW_RxBufIndex]<<8);
  SMBvariables[SMBV_BattStatus][lobyte] &= 0xF0;  //since this cmd is OK, clear Error bits per sbdat110, 4.3.2
  return 0;
}


uint8_t SMBW_BattMode(void)  	//  3
{
  uint8_t tempH = TW_RxBuf[TW_RxBufIndex+1];
  uint8_t tempL;

  tempL = SMBvariables[SMBV_BattMode][lobyte] & 0xF0;

  if(tempH & 0x1C)
    return SMBerr_AccessDenied;		//attempt to write to reserved bits!


  if(~(tempL & INTERNAL_CHARGE_CONTROLLER))
  {
    tempH &= ~CHARGE_CONTROLLER_ENABLED;  //feature not present, don't let it get turned on.
  }


  if(tempH & PRIMARY_BATTERY)
  {
    ;	//let the Host do what it wants with this flag; we ignore it.
  }

  if(tempH & ALARM_MODE)
    ; //SetAlarmMode;	//this DISABLES sending alarm msgs for 60 secs


  if(tempH & CHARGER_MODE)
  {
    ;	//Allow Host to enable us to send Master-mode Charger-Voltage/Current msgs to the Charger
  }


  if(tempH & CAPACITY_MODE)
  {
    ;	//Host must be allowed to control this bit (report in mAH or 10mWH)
  }


  SMBvar_int[SMBV_BattMode] = tempL | (tempH<<8);	//write the modified bits.

  return 0;
}



uint8_t SMBW_AtRate(void)     	//  4
{
  uint8_t temp = TW_RxBuf[TW_RxBufIndex++];
  SMBvar_int[SMBV_AtRate] = temp | (TW_RxBuf[TW_RxBufIndex++]<<8);
  SMBvariables[SMBV_BattStatus][lobyte] &= 0xF0;  //since this cmd is OK, clear Error bits per sbdat110, 4.3.2
  return 0;
}

uint8_t SMBW_Opt5(void)       	// 0x2F
{
  __disable_interrupt();
  MCUSR = 0x1F;                         //clear all reset sources before jumping to bootloader code.
//  __watchdog_reset;					//

  asm("jmp 0x9000");					// Byte adress

//  __watchdog_reset;						// reset watchdog
  return(1);							// to avoid compiler warning, should never be reached

//  return SMBerr_ReservedCommand; // <-- Compiler generates warning if return statement is missing.
}

uint8_t SMBW_Opt4(void)       	// 0x3C
{
/*  calibration_state_req = TW_RxBuf[TW_RxBufIndex++];
  calibration_state_req |= (uint16_t)TW_RxBuf[TW_RxBufIndex++] << 8; // Store request details.
  SetCalibRequest; // Set action flag so that main loop starts calibrating.
  return 0; // Return OK.
	*/
	  return SMBerr_ReservedCommand;

}

uint8_t SMBW_Opt3(void)       	// 0x3D
{
  return SMBerr_ReservedCommand;
}

uint8_t SMBW_Opt2(void)       	// 0x3E
{
  return SMBerr_ReservedCommand;
}

uint8_t SMBW_Opt1(void)       	// 0x3F
{
  return SMBerr_ReservedCommand;
}

uint8_t SMBW_Invalid(void)	//This should never execute, if error is caught early!
{
  return SMBerr_AccessDenied;
}



//Table of pointers to functions, indexed from the received SMBus Command byte.
ptr2funcUC_V SMB_WriteCmd[HIGHEST_SMB_CMD+1] =
{
  SMBW_MfrAccess,	//  0
  SMBW_RemCapAlm,	//  1
  SMBW_RemTimeAlm,	//  2
  SMBW_BattMode,  	//  3
  SMBW_AtRate,     	//  4
  SMBW_Invalid,
  SMBW_Invalid,
  SMBW_Invalid,
  SMBW_Invalid,
  SMBW_Invalid,
  SMBW_Invalid,
  SMBW_Invalid,
  SMBW_Invalid,
  SMBW_Invalid,
  SMBW_Invalid,
  SMBW_Invalid,		//0x0F

  SMBW_Invalid,
  SMBW_Invalid,
  SMBW_Invalid,
  SMBW_Invalid,
  SMBW_Invalid,
  SMBW_Invalid,
  SMBW_Invalid,
  SMBW_Invalid,
  SMBW_Invalid,
  SMBW_Invalid,
  SMBW_Invalid,
  SMBW_Invalid,
  SMBW_Invalid,
  SMBW_Invalid,
  SMBW_Invalid,
  SMBW_Invalid,		//0x1F

  SMBW_Invalid,
  SMBW_Invalid,
  SMBW_Invalid,
  SMBW_Invalid,
  SMBW_Invalid,
  SMBW_Invalid,
  SMBW_Invalid,
  SMBW_Invalid,
  SMBW_Invalid,
  SMBW_Invalid,
  SMBW_Invalid,
  SMBW_Invalid,
  SMBW_Invalid,
  SMBW_Invalid,
  SMBW_Invalid,
  SMBW_Opt5,       	// 0x2F

  SMBW_Invalid,
  SMBW_Invalid,
  SMBW_Invalid,
  SMBW_Invalid,
  SMBW_Invalid,
  SMBW_Invalid,
  SMBW_Invalid,
  SMBW_Invalid,
  SMBW_Invalid,
  SMBW_Invalid,
  SMBW_Invalid,
  SMBW_Invalid,
  SMBW_Opt4,       	// 0x3C
  SMBW_Opt3,       	// 0x3D
  SMBW_Opt2,       	// 0x3E
  SMBW_Opt1       	// 0x3F
};




/* *************************************************************************
 *
 *   Utilities for SMBus commmunications
 *
 ************************************************************************* */



__flash uint8_t crctable[16] =  {0,0x07,0x0E,0x90, 0x1c,0x1b,0x12,0x15, 0x38,0x3F,0x36,0x31, 0x24,0x23,0x2A,0x2D};
__flash uint8_t crctable2[16] = {0,0x70,0xE0,0x09, 0xC1,0xB1,0x21,0x51, 0x83,0xF3,0x63,0x13, 0x42,0x32,0xA2,0xD2};

uint8_t FastCRC(uint8_t LastCRC, uint8_t newbyte)
{
  uint8_t index;

  index = newbyte;
  index ^= LastCRC;
  index >>= 4;
  LastCRC &= 0x0F;
  LastCRC ^= crctable2[index];

  index = LastCRC;
  index ^= newbyte;
  index &= 0x0F;
  LastCRC &= 0xF0;
  LastCRC ^= crctable[index];

  return(LastCRC);
}


//! \todo  This can be modified to load defaults from EEPROM rather than fixed values to the SMB variables.
// This sets power-up defaults.
void InitSMBvariables(void)
{
  SMBvar_int[SMBV_MfrAccess] = 0x4060;				// Mega406 ap-note, revision 0 code
  SMBvar_int[SMBV_RemCapAlm] = (PACK_DESIGNCAPTYP / 10);	// per sbdat110, 4.4.1
  SMBvar_int[SMBV_RemTimeAlm] = 0x000A;			// per sbdat110, 4.4.1
  SMBvar_int[SMBV_BattMode  ] = 0x0000;	//
  SMBvar_int[SMBV_AtRate    ] = 0x0000;	//

/* For testing with no calcs
  SMBvar_int[SMBV_AtRateTTF ] = 0x0000;	//
  SMBvar_int[SMBV_AtRateTTE ] = 0x0000;	//
  SMBvar_int[SMBV_AtRateOK  ] = 0x0000;	//

  SMBvar_int[SMBV_Temperature] = 0x0000;	//
  SMBvar_int[SMBV_Voltage    ] = 0x0000;	//
  SMBvar_int[SMBV_Current    ] = 0x0000;	//
  SMBvar_int[SMBV_AvgCurrent ] = 0x0000;	//
  SMBvar_int[SMBV_MaxError   ] = 0x0000;	//
  SMBvar_int[SMBV_RelSOC     ] = 0x0000;	//
  SMBvar_int[SMBV_AbsSOC     ] = 0x0000;	//
  SMBvar_int[SMBV_RemCap     ] = 0x0000;	//

  SMBvar_int[SMBV_FullChgCap] = 0x0000;	//
  SMBvar_int[SMBV_RunTTE    ] = 0x0000;	//
  SMBvar_int[SMBV_AvgTTE    ] = 0x0000;	//
  SMBvar_int[SMBV_AvgTTF    ] = 0x0000;	//
  SMBvar_int[SMBV_ChgCurrent] = 0x0000;	//
  SMBvar_int[SMBV_ChgVoltage] = 0x0000;	//
*/
  SMBvar_int[SMBV_BattStatus] = 0x0080;	//per sbdat110, 4.4.1
  SMBvar_int[SMBV_CycleCount] = 0x0000;	//per sbdat110, 4.4.1

/* For testing with no calcs
  SMBvar_int[SMBV_DesignCap ] = 0x0000;	//
  SMBvar_int[SMBV_DesignVolt] = 0x0000;	//
*/

  SMBvar_int[SMBV_SpecInfo  ] = 0x0031;	// no scaling of I or V; we support PEC, and we're V1.1
  SMBvar_int[SMBV_MfrDate   ] = ((2005-1980)<<9)+(8<<5)+(31);	//! \todo Fill in current year, month, day. Values are octal
  SMBvar_int[SMBV_SerialNo  ] = 12345;	// arbitrary...

  //Note that for the Block-Read variables, we copy those into a RAM buffer only as needed.
  // These are MfrName, DeviceName, DeviceChem, and MfrData.

//SH  SetMaxTopAcc((long)6600*10727);   //! \todo for testing, initialized value before reaching fully charged, 6600mAh * 10727
//  RunningAcc = (long)1000*10727;    //for testing, to start at other point than 0, 1000 * 10727
}

