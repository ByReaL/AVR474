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
* - Description       : Header file for SMBSlave.c. Contains some
*                       configuration parameters.
*
* $Revision: 5627 $
* $Date: 2009-05-15 14:49:21 +0800 (Fri, 15 May 2009) $
*****************************************************************************/


#ifndef __SMB_SLAVE_H__
#define __SMB_SLAVE_H__


//Prototypes...
void InitSMBus(void);
//void SMB_Master(void);
//void MasterInsertMsg(unsigned char addr, unsigned char cmd, unsigned int data);
void SMB_CmdInterpreter(void);
void SMB_RestoreBus(void);	//this is used by generic timer to recover BusFault
//void InitSMBvariables(void);
//void Check50uS(void);



//This is used to check for an out-of-bounds SMBus command.
#define HIGHEST_SMB_CMD 0x3F


//Two-Wire-Interface TWSR (STATUS) values
//Note that since our Prescale value is 0, we don't need to MASK the Status byte.

//  Globally applicable TWI status codes:
#define TWS_MASK        0xF8	/* Two-Wire Status Mask */
#define TWS_NSTAT       0xF8	/* No Status Available now */

//  MASTER-related Status codes:
#define TWS_START       0x08
#define TWS_RESTART     0x10
#define TWS_WRITE_ACK   0x18	/* sent a SLA+W, got ACK */
#define TWS_WRITE_NAK   0x20	/* sent SLA+W, got NAK */
#define TWS_TXDATA_ACK  0x28	/* Data tx'd & was ACK'd */
#define TWS_TXDATA_NAK  0x30	/* Data tx'd & was NAK'd */
#define TWS_LOST_ARB    0x38	/* lost bus arbitration */

#define TWS_READ_ACK    0x40	/* got ACK from a SLA+R request */
#define TWS_READ_NAK    0x48	/* got NAK from a SLA+R request */
#define TWS_RXDATA_ACK  0x50	/* We rcvd data and ACKd back */
#define TWS_RXDATA_NACK 0x58	/* We rcvd data and we NACKd back */


//  SLAVE-related Status codes:
#define TWS_SLA_W       0x60	/* Got SLA + Write */
#define TWS_SLA_R       0xA8	/* Got SLA + Read  */
#define TWS_RDATA       0x80	/* Got a data byte */
#define TWS_RCMD        0x80	/* Got a command byte */
#define TWS_RSTOP       0xA0	/* Got a Stop */
#define TWS_REPEAT      0xA0	/* Got a Repeated-Start */
#define TWS_RACK        0xB8	/* Send a data byte and got an ACK back */
#define TWS_RNAK        0xC0	/* Sent a data byte and got a NAK back */
#define TWS_FINAL       0xC8	/* Sent the final byte, got ACK back */
#define TWS_BERR        0x00	/* Saw a Bus Error */


// Two-Wire CONTROL values

#define TWC_GO          0x85	/* clr TWINT; assert ENA & IntEna */
#define TWC_READ_NoACK  0x85	/* read a byte, but don't ACK when done */
#define TWC_START       0xA5	/* send START bit, assert ENA & IntEna */
#define TWC_STOP        0x94	/* leave INT *DISabled* when done */
#define TWC_RESTART     0xB5    /* send STOP, then START again; INT ena */


/* ************************************************************************* */
#define SMBvariables SV.SMBvar
#define SMBvar_int   SV.SMBint

#ifdef MODULE_SMBUS
  union { volatile unsigned char SMBvar[29][2]; volatile unsigned int SMBint[29]; } SV;

  //Note: all of these strings must fit inside TW_TxBuf with room for Addr+Cmd+PEC
  //Also, at the front of each string is a "string constant" that indicates the
  //length of the rest of the string; this constant is in OCTAL!!
  __flash char str_MfrName[]    = "\014Atmel Norway"; //__flash char
  __flash char str_DeviceName[] = "\005SB100";
  __flash char str_DeviceChem[] = "\004LION";
  __flash char str_MfrData[]    = "\013MfrDataArea";

#else

  extern union { volatile unsigned char SMBvar[29][2]; volatile unsigned int SMBint[29]; } SV;

#endif



#define HOST_ADDR    0x10  // According to smbus specification Appendix C
#define CHARGER_ADDR 0x12
#define BATTERY_ADDR 0x16

//SMBus Variable + Command Name offset list
#define SMBV_MfrAccess  0
#define SMBV_RemCapAlm  1
#define SMBV_RemTimeAlm 2
#define SMBV_BattMode   3
#define SMBV_AtRate     4
#define SMBV_AtRateTTF  5
#define SMBV_AtRateTTE  6
#define SMBV_AtRateOK   7

#define SMBV_Temperature 8
#define SMBV_Voltage     9
#define SMBV_Current    10
#define SMBV_AvgCurrent 11
#define SMBV_MaxError   12
#define SMBV_RelSOC     13
#define SMBV_AbsSOC     14
#define SMBV_RemCap     15

#define SMBV_FullChgCap 16
#define SMBV_RunTTE     17
#define SMBV_AvgTTE     18
#define SMBV_AvgTTF     19
#define SMBV_ChgCurrent 20
#define SMBV_ChgVoltage 21
#define SMBV_BattStatus 22
#define SMBV_CycleCount 23

#define SMBV_DesignCap  24
#define SMBV_DesignVolt 25
#define SMBV_SpecInfo   26
#define SMBV_MfrDate    27
#define SMBV_SerialNo   28

#define SMBV_MfrName    32
#define SMBV_DeviceName 33
#define SMBV_DeviceChem 34
#define SMBV_MfrData    35
#define SMBV_Opt5     0x2F
#define SMBV_Opt4     0x3C
#define SMBV_Opt3     0x3D
#define SMBV_Opt2     0x3E
#define SMBV_Opt1     0x3F



//Offsets within each of the 29 SMBvariables elements
#define lobyte 0
#define hibyte 1




//SMBV_BatteryMode bit definitions

//  HiByte
#define CHARGE_CONTROLLER_ENABLED	0x01
#define PRIMARY_BATTERY			0x02
#define ALARM_MODE			0x20
#define CHARGER_MODE			0x40
#define CAPACITY_MODE			0x80

//  LoByte
#define INTERNAL_CHARGE_CONTROLLER	0x01
#define PRIMARY_BATTERY_SUPPORT		0x02
#define CONDITION_FLAG			0x80



//SMBV_BatteryStatus bit definitions.

// HiByte
/* * * * * * Alarm Bits * * * * */
#define OVER_CHARGED_ALARM		0x80
#define TERMINATE_CHARGE_ALARM		0x40
#define OVER_TEMP_ALARM			0x10
#define TERMINATE_DISCHARGE_ALARM	0x08
#define REMAINING_CAPACITY_ALARM	0x02
#define REMAINING_TIME_ALARM		0x01

// LoByte
/* * * * * * Status Bits * * * * */
#define INITIALIZED		0x80
#define DISCHARGING		0x40
#define FULLY_CHARGED		0x20
#define FULLY_DISCHARGED	0x10

/* * * * * * Error Codes * * * * */
#define SMBerr_OK               0x00	// r/w The Smart Battery processed the function code without detecting any errors.
#define SMBerr_Busy             0x01	// r/w The Smart Battery is unable to process the function code at this time.
#define SMBerr_ReservedCommand  0x02	// r/w The Smart Battery detected an attempt to read or write to a function code
					//     reserved by this version of the specification. The Smart Battery detected
					//     an attempt to access an unsupported optional manufacturer function code.
#define SMBerr_UnsuptdCommand   0x03	// r/w The Smart Battery does not support this function code which is
					//     defined in this version of the specification.
#define SMBerr_AccessDenied     0x04	//  w  The Smart Battery detected an attempt to write to a read-only function code.
#define SMBerr_OverUnderflow    0x05	// r/w The Smart Battery detected a data overflow or under flow.
#define SMBerr_BadSize          0x06 	//  w  The Smart Battery detected an attempt to write to a function code with
					//     an incorrect size data block.
#define SMBerr_UnknownError     0x07 	// r/w The Smart Battery detected an unidentifiable error.

#endif
