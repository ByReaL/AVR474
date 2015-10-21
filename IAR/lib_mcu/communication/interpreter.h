/* This file has been prepared for Doxygen automatic documentation generation.*/
/*! \file *********************************************************************
 *
 * \brief
 *       SMBus Protocol command interpreter.
 *
 * \par Application note:
 *      AVR438: Smart Battery Reference Design
 *
 * \par Documentation:
 *      For comprehensive code documentation, supported compilers, compiler
 *      settings and supported devices see readme.html
 *
 * \author
 *      Atmel Corporation: http://www.atmel.com \n
 *      Support email: avr@atmel.com \n
 *
 ******************************************************************************/

/*
Things to watch for:
  > Ttimeout = 25mS-35mS (clk held low)
  > Tbuf = buss free time after stop > 5uS.
  > Tpor = 500mS max = startup time allowed before device must respond to SMBus cmds
  > Tpdn = power-down time =  >=2.5secs of bus low  (3.1.4.2)

Ttimeout (Bus Timeout) is detected by reasserting a wdog timer
each time a transition occurs in the TWI state machine.

Tbuf is assured by requiring that the user exit the state machine
completely after sending or receiving a STOP condition, rather than
looping around internal to the state machine to send the next START.

Tpor is dependent on the list of startup tasks.  The Power-On Reset itself
is determined to have occurred via the interrupt.

Tpdn is determined by:


SMBus requires that the TWI module act both as a Slave device and, on occasion,
as a Master.  The management of the hardware for operating between these two
modes is handled in the TWI.C module.  Essentially, the TWI module is kept in
Slave mode by default.  When in IDLE state *AND* the bus is Idle, it checks for
the presence of a command in the Master Command buffer.  If present, it will
initiate the switch to Master Mode and commence the transmission.

Note that to know for certain that the bus is Idle, we must rely on TWSR.
Fortunately, TWSR does in fact accurately reflect the condition of the bus
regardless of whether the Mega406 is actually communicating on it or not.

*/


//As defined by SMBus spec, used for dynamically assigned addresses.
#define SMB_defaultaddr 0xC2

//Addresses
#define SMB_prototype0 0x90
#define SMB_prototype1 0x92
#define SMB_prototype2 0x94
#define SMB_prototype3 0x96

//Command Type flags
#define SMBinvalid 1
#define SMBmaster  2
#define SMBslave   4
#define SCRW 0x10	/*SMBus Command Read Word*/
#define SCWW 0x20	/*SMBus Command Write Word*/
#define SCRG 0x40	/*SMBus Command Read Block(Group)*/
#define SCWG 0x80   /*SMBus Command Write Block (Group)*/



//Information within the table is defined as follows:
//  [0] = Command-associated features & characteristics flags.
//  [1] =
//  [2] =
//  [3] =

__flash unsigned char SM_Cmd_Table[0x40][2] =
{
  { SMBslave|SCRW|SCWW, },	//Manufacturer Access, 0x00, r/w word;
  { SMBslave|SCRW|SCWW, },	//RemainingCapacityAlarm*, 0x01, r/w word; in mAh or 10mWh
  { SMBslave|SCRW|SCWW, },	//RemainingTimeAlarm*, 0x02, r/w word; in minutes
  { SMBslave|SCRW|SCWW, },	//BatteryMode, 0x03, r/w word; in bit flags
  { SMBslave|SCRW|SCWW, },	//AtRate 0x04 r/w mA or 10mW
  { SMBslave|SCRW,      },	//AtRateTimeToFull 0x05 r minutes
  { SMBslave|SCRW,      },	//AtRateTimeToEmpty* 0x06 r minutes
  { SMBslave|SCRW,      },	//AtRateOK* 0x07 r Boolean

  { SMBslave|SCRW,      },	//Temperature 0x08 r 0.1°K
  { SMBslave|SCRW,      },	//Voltage 0x09 r mV
  { SMBslave|SCRW,      },	//Current 0x0a r mA
  { SMBslave|SCRW,      },	//AverageCurrent 0x0b r mA
  { SMBslave|SCRW,      },	//MaxError 0x0c r percent
  { SMBslave|SCRW,      },	//RelativeStateOfCharge 0x0d r percent
  { SMBslave|SCRW,      },	//AbsoluteStateOfCharge 0x0e r percent
  { SMBslave|SCRW,      },	//RemainingCapacity 0x0f r mAh or 10mWh

  { SMBslave|SCRW,      },	//FullChargeCapacity 0x10 r mAh or 10mWh
  { SMBslave|SCRW,      },	//RunTimeToEmpty* 0x11 r minutes
  { SMBslave|SCRW,      },	//AverageTimeToEmpty* 0x12 r minutes
  { SMBslave|SCRW,      },	//AverageTimeToFull 0x13 r minutes
  { SMBslave|SMBmaster|SCRW|SCWW,},      //ChargingCurrent 0x14 r mA
  { SMBslave|SMBmaster|SCRW|SCWW,},      //ChargingVoltage 0x15 r mV
  { SMBslave|SCRW,      },	//BatteryStatus* 0x16 r bit flags
  { SMBslave|SCRW,      },	//CycleCount 0x17 r count

  { SMBslave|SCRW,      },	//DesignCapacity 0x18 r mAh or 10mWh
  { SMBslave|SCRW,      },	//DesignVoltage 0x19 r mV
  { SMBslave|SCRW,      },	//SpecificationInfo 0x1a r unsigned int
  { SMBslave|SCRW,      },	//ManufactureDate 0x1b r unsigned int
  { SMBslave|SCRW,      },	//SerialNumber 0x1c r number
  { SMBinvalid,         },	//reserved 0x1d
  { SMBinvalid,         },	//reserved 0x1e
  { SMBinvalid,         },	//reserved 0x1f

  { SMBslave|SCRG,      },	//ManufacturerName 0x20 r string
  { SMBslave|SCRG,      },	//DeviceName 0x21 r string
  { SMBslave|SCRG,      },	//DeviceChemistry 0x22 r string
  { SMBslave|SCRG,      },	//ManufacturerData 0x23 r data
  { SMBinvalid,         },	//reserved 0x24
  { SMBinvalid,         },	//reserved 0x25
  { SMBinvalid,         },	//reserved 0x26
  { SMBinvalid,         },	//reserved 0x27

  { SMBinvalid,         },	//reserved 0x28
  { SMBinvalid,         },	//reserved 0x29
  { SMBinvalid,         },	//reserved 0x2a
  { SMBinvalid,         },	//reserved 0x2b
  { SMBinvalid,         },	//reserved 0x2c
  { SMBinvalid,         },	//reserved 0x2d
  { SMBinvalid,         },	//reserved 0x2e
  { SMBslave|SCRW|SCWW, },      //OptionalMfgFunction5 0x2f r/w data

  { SMBinvalid,         },	//reserved 0x30
  { SMBinvalid,         },	//reserved 0x31
  { SMBinvalid,         },	//reserved 0x32
  { SMBinvalid,         },	//reserved 0x33
  { SMBinvalid,         },	//reserved 0x34
  { SMBinvalid,         },	//reserved 0x35
  { SMBinvalid,         },	//reserved 0x36
  { SMBinvalid,         },	//reserved 0x37

  { SMBinvalid,         },	//reserved 0x38
  { SMBinvalid,         },	//reserved 0x39
  { SMBinvalid,         },	//reserved 0x3a
  { SMBinvalid,         },	//reserved 0x3b
  { SMBslave|SCRW|SCWW, },	//OptionalMfgFunction4 0x3c r/w word
  { SMBslave|SCRW|SCWW, },	//OptionalMfgFunction3 0x3d r/w word
  { SMBslave|SCRW|SCWW, },	//OptionalMfgFunction2 0x3e r/w word
  { SMBslave|SCRW|SCWW, }	//OptionalMfgFunction1 0x3f r/w word
};



/*

Status Handler:

Current		TWSR						   					Next
State	    Status	Meaning			Actions		   			State
----------  ------	------------	----------------------	------------
Idle		0x60	saw SLA w/WR	set TWEA		  		Wait4Cmd

			0xA8	saw SLA w/RD	goto ReplyData		    n/a

Wait4Cmd	0x80	got CMD			store Cmd Byte,		
									set TWEA		  		Wait4RW
		
Wait4RW		0x80	got data byte	goto Wait4Data handler	    n/a

			0xA0	SLA incoming	set up data for Reply 	    Idle
									(based on Cmd rcvd)	

Wait4Data	0x80	got data		store Data byte,
									set TWEA		  		Wait4Data

			0xA0	got Stop		flag MsgDone.		    Idle
			
ReplyData	0xA8	must send first	  load TWDR, and set	  ReplyData
									  byte of Reply	  	  TWEA only if not last

			0xB8	got ACK; send 	Load TWDR, and set	  ReplyData
									out more data. 	      TWEA only if not last

			0xC0	got NAK 		if have more, flag Error;   Idle
									if no more, done.	    	Idle

			0xC8	got ACK after 	flag error, 		    	Idle
									sent Final Byte	  			set TWEA.



Normal sequence for Write Word without PEC:

CURRENT
STATE	   BUS ACTIVITY      TWSR    RESPONSE
-------    ----------------  ----    --------------------------------------------------
Idle       Receives SLA+W    0x60    Clear SMB_Master_Request_Delay. State=Wait4Cmd.
Wait4Cmd   Receives Cmd      0x80    Get Features & validate command. State=Wait4RW.
Wait4RW    Receives LowByte  0x80    Init RX buffer & store. State=Wait4Data.
Wait4Data  Receives HighByte 0x80    Store. Flag Foreground to process it.
< Foreground routine analyzes received command, then clears TWINT. Master issues STOP. >
Wait4Data  Receives STOP     0xA0    State=Idle.


Normal sequence for Write Word WITH PEC:

CURRENT
STATE	   BUS ACTIVITY      TWSR    RESPONSE
-------    ----------------  ----    --------------------------------------------------
Idle       Receives SLA+W    0x60    Clear SMB_Master_Request_Delay. State=Wait4Cmd.
Wait4Cmd   Receives Cmd      0x80    Get Features & validate command. State=Wait4RW.
Wait4RW    Receives LowByte  0x80    Init RX buffer & store. State=Wait4Data.
Wait4Data  Receives HighByte 0x80    Store.
Wait4Data  Receive PEC       0x80    Store. Flag Foreground to process it.
< Foreground routine analyzes received command, then clears TWINT. Master issues STOP. >
Wait4Data  Receive STOP      0xA0    State=Idle.




Normal sequence for Read Word without PEC:

CURRENT
STATE	   BUS ACTIVITY      TWSR    RESPONSE
-------    ---------------   ----    --------------------------------------------------
Idle       Received SLA+W    0x60    Clear SMB_Master_Request_Delay. State=Wait4Cmd.
Wait4Cmd   Received Cmd      0x80    Get Features & validate command. State=Wait4RW.
Wait4RW    Received Start    0xA0    Flag foreground to set up Reply data. State=ReplyData.
< Foreground routine analyzes received command, then clears TWINT. Master continues. >
ReplyData  Received SLA+R    0xA8    Load TWDR & set TWEA.
ReplyData  Received ACK      0xB8    Load TWDR but do not set TWEA (expect NAK back).
ReplyData  Received NAK      0xC0    State=Idle.


Normal sequence for Read Word WITH PEC:

CURRENT
STATE	   BUS ACTIVITY      TWSR    RESPONSE
-------    ---------------   ----    --------------------------------------------------
Idle       Received SLA+W    0x60    Clear SMB_Master_Request_Delay. State=Wait4Cmd.
Wait4Cmd   Received Cmd      0x80    Get Features & validate command. State=Wait4RW.
Wait4RW    Received Start    0xA0    Flag foreground to set up Reply data. State=ReplyData.
< Foreground routine analyzes received command, then clears TWINT. Master continues. >
ReplyData  Received SLA+R    0xA8    Load TWDR & set TWEA.
ReplyData  Received ACK      0xB8    Load TWDR & set TWEA.
ReplyData  Received ACK      0xB8    Load TWDR but do not set TWEA (expect NAK back).
ReplyData  Received NAK      0xC0    State=Idle.



*/
