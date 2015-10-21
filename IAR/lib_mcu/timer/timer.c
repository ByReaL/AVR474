/* This file has been prepared for Doxygen automatic documentation generation.*/
/*! \file *********************************************************************
 *
 * \brief
 *      Timer routines.
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


#include "common.h"
#include "smbus.h"

#define MODULE_TIMER
#include "timer.h"



/* *****************************************************
 *
 * Timer Usage:
 *
 * T0: serves several functions...
 *     1. basic tick rate for generic timers
 *     2. T0CMPA is optional LED PWM function
 *     3. T0CMPB is Charge FET PWM
 *   The clock src for T0 is main clk, 1MHz.
 *   Therefore, base period is 1MHz/256 = 3.9KHz = 256uS
 *   if prescaler = 1.
 *   For normal operation, we need both the LEDs on CMPA
 *   and the overflow as a timer tick. We will therefore
 *   use prescale = 8. This yields a 2.048mS tick, and
 *   a 100Hz update rate across all 5 LEDs.  The PWM for
 *   the FETs is available in this design, although it is
 *   not used.
 *
 * T1: not used in this design.
 *
 *
 * *****************************************************/



void T1init(void)
{
	// Initialize LED pins
	PORTB |= 0x1F;
	DDRB |= 0x1F;
	SetLEDs(50);
	
	//Initialize button input pullup and pin change interrupt
	DDRA &= ~(1<<PORTA3);
	PORTA |= (1<<PORTA3);
	PCMSK0 = 0x08; //Enable Pin change interrupt on pin 3
	PCICR = 0x01; //Enable Pin change 0 interrupt
	
	// Configure the timers used for the LEDs
	TCCR1A = 0;
	TCCR1B = 2;		//prescale = 8
	OCR1A = 10;		//use 75% brightness level so we can see the waveform for testing
	OCR1B = 240;		//available for PWM use by FETs
	
	TIFR1 = 0x0F;		//force all timer interrupt flags to be cleared.
	TIMSK1 = (1<<TOIE1);// (1<<OCIE1A)|(1<<OCIE1B);		//enable interrupts as needed.

}

uint8_t ShowLedBusy(void)
{
	if (LedTimeout == 0){
		return false;
	}else{
		return true;
	}
}


void SetLEDbrightness(unsigned char value)
{
  //Don't let the LED brightness go higher than 250, just so that
  // we don't collide with the overflow interrupt.
  if(value <= 250)
    OCR1A = value;
  else
    OCR1A = 250;
}

void SetLEDs(unsigned char LEDflags)
{
  LEDs = 0;
  if(LEDflags > 10)	LEDs |= 0x01;
  if(LEDflags > 30)	LEDs |= 0x02;
  if(LEDflags > 50)	LEDs |= 0x04;
  if(LEDflags > 70)	LEDs |= 0x08;
  if(LEDflags > 90)	LEDs |= 0x10;
}


/* Pin change interrupt on button input.
 *
 * This interrupt will trigger if there is a pin change on the button. This will
 * enable the timers to show status codes on the LEDs.
 */
#pragma vector = PCINT0_vect
__interrupt void PCINT0_ISR(void)
{
	if(ButtonIsPushed()){
		PORTB |= 0x1F;
		LedTimeout = 1;
		TIMSK1 &= ~((1<<OCIE1A) | (1<<OCIE1B));//Disable both interrupts
		TIFR1 |= (1<<OCIE1A) | (1<<OCIE1B); //Clear flags
		TIMSK1 |= (1<<OCIE1B);//Enable CompB interrupt
		PCICR &= ~(0x01); // Disable pin change interrupt
	}
}


/* This interrupt reduce the timeout until it is zero, and then turn of the LEDs.
 *
 * This interrupt will re-enable the pinchange interrupt for the button and disable
 * both compare interrupts.
 */
#pragma vector = TIMER1_COMPA_vect
__interrupt void T1CMPA_ISR(void)
{
	//PINB |= 0x1F;

	if(--LedTimeout == 0){
		PORTB |= 0x1F;
		TIMSK1 &= ~((1<<OCIE1A) | (1<<OCIE1B)); //Disable all comp interrupts
		PCICR |= 0x01; //Enable pin change interrupts again
	}
}


/* This interrupt check the hold time of the button pushed.
 *
 * If the button is pushed for a time longer than LED_STATUS_TIMEOUT, the
 * error code is shown instead of the SoC.
 */
#pragma vector = TIMER1_COMPB_vect
__interrupt void T1_COMPB_ISR(void)
{
	if(ButtonIsPushed() && (LedTimeout <= LED_STATUS_TIMEOUT)){
		LedTimeout++;
	}else{
		if(LedTimeout >= LED_STATUS_TIMEOUT){
			PORTB = ~(ErrFlags & 0x1F);
		}else{
			PORTB = ~(LEDs & 0x1F);
		}
		LedTimeout = LED_ON_TIMEOUT;
		TIMSK1 &= ~((1<<OCIE1A) | (1<<OCIE1B));//Disable this interrupt
		TIFR1 |= (1<<OCIE1A) | (1<<OCIE1B); //Clear flags
		TIMSK1 |= (1<<OCIE1A);//Enable CompA interrupt
	}	
}




//NOTE:  all of these fire from within the ISR, so keep them short!!

void generictimer0expired(void)
{
  SMB_RestoreBus();
}

void generictimer1expired(void)
{
//open for your use
}

void generictimer2expired(void)
{
//open for your use
}

void generictimer3expired(void)
{
//open for your use
}

void generictimer4expired(void)
{
//open for your use
}

void generictimer5expired(void)
{
//open for your use
}

void generictimer6expired(void)
{
//open for your use
}

void generictimer7expired(void)
{
//open for your use
}

//Delay is given in 1mS intervals, but timed in 2mS intervals.
void SetGenericTimer(unsigned char index, unsigned int delay)
{
  ++delay;
  delay >>= 1;	//convert from 1mS interval to 2mS resolution
  generictimer[index] = (unsigned char) delay;
}

unsigned char GetGenericTimer(unsigned char index)
{
  return generictimer[index];
}


typedef void (*ptr2funcV_V)(void);

//Table of pointers to functions, indexed from the received SMBus Command byte.
ptr2funcV_V GenericExpire[8] =
{
  generictimer0expired,
  generictimer1expired,
  generictimer2expired,
  generictimer3expired,
  generictimer4expired,
  generictimer5expired,
  generictimer6expired,
  generictimer7expired
};



//This interrupt functions as the main timer tick for the code.  Each time, a list
// of generic timers is decremented.  Whenever one reaches zero, its expiration function
// is called.  As the prescaler for T1 is initialized to 8, this fires every 2.048mS.
#pragma vector = TIMER1_OVF_vect
__interrupt void T1OVF_ISR(void)
{
  unsigned int temp;
  unsigned char i;

  for(i=0; i<8; i++)
  {
    temp = generictimer[i];
    if(0 != temp)
    {
      temp--;
      generictimer[i] = temp;
      if(temp == 0)	//when it hits 0, flag it!
        (GenericExpire[i])();
    }
  }

}
