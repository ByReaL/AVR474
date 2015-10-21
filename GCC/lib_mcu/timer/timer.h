/* This file has been prepared for Doxygen automatic documentation generation.*/
/*! \file *********************************************************************
 *
 * \brief
 *      Headerfile for timer.c
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

#ifndef _TIMER_H_
#define _TIMER_H_

void T1init(void);
void SetLEDbrightness(unsigned char value);
void SetLEDs(unsigned char flags);
void SetGenericTimer(unsigned char index, unsigned int delay);
unsigned char GetGenericTimer(unsigned char index);
uint8_t ShowLedBusy(void);

#define ButtonIsPushed()     ((PINA & (1<<PORTA3))!= (1<<PORTA3))
#define ButtonIsNotPushed()  (PINA & (1<<PORTA3))
#define LED_STATUS_TIMEOUT   1024
#define LED_ON_TIMEOUT       2000


#ifdef MODULE_TIMER

  unsigned char LEDs = 0;		//bit 0 = LED1, bit 4 = LED5
  unsigned char ErrFlags = 5;		//Error flags
  unsigned int LedTimeout = 0;
  unsigned char generictimer[8] = {0};

#else
  extern unsigned char LEDs;
  extern unsigned char generictimer[8];

#endif



//List of assignments for generic timer channels
#define SMBfaultTimer 0
#define OneQtrSecond  1
#define genericTimer2 2
#define genericTimer3 3
#define genericTimer4 4
#define genericTimer5 5
#define genericTimer6 6
#define genericTimer7 7



#endif  // _TIMER_H_
