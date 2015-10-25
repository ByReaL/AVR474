/* This file has been prepared for Doxygen automatic documentation generation.*/
/*! \file *********************************************************************
 *
 * \brief
 *      Voltage regulator monitoring interface routines.
 *
 *      If the voltage regulator detects that the voltage VFET drops below the
 *      Regulator Short-circuit Level (see datasheet), the Voltage Regulator
 *      enters the Battery Pack Short mode. In this mode, VFET is disconnected
 *      from VREG to avoid a quick drop in the voltage regulator output. When
 *      the voltage regulator enters this mode, the chip will be completely
 *      powered by the external reservoir capacitor (CREG). This allows the
 *      chip to operate a certain time without entering BOD reset, even if the
 *      VFET voltage is too low for the voltage regulator to operate. The time
 *      is given by the difference between operating voltage minus BOD level
 *      multiplied by capacitance and divided by average load current. For C =
 *      2.2uF and 2.9V BOD level and Iaverage = 100uA, time = deltaV *
 *      Capacitance / Iaverage = 0.4V * 2.2uF / 100uA = 8.8ms. To reduce the
 *      current consumption and thus prolong the operating time before the BOD
 *      (Brown out Detection) is triggered. the device should disable modules
 *      using very much power and enter power save sleep as soon as possible.\n
 *
 *      If an EEPROM write is ongoing the reservoir capacitor needs to be
 *      inconviently large to sustain operation until the write finishes, so
 *      we doesn't account for EEPROM writes.
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

/******************************************************************************
 * Includes.
 ******************************************************************************/
#include "vreg_mon.h"
#include "watchdog.h"
#include "pwr_mgmnt.h"
#include <avr/interrupt.h>
#include <avr/wdt.h>
/******************************************************************************
 * Variables.
 ******************************************************************************/
uint8_t vregmon_count = 0;
uint8_t SaveVregmonVar[PRR0_SAVE+1];


/******************************************************************************
 * Functions.
 ******************************************************************************/

/******************************************************************************/
/*! \brief VREGMON interrupt routine.
 *
 *  This routine checks that the condition still exists and if so disables all
 *  interrupts. It also disables the vadc to save power. The watchdog is then
 *  configured to 16 ms timeout and interrupt mode. Global interrupts are then
 *  enabled before entering power save sleep. The watchdog then wakes up the
 *  device and the condition should have disappeared, either because the short
 *  circuit protection have triggered or the reservoir cap have been
 *  sufficiently charged. Global interrupts are then disabled and the various
 *  modules/interrupts restored, and finally the #vregmon_count incremented.
 */
ISR(VREGMON_vect)
{
	/* Check that VREGMON condition still exists as interrupt can't wake the
	   device from power save sleep and interrupt flag will thus remain and
	   trigger an interrupt even if the condition has disappeared before the
	   device wakes up. */
	if ( VREGMON_ConditionExists() ) {
		// Save which external interrupts are enabled and disable all.
		SaveVregmonVar[EXTINT_SAVE] = EIMSK;
		EIMSK = 0x00;
		// Save which battery protection interrupts are enabled and disable all.
		SaveVregmonVar[BATPROT_SAVE] = BPIMSK;
		BPIMSK = 0x00;
		// Save which Timer0 interrupts are enabled and disable all.
		SaveVregmonVar[TIMER0_SAVE] = TIMSK0;
		TIMSK0 = 0x00;
		// Save which Timer1 interrupts are enabled and disable all.
		SaveVregmonVar[TIMER1_SAVE] = TIMSK1;
		TIMSK1 = 0x00;
		// Save SPI interrupt (if enabled) and disable it.
		SaveVregmonVar[SPI_SAVE] = SPCR;
		SPCR = SaveVregmonVar[SPI_SAVE] & ~(1<<SPIE);

		// Disable VADC if running. (150uA power consumption.)
		SaveVregmonVar[VADC_SAVE] = VADCSR;
		VADCSR = (1<<VADCCIF);  // Clear interrupt flag and rest if VADCSR.

		// Disable CCADC interrupts, but let it continue to run.
		SaveVregmonVar[CCADC_SAVE] = CADCSRB & ( (1<<CADACIE) | (1<<CADRCIE) | (1<<CADICIE) );
		CADCSRB = 0x00;

		// Disable EEPROM interrupt.
		SaveVregmonVar[EEPROM_SAVE] = EECR & (1<<EERIE);
		EECR = 0x00;

		// Save WDT configuration.
		SaveVregmonVar[WDT_SAVE] = WDTCSR;
		wdt_reset();
		WDT_SetTimeOut( WDTO_16ms );
		WDT_EnableWakeUpInterrupt();

		/* Warning: The following code enables *nested* interrupts. It waits
		   for a WDT timeout to wake up the device, and then checks that the
		   vregmon condition has really disappeared. If not it just goes back
		   to sleep and waits for the watchdog to reset the device. */
		__enable_interrupt();
		while ( VREGMON_ConditionExists() ) {
			SLEEP_ENTER_POWER_SAVE();
		};
		__disable_interrupt();
		/* Nested interrupts disabled.*/

		// Set WDT to original settings.
		WDT_SetTimeOut( (WDT_timeout_t)(SaveVregmonVar[WDT_SAVE]  & WDT0_mask) );
		WDTCSR = SaveVregmonVar[WDT_SAVE];  // Possibly re-enable wdt interrupt.

		// Restore PRR, interrupts and modules.
		EIMSK = SaveVregmonVar[EXTINT_SAVE];
		BPIMSK = SaveVregmonVar[BATPROT_SAVE];
		TIMSK0 = SaveVregmonVar[TIMER0_SAVE];
		TIMSK1 = SaveVregmonVar[TIMER1_SAVE];
		SPCR = SaveVregmonVar[SPI_SAVE];
		VADCSR = SaveVregmonVar[VADC_SAVE];
		CADCSRB = SaveVregmonVar[CCADC_SAVE];
		EECR = SaveVregmonVar[EEPROM_SAVE];

		vregmon_count++; // Flag that a VREGMON condition has happened.
	} else {
		/* VREGMON condition has been cleared (ROCS bit not set), so the
		   interrupt's only purpose is to clear the flag. */
	}
}
	


/******************************************************************************/
/*! \brief Number of times VREGMON has been triggered.
 *
 *  This function returns the (module internal) #vregmon_count variable
 *  which contain the number of times a vreg condition has happened since the
 *  last readout. The variable is cleared in the process. The vregmon interrupt
 *  increments the variable each time it is run and disables modules.
 *
 *  \return 0 if a vregmon condition has not been triggered, or the number of
 *          times a vregmon condition has been triggered.
 */
uint8_t VREGMON_ConditionTriggered(void)
{
	bool temp_var = vregmon_count;
	vregmon_count = 0;
	return temp_var;
}

