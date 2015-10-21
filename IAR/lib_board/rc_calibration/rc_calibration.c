/* This file has been prepared for Doxygen automatic documentation generation.*/
/*! \file *********************************************************************
 *
 * \brief
 *      Functions for calibrating and estimating clock periods of the RC
 *		  Oscillators.
 *
 * \par Application note:
 *      AVR474: SB202 Firmware.
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

/******************************************************************************
 Included files
******************************************************************************/
#include "common.h"
#include "rc_calibration.h"
#include "ATmega32HVB_signature.h"

/******************************************************************************
 Prototypes for private functions
******************************************************************************/
void RCCAL_StopTimer0();
void RCCAL_StartOSIsampleClockPeriod(uint8_t clock_source);

/******************************************************************************
 File scope variables
******************************************************************************/
RCCAL_CalibState_t RCCAL_CalibState;
RCCAL_IC_State_t RCCAL_IC_State;

int16_t LastICResult;


/******************************************************************************/
/*! \brief Calculate Slow RC period time.
 *
 *	Calculate Slow RC period time given a temperature. The formula for
 *	calculation of Slow RC period is used with the signature data (loaded from
 *	the signature row) to calculate the actual period as described in the
 *	data sheet and application note. The result is in ms * 1024. It can thus be
 *	seen as ms in 16 bit fixed point format, with 10 fractional bits, i.e.
 *	0x XXXX XX.xx xxxx xxxx.
 *
 *  \param	temperature Temperature in Kelvin
 *
 *	\return	Slow RC period in ms*1024
 */
uint16_t RCCAL_CalculateSlowRCperiod(int16_t temperature)
{
	uint16_t slow_RC_period;
	int16_t  slow_RC_temperature_coeff;
	uint16_t calib_temperature;

	uint8_t interrupt_status = __save_interrupt();  // Disable interrupts.
	__disable_interrupt();
	
	//Read necessary data from the signature row. The delay cycles are to avoid
	//accessing the SPMCSR register within 6 cycles of previous write, since it
	//is locked for further writing during these cycles.
	slow_RC_period = READ_SIGNATUREWORD(SIG_SLOW_RC_L);

	slow_RC_temperature_coeff = READ_SIGNATUREWORD(SIG_SLOW_RC_PRED_L);

	calib_temperature = READ_SIGNATUREBYTE(SIG_TEMPERATURE_HOT) + ZERO_KELVIN;
	
	__restore_interrupt(interrupt_status);
	
	return (uint16_t)(slow_RC_period - ((temperature - calib_temperature)*slow_RC_temperature_coeff)/64);
}


/******************************************************************************/
/*! \brief Calculate Ultra-low power RC (Ulp RC) period time.
 *
 *	Calculate Ulp RC period time given a temperature. The formula for
 *	calculation of Ulp RC period is used with the signature data (loaded from
 *	the signature row) to calculate the actual period as described in the
 *	data sheet and application note. The result is in ms * 1024. It can thus be
 *	seen as ms in 16 bit fixed point format, with 10 fractional bits, i.e.
 *	0x XXXX XX.xx xxxx xxxx.
 *
 *  \param	temperature Temperature in Kelvin
 *
 *	\return	Ulp RC period in ms*1024
 */
uint16_t RCCAL_CalculateUlpRCperiod(int16_t temperature)
{
        uint16_t ulp_RC_period;
	int16_t  ulp_RC_temperature_coeff;
	uint16_t calib_temperature;

	uint8_t interrupt_status = __save_interrupt();  // Disable interrupts.
	__disable_interrupt();
	
	//Read necessary data from the signature row. The delay cycles are to avoid
	//accessing the SPMCSR register within 6 cycles of previous write, since it
	//is locked for further writing during these cycles.
	ulp_RC_period = READ_SIGNATUREWORD(SIG_ULP_RC_L);

	ulp_RC_temperature_coeff = READ_SIGNATUREWORD(SIG_ULP_RC_PRED_L);

	calib_temperature = READ_SIGNATUREBYTE(SIG_TEMPERATURE_HOT) + ZERO_KELVIN;
	
	__restore_interrupt(interrupt_status);
	
	return (uint16_t)(ulp_RC_period - ((temperature - calib_temperature)*ulp_RC_temperature_coeff)/64);
}

/******************************************************************************/
/*! \brief Start the Fast RC calibration
 *
 */
void RCCAL_StartCalibrateFastRC()
{
	RCCAL_CalibState = RCCAL_CALIB_INIT;
}


/******************************************************************************/
/*! \brief Calibrate the Fast RC against the Slow RC oscillator runtime.
 *
 *	Since the value we're searching for is probably close to the current we do
 *	a linear search. It searches the segment where FOSCCAL default
 *	is and the one below. When searching downwards we use the FOSC SEGMENT
 *	signature byte to avoid a jump in frequency when reaching the lower segment
 *	and thus giving a longer continous search range. FOSC segment is the first
 *	value in the previous segment that gives a lower frequency than the lowest
 *	value in the FOSCCAL default segment. If we reach the bottom of the lower
 *	segment or the top of the default segment we select the default FOSCCAL value
 *	and return failed as this range should provide a more than sufficient range
 *	for adjusting FOSCCAL over all temperatures, so something is wrong.
 *
 *	Note that no other boundary check is done, but even if the FOSCCAL value is
 *	outside the search range the frequency should be correctly calibrated,
 *	although jumps in frequency results will be encountered. If the FOSCCAL
 *	value is between the lowest value in FOSCCAL default segment and the FOSC
 *	SEGMENT, a value in that range might be selected.
 *
 *  \note  Do not call with different cal_RC_period during the "working" part of
 *         the function. If the slow RC period change during calibration, re-run
 *         RCCAL_StartCalibrateFastRC() to re-initialize some variables.
 *	
 *	\param	cal_RC_period   Slow RC period in ms * 1024
 */
void RCCAL_CalibrateFastRCruntime(uint16_t cal_RC_period)
{
	static uint8_t fosccal_segment;
	static uint8_t fosccal_default;
	static int16_t error;
	static int16_t last_error;
	static uint8_t fosccal_saved;
	uint8_t interrupt_status;
	
	switch(RCCAL_CalibState) {
		
	case RCCAL_CALIB_INIT:
		//Read necessary data from the signature row. The delay cycles are to avoid
		//accessing the SPMCSR register within 6 cycles of previous write, since it
		//is locked for further writing during these cycles.
	   	
	    	interrupt_status = __save_interrupt();  // Disable interrupts.
		__disable_interrupt();
		
		fosccal_default = READ_SIGNATUREBYTE(SIG_FOSCCAL_DEFAULT);

		fosccal_segment = READ_SIGNATUREBYTE(SIG_FOSC_SEGMENT);
		
		__restore_interrupt(interrupt_status);
		
		RCCAL_StartOSIsampleClockPeriod(SLOW_RC);
		RCCAL_CalibState = RCCAL_CALIB_WORKING;
		
		last_error = 32767; // last_error is invalid after a temperature change
		break;
		
	case RCCAL_CALIB_WORKING:
		{
			if(RCCAL_IC_FINISHED == RCCAL_IC_State) {
				uint8_t bottom_limit = fosccal_segment & 0xE0;
				uint8_t top_limit =  (fosccal_default & 0xE0) + 0x1F;
				
				uint16_t target_period = cal_RC_period * FAST_RC_TARGET_MHZ;	// Only whole MHz accounted for.
				
				error = LastICResult - target_period;
				
				// If absolute of last error is lower than absolute of current error, use the fosccal value from last test
				// (this happens when going from for example -4 error to +5 error)
				if(last_error < 0 && error > 0) {
					if(-last_error < error) {
						FOSCCAL = fosccal_saved;
						RCCAL_CalibState = RCCAL_CALIB_FINISHED;
						break;
					}
				}
				if(last_error > 0 && error < 0) {
					if(last_error < -error) {
						FOSCCAL = fosccal_saved;
						RCCAL_CalibState = RCCAL_CALIB_FINISHED;
						break;
					}
				}
				
				if(error > 0) {
					// Search downwards.
					uint8_t check_value = bottom_limit + 0x1F;
					last_error = error;
					fosccal_saved = FOSCCAL;
					if ( FOSCCAL-- == check_value ) {	
						FOSCCAL = fosccal_segment;	// Use fosccal segment value to avoid a step in frequency when going to lower segment.
					} else if (FOSCCAL < bottom_limit) {
						FOSCCAL = fosccal_default;	// The range have been searched without success, so we select default fosccal value.
						RCCAL_CalibState = RCCAL_CALIB_FAILURE;
					}
					RCCAL_StartOSIsampleClockPeriod(SLOW_RC);
					
				} else if (error < 0) {
					// Search upwards.
					uint8_t check_value = fosccal_segment + 0x01;
					last_error = error;
					fosccal_saved = FOSCCAL;
					if ( FOSCCAL++ == check_value ) {	
						FOSCCAL = fosccal_default & 0xE0; // Use fosccal segment value to avoid a step in frequency when going to upper segment.
					} else if ( FOSCCAL > top_limit ) {
						FOSCCAL = fosccal_default;	// The range have been searched without success, so we select default fosccal value.
						RCCAL_CalibState = RCCAL_CALIB_FAILURE;
					}
					RCCAL_StartOSIsampleClockPeriod(SLOW_RC);
					
				} else {
					// error is equal to zero, so stop at this value
					RCCAL_CalibState = RCCAL_CALIB_FINISHED;
					break;
				}
			}
		}
		break;
		
	default:
		// If it gets here it's no error, probably the calibration is finished
		break;
		
	}
}

/***********************************************************/
/*! \brief Returns the current state of fast rc calibration
 *
 */
RCCAL_CalibState_t RCCAL_GetState()
{
	return RCCAL_CalibState;
}


/******************************************************************************/
/*! \brief Measure OSI prescaled clock periods.
 *
 *	Measure number of Fast RC clock cycles in RC_CALIB_CYCLES number of
 *	Oscillator Sampling Interface prescaled Clock periods. The prescaler is 128
 *	for ATmega32HVB. osi_cycles is typically 8, giving a total of 1024 cycles
 *	for the selected input clock. Available inputs are Slow RC and ULP RC.
 *
 *	Warning: The routine only takes into account one overflow of OCR0, so make
 *	sure the number of Fast RC cycles used for calibration are less than 2^16.
 *
 *	\param	clock_source Which clock to use (slow RC or ULP RC)
 *
 */
void RCCAL_StartOSIsampleClockPeriod(uint8_t clock_source)
{
	OSICSR = clock_source | (1 << OSIEN);	// OSI enable and select
	TCCR0A = (1 << TCW0) | (1 << ICEN0) | (0 << ICS0) | (1<<ICES0);	// 16-bit mode, falling edge trigger
	TCCR0B = T0_PRESCALER;					// Timer0 running with 1x prescaling.
	TIFR0 |= (1 << ICF0);					// Clear interrupt flag in case it was set from before.
	TIMSK0 |= (1<<ICIE0);					// Enable the input capture interrupt
	
	TCNT0 = 0; // Clear the counter to avoid overflow
	
	RCCAL_IC_State = RCCAL_IC_INITIAL_CAPTURE;
	
}

/******************************************************/
/*! \brief Interrupt routine for Timer0 input capture
 *
 *	Measure number of Fast RC clock cycles in RC_CALIB_CYCLES number of
 *	Oscillator Sampling Interface prescaled Clock periods. The prescaler is 128
 *	for ATmega32HVB. osi_cycles is typically 8, giving a total of 1024 cycles
 *	for the selected input clock. Available inputs are Slow RC and ULP RC.
 *
 *	Warning: The routine only takes into account one overflow of OCR0, so make
 *	sure the number of Fast RC cycles used for calibration are less than 2^16.
 *
 */
#pragma vector=TIMER0_CAPT_vect
__interrupt void RCCAL_InputCapture_ISR()
{
	static uint8_t i = 0;
	static uint16_t initial_capture, final_capture;
	
	
	if(RCCAL_IC_INITIAL_CAPTURE == RCCAL_IC_State ) {
		i = 0;
		initial_capture = OCR0A;
		initial_capture |= OCR0B << 8;
		RCCAL_IC_State = RCCAL_IC_CAPTURES;
	
	} else {
		++i;
		if(i == RC_CALIB_CYCLES) {
			final_capture = OCR0A;
			final_capture |= OCR0B << 8;
			LastICResult = final_capture - initial_capture;
			RCCAL_IC_State = RCCAL_IC_FINISHED;
			TIMSK0 &= ~(1<<ICIE0);
		}
	}
}
