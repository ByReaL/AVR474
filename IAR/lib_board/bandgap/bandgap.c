/* This file has been prepared for Doxygen automatic documentation generation.*/
/*! \file *********************************************************************
 *
 * \brief
 *      Routine for voltage reference calibration.
 *
 *      This file contains the function for calibration of the bandgap voltage
 *      reference.
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


/* Includes. */
#include "common.h"
#include "ATmega32HVB_signature.h"



/* Functions. */

/*! \brief Bandgap initialisation.
 *
 * Initialises the bandgap voltage reference to factory calibration. Unless
 * values from second insertion exists #FAILURE is returned.
 *
 * \return #SUCCESS or #FAILURE
 */
err_t BANDGAP_Initialization(void)
{
	uint8_t bgccr_cal;
	uint8_t bgcrr_cal;
	err_t status;

	// Read BGCRR value from signature row and check that it is valid.
	bgcrr_cal = READ_SIGNATUREBYTE(SIG_BGCRR_CALIB_25C);
	bgccr_cal = READ_SIGNATUREBYTE(SIG_BGCCR_CALIB_25C);
	if ( bgccr_cal == 0xFF ) {		
		status = FAILURE;
	}  else {
		/* The values from the signature row are valid. BGCRR can be set freely,
		   while BGCCR needs to be adjusted slowly upwards (20us/step) to avoid BOD.*/
		BGCRR = bgcrr_cal;
        while (BGCCR < bgccr_cal) {		// Adjust BGCCR up if needed.
			BGCCR++;
			__delay_cycles(20*SYSTEM_CLK_HZ/1000000);
		}
		/* Adjust BGCCR down in case it has been adjusted previously. This will not
		   give a BOD, but ADC measurements will be erroneous until VREF is stabilized.*/
		BGCCR = bgccr_cal;
		status = SUCCESS;
	}
	return status;
}
