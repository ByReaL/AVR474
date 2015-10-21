/* This file has been prepared for Doxygen automatic documentation generation.*/
/*! \file *********************************************************************
 *
 * \brief
 *      Signature row defines for ATmega16HVB and ATmega32HVB.
 *
 *      Contains define for signaure reading and address defines for the bytes.
 *
 * \par Application note:
 *      AVR353: Voltage Reference Calibration and Voltage ADC Usage
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


/*! Address defines for the signature row data in ATmega16HVB and ATmega32HVB.
	See datasheet and application note for details. */
#define SIG_DEVICE_ID0           0x00    // 0x1E
#define SIG_FOSCCAL_DEFAULT      0x01
#define SIG_DEVICE_ID1           0x02    // 0x95(32HVB) / 0x94(16HVB)
#define SIG_FOSC_SEGMENT         0x03
#define SIG_DEVICE_ID2           0x04    // 0x10(32HVB) / 0x0D(16HVB)
#define SIG_RESERVED_0x05        0x05
#define SIG_SLOW_RC_L            0x06
#define SIG_SLOW_RC_H            0x07
#define SIG_SLOW_RC_PRED_L       0x08
#define SIG_SLOW_RC_PRED_H       0x09
#define SIG_ULP_RC_FREQ          0x0A
#define SIG_SLOW_RC_FREQ         0x0B
#define SIG_ULP_RC_PRED_L        0x0C
#define SIG_ULP_RC_PRED_H        0x0D

#define SIG_ULP_RC_L             0x0E
#define SIG_ULP_RC_H             0x0F
#define SIG_BGCRR_CALIB_25C      0x10
#define SIG_BGCCR_CALIB_25C      0x11
#define SIG_RESERVED_0x12        0x12
#define SIG_BGCCR_CALIB_HOT      0x13

#define SIG_VADC_RAW_CELL1_L     0x14
#define SIG_VADC_RAW_CELL1_H     0x15
#define SIG_RESERVED_0x16        0x16
#define SIG_RESERVED_0x17        0x17
#define SIG_VPTAT_L              0x18
#define SIG_VPTAT_H              0x19
#define SIG_VADC_CELL1_GAIN_L    0x1A
#define SIG_VADC_CELL1_GAIN_H    0x1B
#define SIG_VADC_CELL2_GAIN_L    0x1C
#define SIG_VADC_CELL2_GAIN_H    0x1D
#define SIG_VADC_CELL3_GAIN_L    0x1E
#define SIG_VADC_CELL3_GAIN_H    0x1F
#define SIG_VADC_CELL4_GAIN_L    0x20
#define SIG_VADC_CELL4_GAIN_H    0x21
#define SIG_VADC_CELL1_OFFSET    0x22
#define SIG_VADC_CELL2_OFFSET    0x23
#define SIG_VADC_CELL3_OFFSET    0x24
#define SIG_VADC_CELL4_OFFSET    0x25
#define SIG_VADC_ADC0_GAIN_L     0x26
#define SIG_VADC_ADC0_GAIN_H     0x27
#define SIG_VADC_ADC1_GAIN_L     0x28
#define SIG_VADC_ADC1_GAIN_H     0x29
#define SIG_VADC_ADC0_OFFSET     0x2A
#define SIG_VADC_ADC1_OFFSET     0x2B
#define SIG_C1_C2_ALPHA          0x2C
#define SIG_C1_C2_BETA           0x2D
#define SIG_TEMPERATURE_HOT      0x30

// TODO the following are not mentioned in the datasheet
#define SIG_CCADC_OFFSET_L       0x34
#define SIG_CCADC_OFFSET_H       0x35
#define SIG_CRC16_FT_L           0x3C
#define SIG_CRC16_FT_H           0x3D
#define SIG_CRC16_2ND_CAL_L      0x3E
#define SIG_CRC16_2ND_CAL_H      0x3F

#define SIG_END_ADDRESS          0x6B    //!< End of signature row.



