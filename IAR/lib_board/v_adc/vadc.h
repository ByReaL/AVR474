/* This file has been prepared for Doxygen automatic documentation generation.*/
/*! \file *********************************************************************
 *
 * \brief
 *      Headerfile for vadc_usage.c.
 *
 *      Contains defines and prototypes for vadc_usage.c.
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

#ifndef VADC_H
#define VADC_H

/* Includes. */

#include "common.h"
#include "battery_pack_parameters.h"

/* Defines. */

//! Number of bit for scaling results (16384). See VADC in datasheets for formulas.
#define VADC_SCALE_BITS     14

//! Upscaling the result from VADC voltage measurements to increase accuracy in temperature
//! measurements. In bits. (note that it is already scaled up by 10 due to the nature of the vadc)
#define VADC_TEMP_VOLTAGE_SCALING 1

//! Number of adc Cell inputs. Defines size of many variables, so must be 2 if 
//  runtime configuration of number of cells shall be used.
#define CELL_INPUTS         BATTPARAM_CELLS_IN_SERIES
//! Number of adc Cell inputs. Defines size of many variables, so must be 2 if 
//  runtime configuration of number of cells shall be used.
#define VADC_INPUTS         CELLTEMPERATURE_INPUTS


typedef enum vadcStateEnum { VADC_NOT_RUNNING = 0, VADC_RUNNING, VADC_SCAN_FAILED } vadcState_t;

struct VADC_Status {
	vadcState_t	state : 2;
	uint8_t     cell1VoltageReady : 1;
	uint8_t     cell2VoltageReady : 1;
	uint8_t     cell3VoltageReady : 1;
	uint8_t     cell4VoltageReady : 1;
	uint8_t     vtempReady : 1;
	uint8_t     cellTempReady : 1;
};

// !!!!! Be sure to update both the enum and define
//! Typedef used where number of cells is input to functions.
typedef enum vadcIndex { CELL1 = 1, CELL2 = 2, CELL3 = 3, CELL4 = 4, VTEMP=5, VADC0 = 6, VADC1 = 7 } vadcIndex_t;
// Defines used in preprocessor directives
#define DEF_CELL1 1
#define DEF_CELL2 2
#define DEF_CELL3 3
#define DEF_CELL4 4
#define DEF_VTEMP 5
#define DEF_VADC0 6
#define DEF_VADC1 7

// Deep undervoltage levels for HVB.
#define VB_DUVR_DFET         500    //!< Voltage across DFET diode




/***** Prototypes. */

void VADC_ScanConfig(vadcIndex_t Cells, vadcIndex_t Vadcs, vadcIndex_t vtempScan);
uint8_t VADC_GetScanConfig();
err_t VADC_StartScan(void);
vadcState_t VADC_ScanState(void);
void VADC_ClearReadyFlags();
bool VADC_CellTempReady(void);
bool VADC_VTempReady(void);
bool VADC_Cell1VoltageReady(void);
bool VADC_Cell2VoltageReady(void);
bool VADC_Cell3VoltageReady(void);
bool VADC_Cell4VoltageReady(void);

void VADC_InitializeVadcCoefficients(void);
uint16_t VADC_readCellVoltage(vadcIndex_t cellIndex);
uint16_t VADC_readVadc(vadcIndex_t vadcIndex);
uint16_t VADC_readSystemTemperature(void);

err_t DUVR_DisableCheck(void);



#endif // VADC_H

