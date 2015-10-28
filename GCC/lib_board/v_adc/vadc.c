/* This file has been prepared for Doxygen automatic documentation generation.*/
/*! \file *********************************************************************
 *
 * \brief
 *      Routines for VADC measurements.
 *
 *      This file contains all functions used for sapmling VADC inputs, that is
 *      Cells, normal vadc iputs that can be used for thermistor measurements,
 *      and lastly the internal temperature reference.
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

#include "vadc.h"
#include "iar_compat.h"

/* File global variables. */

static uint16_t cellGain[CELL_INPUTS];         //!< Gain coefficients for Cells.
static int8_t   cellOffset[CELL_INPUTS];       //!< Offset for cells.
static uint16_t cellMeasurement[CELL_INPUTS];  //!< Cell input samples from the vadc ISR.
static uint16_t cellVoltage[CELL_INPUTS];      //!< Cell input voltage. Set to 0xFFFF if not calculated from latest cellMeasurement

#if VADC_INPUTS != 0
static uint16_t vadcGain[VADC_INPUTS];         //!< VADC channel gain.
static int8_t   vadcOffset[VADC_INPUTS];       //!< VADC channel offset.
static uint16_t vadcMeasurement[VADC_INPUTS];  //!< VADC input samples from the vadc ISR.
#endif

static uint16_t vptatCoeff;                    //!< Internal temperature sensor coefficient
static uint16_t vtempMeasurement;              //!< VTEMP input sample from the vadc ISR.

struct VADC_Status vadcStatus;

/*! \brief VADC channel sampling config.
 *
 * Configuration of which VADC channels to be scanned. A bit is used for each
 * input according to the MUX setting, e.g. (1 << CELL1) for cell1 measurements.
 *
 * VADC_ScanConfig() configures this variable. It is used by VADC_StartScan()
 * to select first input, while VADC_ISR() then scans the channels selected in a
 * fixed order but only those selected in vadcScanConfig.
 *
 * The highest MUX setting that can be contained in this 8 bit variable is 7, so
 * it needs to be expanded for devices with more vadc inputs than ATmega16HVA.
 * CELL1 is assumed to always be measured.
 */
static uint8_t vadcScanConfig;





/**** Functions. */


/*! \brief Select VADC channels to be scanned.
 *
 * Configuration of which VADC channel(s) to be sampled is stored in the file
 * global variable #vadcScanConfig. Enums used from #vadcIndex_t .
 *
 * \param CellInputs Name of highest cell to be sampled.
 * \param VadcInputs Name of Highest vadc input to be sampled.
 * \param vtempScan     Scan internal temperature sensor if #VTEMP.
 */
void VADC_ScanConfig(vadcIndex_t CellInputs, vadcIndex_t VadcInputs, vadcIndex_t vtempScan)
{
	vadcScanConfig = 0;
	
#if CELL_INPUTS >= 4
	if ( CellInputs == CELL4 ) {
		vadcScanConfig |= (1 << CELL4) | (1 << CELL3) | (1 << CELL2) | (1 << CELL1);
	}
#endif
#if CELL_INPUTS >= 3
	if ( CellInputs == CELL3 ) {
		vadcScanConfig |= (1 << CELL3) | (1 << CELL2) | (1 << CELL1);
	}
#endif
#if CELL_INPUTS >= 2
	if ( CellInputs == CELL2 ) {
		vadcScanConfig |= (1 << CELL2) | (1 << CELL1);
	}
#endif
	if ( CellInputs == CELL1 ) {
		vadcScanConfig |= (1 << CELL1);
	}
	if (VadcInputs == VADCZ1) {
		vadcScanConfig |= (1 << VADCZ1)| (1 << VADCZ0);
	}
	if (VadcInputs == VADCZ0) {
		vadcScanConfig |= (1 << VADCZ0);
	}
	if (vtempScan == VTEMP) {
		vadcScanConfig |= (1 << VTEMP);
	}
}

/*! \brief Return the current ScanConfig
 *
 */
uint8_t VADC_GetScanConfig() {
	return vadcScanConfig;
}

/*! \brief Clear the ready flags
 */
void VADC_ClearReadyFlags()
{
#if CELL_INPUTS >= 4
	vadcStatus.cell4VoltageReady = false;
#endif
#if CELL_INPUTS >= 3
	vadcStatus.cell3VoltageReady = false;
#endif
#if CELL_INPUTS >= 2
	vadcStatus.cell2VoltageReady = false;
#endif
	vadcStatus.cell1VoltageReady = false;
	vadcStatus.cellTempReady = false;
	vadcStatus.vtempReady = false;
}

/*! \brief Start a scan of all VADC channels.
 *
 * VADC is set up and interrupts enabled. The first vadc channel to be sampled
 * is selected according to #vadcScanConfig. The interrupt routine VADC_ISR
 * does the actual vadc measurements. Global interrupts need to be enabled in
 * main for the vadc interrupt routine to be run.
 *
 * \return #SUCCESS or #FAILURE
 */
err_t VADC_StartScan(void)
{
	err_t status = SUCCESS;
	
	// start set up for first channel.
	if ( vadcScanConfig & (1 << VTEMP) ) {
		VADMUX = VTEMP;
	} else if ( vadcScanConfig & (1 << VADCZ0) ) {
		VADMUX = VADCZ0;
	} else if ( vadcScanConfig & (1 << CELL1) ) {
		VADMUX = CELL1;
	} else {
		status = FAILURE;
	}
	if (status == SUCCESS) {
		VADCSR = (1<<VADCCIF);     //clear any pending interrupt
		VADCSR = (1<<VADEN) | (1<<VADSC) | (1<<VADCCIE);  //Start the first conversion & enable the vadc.
		// Global interrupts need to be enabled in main for the vadc interrupt routine to be run.
		vadcStatus.state = VADC_RUNNING;
		VADC_ClearReadyFlags();
	}
	return status;
}


/*!	\brief Interrupt Service Function to measure VADC values.
 *
* This ISR scans all vadc channels selected by vadcScanConfig and then
 * disables itself.  It continues from the input selected in VADC_StartScan().
 * The order is fixed; VTEMP, ADC0, ADC1, CELL1 and CELL2 last. Measurements
 * are stored in global variables  vtempMeasurement, vadcMeasurement and
 * cellMeasurement. Cells are measured last to give the possiblity to
 * disable cell balancing (if applicable).
 */
ISR(VADC_vect)
{
	uint8_t state;
	
	state = VADMUX;
	
	switch (state) {
	case VTEMP :
		vtempMeasurement = VADC;
		if (vadcScanConfig & (1 << VADCZ0) ) {
			VADMUX = VADCZ0;
		} else {
			VADMUX = CELL1;
		}
		vadcStatus.vtempReady = true;
		break;
#if VADC_INPUTS != 0
	case VADCZ0 :
		vadcMeasurement[0] = VADC;
		if ( vadcScanConfig & (1 << VADCZ1) ) {
			VADMUX = VADCZ1;
		} else {
			VADMUX = CELL1;
			vadcStatus.cellTempReady = true;
		}
		break;
  #if VADC_INPUTS >= 2
	case VADCZ1 :
		vadcMeasurement[1] = VADC;
		VADMUX = CELL1;
		vadcStatus.cellTempReady = true;
		break;
  #endif // VADC_INPUTS >= 2
#endif // VADC_INPUTS != 0
	case CELL1 :
		cellMeasurement[0] = VADC;
		cellVoltage[0] = 0xFFFF;
		vadcStatus.cell1VoltageReady = true;
		if ( vadcScanConfig & (1 << CELL2) ) {
			VADMUX = CELL2;
		} else {
			vadcStatus.state = VADC_NOT_RUNNING;
		}
		break;
#if CELL_INPUTS >= 2
	case CELL2 :
		cellMeasurement[1] = VADC;
		cellVoltage[1] = 0xFFFF;
		vadcStatus.cell2VoltageReady = true;
		if ( vadcScanConfig & (1 << CELL3) ) {
			VADMUX = CELL3;
		} else {
			vadcStatus.state = VADC_NOT_RUNNING;
		}
		break;
#endif // CELL_INPUTS >= 2
#if CELL_INPUTS >= 3
	case CELL3 :
		cellMeasurement[2] = VADC;
		cellVoltage[2] = 0xFFFF;
		vadcStatus.cell3VoltageReady = true;
		if ( vadcScanConfig & (1 << CELL4) ) {
			VADMUX = CELL4;
		} else {
			vadcStatus.state = VADC_NOT_RUNNING;
		}
		break;
#endif // CELL_INPUTS >= 3
#if CELL_INPUTS >= 4
	case CELL4 :
		cellMeasurement[3] = VADC;
		cellVoltage[3] = 0xFFFF;
		vadcStatus.cell4VoltageReady = true;
		vadcStatus.state = VADC_NOT_RUNNING;
		break;
#endif // CELL_INPUTS >= 4


	    default:
		vadcStatus.state = VADC_SCAN_FAILED;
		break;
	}
	
	if (vadcStatus.state == VADC_RUNNING) {
		VADCSR |= (1 << VADSC);         // Start next conversion now.
	} else {
		VADCSR &= ~(1 << VADEN);        // Disable VADC (until next scan).
	}
}


/*!	\brief Check VADC scan status.
 *
 * The internal VADC_Status struct
 *
 * \return vadcStatus.state
 */
vadcState_t VADC_ScanState(void)
{
	return vadcStatus.state;
}

/*! \brief Return true if new cell temperature readings are available */
bool VADC_CellTempReady(void)
{
	uint8_t interruptState = __save_interrupt();
	__disable_interrupt();
	
	bool flagState = vadcStatus.cellTempReady;
	
	__restore_interrupt( interruptState );
	
	return flagState;
}

/*! \brief Return true if new core temperature reading s available */
bool VADC_VTempReady(void)
{
	uint8_t interruptState = __save_interrupt();
	__disable_interrupt();
	
	bool flagState = vadcStatus.vtempReady;
	
	__restore_interrupt( interruptState );
	
	return flagState;
}

/*! \brief Return true if new cell1 voltage readings are available */
bool VADC_Cell1VoltageReady(void)
{
	uint8_t interruptState = __save_interrupt();
	__disable_interrupt();
	
	bool flagState = vadcStatus.cell1VoltageReady;
	
	__restore_interrupt( interruptState );
	
	return flagState;
}

/*! \brief Return true if new cell2 voltage readings are available */
#if CELL_INPUTS >= 2
bool VADC_Cell2VoltageReady(void)
{
	uint8_t interruptState = __save_interrupt();
	__disable_interrupt();
	
	bool flagState = vadcStatus.cell2VoltageReady;
	
	__restore_interrupt( interruptState );
	
	return flagState;
}
#endif


/*! \brief Return true if new cell3 voltage readings are available */
#if CELL_INPUTS >= 3
bool VADC_Cell3VoltageReady(void)
{
	uint8_t interruptState = __save_interrupt();
	__disable_interrupt();
	
	bool flagState = vadcStatus.cell3VoltageReady;
	
	__restore_interrupt( interruptState );
	
	return flagState;
}
#endif


/*! \brief Return true if new cell4 voltage readings are available */
#if CELL_INPUTS >= 4
bool VADC_Cell4VoltageReady(void)
{
	uint8_t interruptState = __save_interrupt();
	__disable_interrupt();
	
	bool flagState = vadcStatus.cell4VoltageReady;
	
	__restore_interrupt( interruptState );
	
	return flagState;
}
#endif


/*! \brief Initialization of variables for cell voltage, adc voltage and vtemp
 *         calculations.
 *
 * The coefficients are stored in (file) global variables. Interrupts are
 * disabled and re-enabled if needed afterwards.
 */
void VADC_InitializeVadcCoefficients(void)
{	
	uint8_t interrupt_status = __save_interrupt();  // Disable interrupts.
	__disable_interrupt();
	
	cellGain[0] = READ_SIGNATUREWORD(SIG_VADC_CELL1_GAIN_L);
	cellOffset[0] = READ_SIGNATUREBYTE(SIG_VADC_CELL1_OFFSET);

#if CELL_INPUTS >= 2
	cellGain[1] = READ_SIGNATUREWORD(SIG_VADC_CELL2_GAIN_L);
 	cellOffset[1] = READ_SIGNATUREBYTE(SIG_VADC_CELL2_OFFSET);
#endif
	
#if CELL_INPUTS >= 3
	cellGain[2] = READ_SIGNATUREWORD(SIG_VADC_CELL3_GAIN_L);
 	cellOffset[2] = READ_SIGNATUREBYTE(SIG_VADC_CELL3_OFFSET);
#endif
	
#if CELL_INPUTS >= 4
	cellGain[3] = READ_SIGNATUREWORD(SIG_VADC_CELL4_GAIN_L);
 	cellOffset[3] = READ_SIGNATUREBYTE(SIG_VADC_CELL4_OFFSET);
#endif

	vptatCoeff = READ_SIGNATUREWORD(SIG_VPTAT_L);

#if VADC_INPUTS >= 1
	vadcGain[0] = READ_SIGNATUREWORD(SIG_VADC_ADC0_GAIN_L);
	vadcOffset[0] = READ_SIGNATUREBYTE(SIG_VADC_ADC0_OFFSET);
#endif
	
#if VADC_INPUTS >= 2
	vadcGain[1] = READ_SIGNATUREWORD(SIG_VADC_ADC1_GAIN_L);
	vadcOffset[1] = READ_SIGNATUREBYTE(SIG_VADC_ADC1_OFFSET);
#endif
	
	__restore_interrupt(interrupt_status);
}


/*! \brief Calculation of cell voltage.
 *
 * \param cellIndex Which cell to calculate voltage for. The #cellMeasurement
 *       array is used as source.
 * \return Voltage for selected cell index [mV] or zero if not valid.
 */
uint16_t VADC_readCellVoltage(vadcIndex_t cellIndex)
{
	uint32_t calc;
	
	cellIndex -= CELL1;

	if (cellIndex < CELL_INPUTS) {
		uint8_t interruptState = __save_interrupt();
		__disable_interrupt();
		
		// If cellVoltage is invalid, get the measurement so we can calculate the cell voltage
		// If it's valid, return it as quick as possible
		// TODO: a bit ugly with the double __restore_interrupt, but i couldn't figure out a simplier (fast) way
		if (0xFFFF == cellVoltage[cellIndex] ) {
			calc = cellMeasurement[cellIndex];
		} else {
			__restore_interrupt(interruptState);
			return cellVoltage[cellIndex];
		}
		__restore_interrupt(interruptState);
		
		// Don't allow negative values
		if(calc < cellOffset[cellIndex]) {
			calc = 0;
		} else {
			calc -= cellOffset[cellIndex];
			calc *= cellGain[cellIndex];
			calc >>= VADC_SCALE_BITS;
		}
		cellVoltage[cellIndex] = (uint16_t)calc;
	} else {
		calc = 0;
	}
	
	return (uint16_t)calc;
}

#if VADC_INPUTS != 0
/*! \brief Calculation of vadc voltage.
 *
 * Returns the voltage of the vadc scaled by 10 * 2^VADC_TEMP_VOLTAGE_SCALING.
 * To get the temperature, send this value to the NTC module.
 *
 * \param vadcIndex Which vadc input to calculate voltage for. The #vadcMeasurement
 *        array is used as source.
 * \return Voltage for selected vadc index [mV] * 10 * 2^VADC_TEMP_VOLTAGE_SCALING or zero if index is not valid.
 */

uint16_t VADC_readVadc(vadcIndex_t vadcIndex)
{

	uint32_t calc;
	
	vadcIndex -= VADCZ0;

	if (vadcIndex < VADC_INPUTS) {
		uint8_t interruptState = __save_interrupt();
		__disable_interrupt();
		calc = vadcMeasurement[vadcIndex];
		__restore_interrupt(interruptState);
		// Don't allow negative values
		if( calc < vadcOffset[vadcIndex] ) {
			calc = 0;
		} else {
			calc -= vadcOffset[vadcIndex];
			calc *= vadcGain[vadcIndex];
			calc >>= (VADC_SCALE_BITS - VADC_TEMP_VOLTAGE_SCALING);
		}
	} else {
		calc = 0;
	}

	return (uint16_t)calc;
}
#endif // VADC_INPUTS != 0



/*! \brief Calculation of chip temperature.
 *
 * The #vtempMeasurement variable sampled from the internal temperature sensor
 * is used to calculate the chip temperature.
 *
 * \return Temperature in 0.1 kelvin [K].
 */
uint16_t VADC_readSystemTemperature(void)
{
	uint32_t calc;

	uint8_t interruptState = __save_interrupt();
	__disable_interrupt();
	calc = vtempMeasurement;
	__restore_interrupt(interruptState);
	calc *= (uint32_t)vptatCoeff*10; // Times 10 to get result in 0.1Kelvin
	calc >>= VADC_SCALE_BITS;

	return (uint16_t)(calc);
}


/*! \brief Check if DUVR mode can be disabled and if so disable it.
 *
 * If the voltage on the cells are sufficiently high after a reset the Deep
 * Under-Voltage Recovery (DUVR) mode should be disabled and possibly the Charge
 * FET (CFET) enabled.
 * This function assumes a VADC scan has been performed and uses the cell
 * samples to calculate cell voltages.
 *
 * \return #SUCCESS or #FAILURE
 */
err_t DUVR_DisableCheck(void)
{
	bool disableDuvr;
	bool enableDischargeFet;
	uint16_t temp_cell;
	
	temp_cell = VADC_readCellVoltage(CELL1);

	if ( temp_cell > battParams.cellDuvrDisableVoltage ) {
		disableDuvr = true;
		//FCSR |= (1 << CFE) | (1 << DUVRD);
		if ( temp_cell > (battParams.cellDuvrDisableVoltage + VB_DUVR_DFET) ) {
			enableDischargeFet = true;
		} else {
			enableDischargeFet = false;
		}
	} else {
		disableDuvr = false;
	}
	
	// If cell1 was above duvr disable limits, check if cell2 is
# if BATTPARAM_CELLS_IN_SERIES >= 2
	temp_cell = VADC_readCellVoltage(CELL2);
	if( disableDuvr ) {
		if ( temp_cell <= battParams.cellDuvrDisableVoltage ) {
			disableDuvr = false;
		} else if(enableDischargeFet) {
			if ( temp_cell <= (battParams.cellDuvrDisableVoltage + VB_DUVR_DFET) ) {
				enableDischargeFet = false;
			}
		}
	}
# endif

# if BATTPARAM_CELLS_IN_SERIES >= 3
	temp_cell = VADC_readCellVoltage(CELL3);
	if( disableDuvr ) {
		if ( temp_cell <= battParams.cellDuvrDisableVoltage ) {
			disableDuvr = false;
		} else if(enableDischargeFet) {
			if ( temp_cell <= (battParams.cellDuvrDisableVoltage + VB_DUVR_DFET) ) {
				enableDischargeFet = false;
			}
		}
	}
# endif

# if BATTPARAM_CELLS_IN_SERIES >= 4
	temp_cell = VADC_readCellVoltage(CELL4);
	if( disableDuvr ) {
		if ( temp_cell <= battParams.cellDuvrDisableVoltage ) {
			disableDuvr = false;
		} else if(enableDischargeFet) {
			if ( temp_cell <= (battParams.cellDuvrDisableVoltage + VB_DUVR_DFET) ) {
				enableDischargeFet = false;
			}
		}
	}
# endif
	
	if( disableDuvr ) {
		if( enableDischargeFet ) {
			FCSR = (1<<CFE) | (1<<DFE) | (1<<DUVRD);
		} else {
			FCSR = (1<<CFE) | (1<<DUVRD);
		}
		return SUCCESS;
	} else {
		return FAILURE;
	}
}


