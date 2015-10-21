/* This file has been prepared for Doxygen automatic documentation generation.*/
/*! \file *********************************************************************
 *
 * \brief
 *      Battery cell voltage monitoring firmware module.
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

/******************************************************************************
 Included files
******************************************************************************/
#include "common.h"
#include "battery_voltage_monitoring.h"
#include "vadc.h"
#include "battery_pack_parameters.h"

/******************************************************************************
 Function declarations.
******************************************************************************/

/******************************************************************************/
/*! \brief Investigate if the battery cells are too low.
 *
 * Requires that a V-ADC scan has been run.
 *
 *	\param cell  Which cell to check. Is not checked by this function, but readCellVoltage will return 0
 *               if the cell is invalid (and thus this function will always return true in that case)
 *
 *	\return True is returned if any of the cells have too low voltage, otherwise false.
 */
bool BATTVMON_IsCellVoltageTooLow( vadcIndex_t cell )
{
	bool status = false;
	
	if( battParams.cellMinVoltage > VADC_readCellVoltage( cell ) ){
		status = true; // Voltage of cell is too low!
		
	}
	
	return status;
}

/******************************************************************************/
/*! \brief Investigate if any of the battery cells are too high.
 *
 * Requires that a V-ADC scan has been run.
 *
 *	\param cell  Which cell to check. Is not checked by this function, but readCellVoltage will return 0
 *               if the cell is invalid (and thus this function will always return false in that case)
 *
 *	\return True is returned if any of the cells have too high voltage, otherwise false.
 */
bool BATTVMON_IsCellVoltageTooHigh( vadcIndex_t cell )
{
	bool status = false;
	
	if( battParams.cellMaxVoltage < VADC_readCellVoltage( cell ) ){
		status = true; // Voltage of cell is too high!
	}

	return status;
}





/****************************************************
 ****************************************************

 Functions below only for more than 1 cell in series

 ****************************************************
 ****************************************************/


#if BATTPARAM_CELLS_IN_SERIES >= 2

/******************************************************************************/
/*! \brief Enable cell balancing: Check for unbalance, if so enables either
 *         of the cell balancing FETs.
 *
 *  This will enable cell balancing on the cells. Different parameters can be
 *  set up to different balancing schemes if that is needed.
 *  This implementation is written for devices with balancing circuit, but can
 *  be modified to be used with standard IO's for balancing.
 */
void BATTVMON_EnableCellBalancing( void )
{
	/* Get the misbalance threshold parameter from EEPROM. */
	uint16_t misbalanceVoltageThreshold = battParams.misbalanceVoltageThreshold;
	
	/* Temporary register for holding the enabling of cell balancing. */
	uint8_t BalCellFlag = 0;
	
	/* Table to hold the Cell voltages. */
	uint16_t cellVoltage[BATTPARAM_CELLS_IN_SERIES];
	
	/* Check if balancing is already enabled, no need to run this if it is. */
	if ( !BATTVMON_IsCellBalancingEnabled() ){
	
		/* Get voltage for cell 1 and 2 always. */
	cellVoltage[0] = VADC_readCellVoltage( CELL1 );
	cellVoltage[1] = VADC_readCellVoltage( CELL2 );

#if (BATTPARAM_CELLS_IN_SERIES >= 3)
		/* Get voltage for cell 3 if this is configured. */
		cellVoltage[2] = VADC_readCellVoltage( CELL3 );
			
#if (BATTPARAM_CELLS_IN_SERIES >= 4)
		/* Get voltage for cell 4 if this is configured. */
		cellVoltage[3] = VADC_readCellVoltage( CELL4 );
				
#endif /* (BATTPARAM_CELLS_IN_SERIES >= 4) */
#endif /* (BATTPARAM_CELLS_IN_SERIES >= 3) */
				
		/* Temporary values to find the cell with highest voltage and index
		 * to keep track of the cell number. */
		uint16_t highest_cell = cellVoltage[0];
		uint8_t  highest_cell_idx = 0;
		
		/* Temporary values to find the cell with lowest voltage. No index
		 * needed here as we're only interested in the voltage reference. */
		uint16_t lowest_cell = cellVoltage[0];
		
		/* Find the cell with the highest and lowest voltage */
		for(uint8_t cellCnt = 0; cellCnt < BATTPARAM_CELLS_IN_SERIES; cellCnt++){
			if (cellVoltage[cellCnt]>highest_cell){
				highest_cell = cellVoltage[cellCnt];
				highest_cell_idx = cellCnt;
			}
			if (cellVoltage[cellCnt]<lowest_cell){
				lowest_cell = cellVoltage[cellCnt];
		}
		
	}
	
#if (BATTPARAM_CELLS_IN_SERIES >= 3)
		/* Temporary values to find the cell with next highest voltage. This is
		* only used for 3 or more cells.*/
		uint16_t high_cell = 0;
		uint8_t  high_cell_idx = 0;
		
		/* Find the cell with the next highest voltage */
		for(uint8_t cellCnt = 0; cellCnt < BATTPARAM_CELLS_IN_SERIES; cellCnt++){
			if ( (cellVoltage[cellCnt]>high_cell) && (cellVoltage[cellCnt]<highest_cell) ){
				high_cell = cellVoltage[cellCnt];
				high_cell_idx = cellCnt;
			}
		}
	
#endif /* (BATTPARAM_CELLS_IN_SERIES >= 3) */
		
		/* Check if the difference between highest and lowest voltage is larger than the threshold. */
		if ( (highest_cell - lowest_cell) > misbalanceVoltageThreshold ){
			
			/* The cell with the largest voltage difference is to be balanced in any case */
			BalCellFlag = (1 << highest_cell_idx);
			
#if (BATTPARAM_CELLS_IN_SERIES >= 3)	
			/* Checking if the difference in cellvoltage of the next cell in the list is larger than the threshold */
			if ((high_cell - lowest_cell)> misbalanceVoltageThreshold){
				
				/* Checking that the cell is not physically next to the one already balancing */
				if (highest_cell_idx > high_cell_idx){
					if ( (highest_cell_idx - high_cell_idx) > 1 ){
						/* Turning on balancing for the next highest cell in the list */
						BalCellFlag |= (1 << high_cell_idx);
			}
				}else{
					if ( (high_cell_idx - highest_cell_idx) > 1 ){
						/* Turning on balancing for the next highest cell in the list */
						BalCellFlag |= (1 << high_cell_idx);
					}
				}
			}
#endif /*(BATTPARAM_CELLS_IN_SERIES >= 3) */
					
		}
	
		/* Setting the Cell balancing register. This will disable cell balancing
		* if the difference is below the threshold. */
		CBCR = BalCellFlag;
	
}
}


/******************************************************************************/
/*! \brief Provide information about whether the balancing fets are enabled (conducting).
 *
 *	\retval bool True is returned if either of the balancing FETs are enabled (conducting).
 */
bool BATTVMON_IsCellBalancingEnabled( void )
{
	bool status = false;
	
	if( CBCR&0x0F ){
		status = true;
	}
	return status;
}

#endif /* BATTPARAM_CELLS_IN_SERIES >= 2 */

