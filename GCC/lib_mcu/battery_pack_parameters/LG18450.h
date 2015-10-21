/* This file has been prepared for Doxygen automatic documentation generation.*/
/*! \file *********************************************************************
 *
 * \brief
 *      Cell specific values
 *
 *      Contains define values that are used in calculations. This information
 *      may be gleaned from datasheets for the cells, or may be determined
 *      experimentally. Not everything in this file is currently used, but it
 *      provides a good framework.
 *
 * \par Application note:
 *      AVR453: Smart Battery Reference Design
 *
 * \par Documentation:
 *      For comprehensive code documentation, supported compilers, compiler
 *      settings and supported devices see readme.html
 *
 * \author
 *      Atmel Corporation: http://www.atmel.com \n
 *      Support email: avr@atmel.com \n
 *      Original author: Rob G. Fries - Apt Inc.\n
 *
 * $Revision: 5627 $
 * $URL: http://norsvn01.norway.atmel.com/apps/appsavr/avr474_SB202_HVB_2_3_4_cell_smart_battery_firmware/trunk/Code/lib_mcu/battery_pack_parameters/LG18450.h $
 * $Date: 2009-05-15 14:49:21 +0800 (Fri, 15 May 2009) $  \n
 ******************************************************************************/


//! \todo  This file must be modified for your particular individual primary cell.

// Basic capacity information
#define CELL_DESIGNCAPTYP 2200		/* in mAh, typical capacity for a single cell */
#define CELL_DESIGNCAPC5 2150		/* in mAh, minimum capacity at C/5 for a single cell */
#define CELL_NOMINALV 3700		/* in mV, typical cell V when fully charged */
//#define CELL_TOOMUCHV 3710		/* in mV, where we must force disconnect if still charging! */
#define CELL_TOOMUCHV 5500		/* in mV, where we must force disconnect if still charging! */


// Discharge information
#define CELL_MINV 3000			/* in mV, absolute low-end voltage permitted */
#define CELL_DISCHG_C5 430		/* discharge current (mA) for C/5 rate (actually 2150mAh capacity figure) */
#define CELL_DISCHG_MAX 4300		/* maximum permissible discharge current (mA) */
//#define CELL_TOOLITTLEV 2990		/* in mV, where we must force disconnect if still discharging! */
#define CELL_TOOLITTLEV 10		/* in mV, where we must force disconnect if still discharging! */

// Charging information
#define CELL_CCCV_V 4200		/* Constant Current / Constant Voltage charging mode, applied voltage (mV) */
#define CELL_CCCV_C 1075		/* Constant Current / Constant Voltage charging mode, applied current (mA) */
#define CELL_CCCV_TERMC 50		/* Constant Current / Constant Voltage charging mode, charge-termination current threshold (mA) */
#define CELL_MAX_CHG_C 2150		/* Maximum permissible charging current (mA) */


// Thermal limits
#define CELL_CHG_TEMP_MIN 0		/* in 'C */
#define CELL_CHG_TEMP_MAX 45		/* in 'C */
#define CELL_DISCHG_TEMP_MIN -20	/* in 'C */
#define CELL_DISCHG_TEMP_MAX 60		/* in 'C */


// Thermal adjustments to capacity
#define CELL_NOMINAL_TEMP 23		/* in 'C; the temperature at which nominal capacity is specified  */
#define CELL_LOWTEMP1 0			/* in 'C, second low-temp point for thermal calculations */
#define CELL_LOWTEMP1_CAPACITY 90	/* in percent, relative to capacity at CELL_NOMINAL_TEMP */
#define CELL_LOWTEMP2 -10		/* in 'C, second low-temp point for thermal calculations */
#define CELL_LOWTEMP2_CAPACITY 70	/* in percent, relative to capacity at CELL_NOMINAL_TEMP */
#define CELL_HITEMP 60			/* in 'C, second low-temp point for thermal calculations */
#define CELL_HITEMP_CAPACITY 95		/* in percent, relative to capacity at CELL_NOMINAL_TEMP */

//Other important values...
#define MAX_IMBALANCE 50			/* in mV, how much imbalance we tolerate before doing Balancing, 254 max */
#define MAX_ONCHIP_TEMP ((70+273)*10)	/* in 0.1'K, maximum CHIP temperature we EVER want to see      */
#define MAX_CELL_TEMPERATURE ((60+273)*10)	/* in 0.1'K, maximum CELL temperature we EVER want to see */


