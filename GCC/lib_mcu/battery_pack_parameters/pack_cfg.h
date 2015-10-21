/* This file has been prepared for Doxygen automatic documentation generation.*/
/*! \file *********************************************************************
 *
 * \brief
 *       Battery pack configuration.
 *
 *       This file defines the number of cells in series, as well as in
 *       parallel. Modify the values to match your pack's configuration.
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
 * $URL: http://norsvn01.norway.atmel.com/apps/appsavr/avr474_SB202_HVB_2_3_4_cell_smart_battery_firmware/trunk/Code/lib_mcu/battery_pack_parameters/pack_cfg.h $
 * $Date: 2009-05-15 14:49:21 +0800 (Fri, 15 May 2009) $  \n
 ******************************************************************************/


//! \todo  Modify these settings to match the battery configuration in your design.

#define PACKWIDTH 3	/* # of cells used in parallel to increase the pack's current capability. */
#define PACKSTACK 4	/* # of cells stacked in series; must be ONLY   2, 3 or 4   for the m406! */

