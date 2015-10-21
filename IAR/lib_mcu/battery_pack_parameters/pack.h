/* This file has been prepared for Doxygen automatic documentation generation.*/
/*! \file *********************************************************************
 *
 * \brief
 *       Battery pack definitions.
 *
 *       This file creates the *PACK's* voltage and current figures, based on
 *       both the individual cell specifications and the pack configuration.
 *
 * \par Application note:
 *      AVR474: SB202 Firmware user's guide.
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
 * $URL: http://norsvn01.norway.atmel.com/apps/appsavr/avr474_SB202_HVB_2_3_4_cell_smart_battery_firmware/trunk/Code/lib_mcu/battery_pack_parameters/pack.h $
 * $Date: 2009-05-15 14:49:21 +0800 (Fri, 15 May 2009) $  \n
 ******************************************************************************/



#include "LG18450.h"	//Change this #include file if using a different cell.
#include "pack_cfg.h"	//Adjust this file for the number of cells in series and in parallel.


// DO NOT MODIFY THE FOLLOWING DEFINITIONS!
// These are derived from the included files (above), so modify those instead.

#define PACK_DESIGNCAPTYP (CELL_DESIGNCAPTYP * PACKWIDTH)	/* in mAh, typical capacity for entire pack   */
#define PACK_DESIGNCAPC5  (CELL_DESIGNCAPC5 * PACKWIDTH)	/* in mAh, minimum capacity at C/5 for entire pack */

#define PACK_NOMINALV     (CELL_NOMINALV * PACKSTACK)		/* in mV, typical, entire pack V when fully charged */
#define PACK_MINV         (CELL_MINV * PACKSTACK)		/* in mV, absolute low-end voltage permitted  */
#define PACK_DISCHG_C5    (CELL_DISCHG_C5 * PACKWIDTH)		/* discharge current (mA) for C/5 rate        */
#define PACK_DISCHG_MAX   (CELL_DISCHG_MAX * PACKWIDTH)		/* maximum permissible discharge current (mA) */

#define PACK_CCCV_V       (CELL_CCCV_V * PACKSTACK)		/* Constant Current / Constant Voltage charging mode, applied voltage (mV) */
#define PACK_CCCV_C       (CELL_CCCV_C * PACKWIDTH)		/* Constant Current / Constant Voltage charging mode, applied current (mA) */
#define PACK_CCCV_TERMC   (CELL_CCCV_TERMC * PACKWIDTH) /* Constant Current / Constant Voltage charging mode, charge-termination current threshold (mA) */
#define PACK_MAX_CHG_C    (CELL_MAX_CHG_C * PACKWIDTH)	/* Maximum permissible charging current (mA)  */


/* Extra calculations we perform on Constants to make code lighter... */
#define PACK_DESIGNCAPMW  ((PACK_DESIGNCAPTYP/100) * (PACK_NOMINALV/100))
#define PACK_PROTECT_DUV  {BPDUV = (0x01<<4) | (0x0D << 0);}	/* 1000mS, 8.80V*90% due to h/w issue */
#define PACK_PROTECT_SHORT {BPSCD = 0;}				/* 20A into 5mOhm = 100mV */
#define PACK_PROTECT_OVERC {BPOCD = 0<<4 | 0<<0;}		/* 10A discharge, 10A charge */
#define PACK_PROTECT_TIME  {CBPTR = 1<<4 | 5<<0;}		/* 122uS short-circuit, 10mS overcurrent Chg/Dischg */


#define PACK_HIBERNATEAMOUNT 1					/* used to update RunningAcc while in Hibernate mode; see WakeUp_ISR().  */

/*
1.1.4	Battery protection detection levels
The characterization of the Deep Under-voltage, Over-current and Short-circuit detection levels
show that the actual thresholds are typically 10% below the numbers given in the datasheet. This
is caused by parasitic elements in the layout that are not included in the simulation models.
The spread in these thresholds are however in the expected range of +/-5% for Over-current
detection levels, and +/-3% for Deep Under-voltage detection levels.

Detailed characterization data will be given in the Analog characterization report.

Workaround for ATmega406 rev. D:
As long as the actual range for these detection levels covers the required range for the
application used for qualification of ATmega406 rev. D, a correct detection level can always be found.

ATmega406 fix:
We assume that it is sufficient to update the datasheet with the actual detection levels found
during characterization. If not, the thresholds can be tuned to better fit the specification in the datasheet.

Battery Protection Parameter Lock Register – BPPLR
Battery Protection Control Register – BPCR
Current Battery Protection Timing Register – CBPTR
Battery Protection Over-current Detection Level Register – BPOCD
Battery Protection Short-circuit Detection Level Register – BPSCD
Battery Protection Deep Under Voltage Register – BPDUV
Battery Protection Interrupt Register – BPIR

*/



