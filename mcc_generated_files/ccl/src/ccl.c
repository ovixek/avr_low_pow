/**
 * =================================================================================================
 * @file ccl.c
 * @brief Configurable Custom Logic (CCL) Hardware Driver Implementation
 * @ingroup ccl
 * @version CCL Driver Version 1.1.1
 *
 * =================================================================================================
 * ARCHITECTURAL ROLE & RUNTIME INITIALIZATION:
 * =================================================================================================
 * The CCL is an array of on-chip programmable Look-Up Tables (LUTs) that execute hardware logic
 * functions without CPU intervention.
 *
 * NOTE ON SYSTEM OPERATION:
 *  - This file contains the MCC-generated baseline CCL initialization function CCL_Initialize().
 *  - During firmware startup in main.c, the application calls CCL_FaultLatch_Init() which
 *    explicitly reconfigures all LUT registers to establish the multi-stage motor safety pipeline:
 *      * LUT0: Glitch-filtered buffer of AC0 overcurrent output (CLK_PER digital filter)
 *      * LUT1: Active-high 2-input OR gate combining PC0 and PC1 limit switches
 *      * LUT2/LUT3: Hardware RS Fault Latch (SET on stall or limit switch, RESET on software event)
 * =================================================================================================
 */

#include "../ccl.h"

/**
 * @brief Initializes the baseline CCL configuration.
 * @return 0 on success.
 */
int8_t CCL_Initialize(void)
{
    /* Configure Sequencer 0 as RS Latch */
    CCL.SEQCTRL0 = CCL_SEQSEL_RS_gc; /* RS Latch */
    
    /* LUT0 Truth Table: Output = IN0 (Buffer mode) */
    CCL.TRUTH0 = 2; /* Truth 0: 2 */
    
    /* LUT0 Input Selection: IN0 = AC0 output, IN1 = Masked */
    CCL.LUT0CTRLB = CCL_INSEL0_AC0_gc     /* AC0 OUT input source */
	                | CCL_INSEL1_MASK_gc; /* Masked input */
    
    /* LUT0 Control A: Clocked by CLK_PER, edge detector disabled, LUT enabled, output pin enabled */
    CCL.LUT0CTRLA = CCL_CLKSRC_CLKPER_gc     /* Peripheral Clock */
	                | CCL_EDGEDET_DIS_gc     /* Edge detector is disabled */
	                | CCL_FILTSEL_DISABLE_gc /* Filter disabled */
	                | 1 << CCL_ENABLE_bp     /* LUT Enable: enabled */
	                | 1 << CCL_OUTEN_bp;     /* Output Enable: enabled */
    
    /* LUT1 Truth Table: Output = IN0 */
    CCL.TRUTH1 = 2; /* Truth 1: 2 */
    
    /* LUT1 Input Selection: IN0 = Event Input A */
    CCL.LUT1CTRLB = CCL_INSEL0_EVENTA_gc  /* Event input source A */
	                | CCL_INSEL1_MASK_gc; /* Masked input */

    /* LUT1 Control A: Clocked by CLK_PER, LUT enabled, output pin disabled (internal routing) */
	CCL.LUT1CTRLA = CCL_CLKSRC_CLKPER_gc     /* Peripheral Clock */
	                | CCL_FILTSEL_DISABLE_gc /* Filter disabled */
	                | 1 << CCL_ENABLE_bp     /* LUT Enable: enabled */
	                | 0 << CCL_OUTEN_bp;     /* Output Enable: disabled */

    /* Master CCL Enable */
    CCL.CTRLA = 1 << CCL_ENABLE_bp      /* Enable: enabled */
	            | 0 << CCL_RUNSTDBY_bp; /* Run in Standby: disabled */

   return 0;    
}
