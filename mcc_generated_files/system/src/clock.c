/**
 * =================================================================================================
 * @file clock.c
 * @brief Clock Controller (CLKCTRL) Initialization: 24 MHz Main System Clock & 48 MHz PLL
 * @ingroup clkctrl
 * @version CLKCTRL Driver Version 1.0.2
 *
 * =================================================================================================
 * CLOCK TREE ARCHITECTURE & DESIGN RATIONALE:
 * =================================================================================================
 *  1. MAIN CLOCK (CLK_MAIN / CLK_PER = 24 MHz):
 *     - Source: Internal High-Frequency Oscillator (OSCHF) set to 24 MHz.
 *     - Prescaler (PEN): Disabled (Main clock prescaler divisor = 1).
 *     - RATIONALE: 24 MHz is the maximum calibrated internal frequency for the AVR-DA family.
 *       Running at 24 MHz ensures highest CPU execution throughput, minimum timer quantization,
 *       and fastest event routing response times across all Core-Independent Peripherals.
 *
 *  2. PHASE-LOCKED LOOP (PLL = 48 MHz for TCD0):
 *     - Source: 24 MHz OSCHF multiplied by 2X (CLKCTRL_MULFAC_2x_gc).
 *     - Target: Used exclusively by Timer D (TCD0) in asynchronous clock mode.
 *     - RATIONALE: The 48 MHz PLL clock allows TCD0 to generate high-frequency 64.2 kHz dual-slope
 *       PWM with 748-step duty cycle resolution, placing switching noise well above human hearing.
 * =================================================================================================
 */

#include "../clock.h"

/**
 * @brief Configures OSCHF (24 MHz), Main Prescaler (PEN=0), and PLL (48 MHz).
 * @details Uses Configuration Change Protection (ccp_write_io) for protected clock registers.
 */
void CLOCK_Initialize(void)
{    
    /* MCLKCTRLA: Select OSCHF as main system clock source; disable external CLKOUT */
    ccp_write_io((void*)&(CLKCTRL.MCLKCTRLA),(0 << CLKCTRL_CLKOUT_bp)   // CLKOUT disabled
            | CLKCTRL_CLKSEL_OSCHF_gc   // CLKSEL Internal high-frequency oscillator
            );

    /* MCLKCTRLB: Main prescaler disabled (PEN = 0) -> CLK_PER = 24 MHz full speed */
    ccp_write_io((void*)&(CLKCTRL.MCLKCTRLB),CLKCTRL_PDIV_2X_gc   // PDIV 2X
            | (0 << CLKCTRL_PEN_bp)   // PEN disabled
            );

    /* MCLKLOCK: Clock configuration lock disabled */
    ccp_write_io((void*)&(CLKCTRL.MCLKLOCK),(0 << CLKCTRL_LOCKEN_bp)   // LOCKEN disabled
            );

    /* MCLKSTATUS: Verify clock oscillator stability flags */
    ccp_write_io((void*)&(CLKCTRL.MCLKSTATUS),(0 << CLKCTRL_EXTS_bp)   // EXTS disabled
            | (1 << CLKCTRL_OSCHFS_bp)   // OSCHFS enabled
            | (0 << CLKCTRL_OSC32KS_bp)   // OSC32KS disabled
            | (1 << CLKCTRL_PLLS_bp)   // PLLS enabled
            | (0 << CLKCTRL_SOSC_bp)   // SOSC disabled
            | (0 << CLKCTRL_XOSC32KS_bp)   // XOSC32KS disabled
            );

    /* OSCHFCTRLA: Set internal high-frequency oscillator to 24 MHz */
    ccp_write_io((void*)&(CLKCTRL.OSCHFCTRLA),(0 << CLKCTRL_AUTOTUNE_bp)   // AUTOTUNE disabled
            | CLKCTRL_FRQSEL_24M_gc   // FRQSEL 24 MHz system clock (default)
            | (0 << CLKCTRL_RUNSTDBY_bp)   // RUNSTDBY disabled
            );

    /* OSCHFTUNE: Factory calibration tuning offset = 0 */
    ccp_write_io((void*)&(CLKCTRL.OSCHFTUNE),0x0   // TUNE 0x0
            );

    /* OSC32KCTRLA: Internal 32 kHz oscillator standby mode disabled */
    ccp_write_io((void*)&(CLKCTRL.OSC32KCTRLA),(0 << CLKCTRL_RUNSTDBY_bp)   // RUNSTDBY disabled
            );

    /* PLLCTRLA: Multiply 24 MHz OSCHF by 2X to generate 48 MHz asynchronous clock for TCD0 */
    ccp_write_io((void*)&(CLKCTRL.PLLCTRLA),CLKCTRL_MULFAC_2x_gc   // MULFAC PLL 2X multiplier
            | (0 << CLKCTRL_RUNSTDBY_bp)   // RUNSTDBY disabled
            | (0 << CLKCTRL_SOURCE_bp)   // SOURCE disabled
            );

    /* XOSC32KCTRLA: External 32 kHz crystal oscillator disabled */
    ccp_write_io((void*)&(CLKCTRL.XOSC32KCTRLA),CLKCTRL_CSUT_1K_gc   // CSUT 1k cycles
            | (0 << CLKCTRL_ENABLE_bp)   // ENABLE disabled
            | (0 << CLKCTRL_LPMODE_bp)   // LPMODE disabled
            | (0 << CLKCTRL_RUNSTDBY_bp)   // RUNSTDBY disabled
            | (0 << CLKCTRL_SEL_bp)   // SEL disabled
            );
}
