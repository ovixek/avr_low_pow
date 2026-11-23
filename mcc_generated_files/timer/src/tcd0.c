/**
 * =================================================================================================
 * @file tcd0.c
 * @brief 12-bit Asynchronous Timer D (TCD0) Driver for Motor PWM Generation & Hardware Fault Shutoff
 * @ingroup tcd0
 * @version TCD0 Driver Version 2.1.0
 *
 * =================================================================================================
 * ARCHITECTURAL ROLE & HARDWARE INTEGRATION:
 * =================================================================================================
 * TCD0 is a specialized 12-bit timer designed for high-resolution power conversion and motor drive.
 *
 * WHY TCD0 IS USED INSTEAD OF TCA0 / TCB FOR MOTOR PWM:
 *  1. ASYNCHRONOUS HARDWARE FAULT INPUT:
 *     TCD0 features dedicated event-triggered fault inputs (EVCTRLA). When asserted by the CCL_LUT2
 *     RS latch via EVSYS CH4, TCD0 hardware INSTANTLY (within <25 ns) overrides the compare outputs
 *     and forces PA5 (WOB / Enable) LOW, physically halting H-bridge current without CPU involvement.
 *  2. INDEPENDENT CLOCK DOMAIN (48 MHz PLL):
 *     TCD0 runs from the internal Phase-Locked Loop (PLL) at 48 MHz, asynchronous to the 24 MHz
 *     system clock (CLK_PER). This enables higher PWM resolution and immunity to CPU clock jitter.
 *  3. CONFIGURABLE FAULT POLARITY (FAULTCTRL):
 *     Under CCP protection, FAULTCTRL forces PA5 (EN) to GND while releasing PA4 (PH) for independent
 *     GPIO direction control.
 * =================================================================================================
 */

#include "../tcd0.h"
#include "../../system/ccp.h"

/**
 * @brief Initializes TCD0 in Dual-Slope PWM mode with hardware fault protection.
 * @details Configures period, duty cycle, asynchronous fault input, and PLL clocking.
 */
void TCD0_Initialize(void)
{
    /* Clear B (Dual-Slope Period):
     * Triangle wave counts 0 -> CMPBCLR -> 0.
     * Period ticks = 2 * (373 + 1) = 748 ticks.
     * At 48 MHz PLL: f_PWM = 48,000,000 / 748 = ~64.17 kHz.
     * WHY DUAL-SLOPE?
     * Symmetrical dual-slope PWM produces lower acoustic noise and harmonic distortion in DC motors. */
    TCD0.CMPBCLR = 373;

    /* SET B (Initial WOB / PA5 PWM Compare Value):
     * Compare threshold where WOB transitions during dual-slope counting.
     * Duty cycle = (373 - 93) / 373 = ~75.0% motor speed. */
    TCD0.CMPBSET =93;

    /* Dual Slope Waveform Generation Mode:
     * Up/down counting generates center-aligned PWM pulses. */
    TCD0.CTRLB = TCD_WGMODE_DS_gc;

    /* Fault Input A:
     * - CFG = ASYNC: Fault detection is asynchronous, bypassing timer clock synchronizers.
     * - ACTION = FAULT: Event directly triggers hardware fault state.
     * - EDGE = RISE_HIGH: Triggers on rising edge / high level from CCL_LUT2 RS latch (EVSYS CH4).
     * - TRIGEI = 1: Enables event input A. */
    TCD0.EVCTRLA = TCD_CFG_ASYNC_gc
                 | TCD_ACTION_FAULT_gc
                 | TCD_EDGE_RISE_HIGH_gc
                 | 1 << TCD_TRIGEI_bp;

    /* Disable external pin input mode; TCD responds exclusively to EVSYS event inputs */
    TCD0.INPUTCTRLA = TCD_INPUTMODE_NONE_gc;
    TCD0.INPUTCTRLB = TCD_INPUTMODE_NONE_gc;

    /* Fault Control Register (Protected by Configuration Change Protection - CCP):
     * - CMPAEN = 0 : WOA (PA4) is disabled from TCD0 control, leaving PA4 free for GPIO Phase/Direction.
     * - CMPBEN = 1 : WOB (PA5) is enabled as TCD0 PWM Enable output.
     * - CMPB = 0   : When a fault occurs, PA5 is forced LOW (0V / GND), instantly cutting motor drive. */
    ccp_write_io((void *)&(TCD0.FAULTCTRL),
                   0 << TCD_CMPAEN_bp       /* Compare A enable: disabled (PA4 = GPIO PH) */
                 | 0 << TCD_CMPA_bp         /* Compare A value: disabled */
                 | 0 << TCD_CMPB_bp         /* Compare B value: 0 (PA5 forced LOW on fault) */
                 | 1 << TCD_CMPBEN_bp       /* Compare B enable: enabled (PA5 = PWM EN) */
                 | 0 << TCD_CMPC_bp         /* Compare C value: disabled */
                 | 0 << TCD_CMPCEN_bp       /* Compare C enable: disabled */
                 | 0 << TCD_CMPD_bp         /* Compare D value: disabled */
                 | 0 << TCD_CMPDEN_bp       /* Compare D enable: disabled */);

    /* Input Mode: Stop outputs on Fault Input A, enter wait-for-software state.
     * Motor remains safely locked OFF until explicitly cleared by user button press in main.c. */
    TCD0.INPUTCTRLA = TCD_INPUTMODE_WAITSW_gc; /* Stop outputs on Fault Input A, wait for software restart */

    /* Clock Configuration:
     * - CLKSEL = PLL: Clocked by internal 48 MHz Phase-Locked Loop.
     * - CNTPRES = DIV1, SYNCPRES = DIV1: No clock division for maximum PWM resolution.
     * - ENABLE = 0: Disabled initially; armed cleanly in main.c tcd_enable() after calibration. */
    TCD0.CTRLA = 0 << TCD_ENABLE_bp
                 | TCD_CLKSEL_PLL_gc
                 | TCD_CNTPRES_DIV1_gc
                 | TCD_SYNCPRES_DIV1_gc;
}
