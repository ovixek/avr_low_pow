/**
 * =================================================================================================
 * @file ac0.c
 * @brief Analog Comparator 0 (AC0) Driver for Ultra-Fast Hardware Overcurrent Sensing
 * @ingroup ac0
 * @version AC0 Driver Version 1.1.0
 *
 * =================================================================================================
 * ARCHITECTURAL ROLE & HARDWARE RATIONALE:
 * =================================================================================================
 * AC0 is the primary analog frontend for motor overcurrent and stall detection.
 *
 * WHY AC0 INSTEAD OF ADC?
 *  - Speed: AC0 makes continuous real-time threshold decisions in <100 nanoseconds. An ADC requires
 *           several microseconds for sampling, conversion, and interrupt servicing.
 *  - Core Independence: AC0's output connects directly to CCL LUT0 via internal silicon routing,
 *           completely bypassing the CPU and memory bus during motor operation.
 *  - Power Profile 0: Provides the fastest possible comparator propagation delay.
 *  - Medium Hysteresis (~25 mV): Rejects motor brush commutation hash and PWM ripple, preventing
 *           erratic edge chatter at the threshold boundary.
 * =================================================================================================
 */

#include <util/atomic.h>
#include "../ac0.h"

static ac_cb_t AC0_cb = NULL;

/**
 * @brief Initializes Analog Comparator 0 (AC0).
 * @return Initialization status (0 = success).
 */
int8_t AC0_Initialize(void)
{
    /* Window Mode Disabled: Single-ended threshold comparison against DACREF */
    AC0.CTRLB = (uint8_t)0x0U;

    /* Initial DACREF = 250 (~4.0V default, overwritten during runtime calibration) */
    AC0.DACREF = (uint8_t)0xFAU;
    
    /* Interrupts disabled initially; armed in main.c after motor calibration */
    AC0.INTCTRL = (uint8_t)0x0U;
    
    /* Input Multiplexer Configuration:
     * - MUXPOS = 0 (Positive Pin 0 / PD2 - motor current sense amplifier output)
     * - MUXNEG = 3 (Internal DAC Reference DACREF, driven by 4.096V VREF)
     * - INVERT  = 0 (Normal non-inverted polarity: Output = HIGH when I_motor > Threshold) */
    AC0.MUXCTRL = (uint8_t)0x3U;

    /* Control Register A (0xC5):
     * - ENABLE  = 1 : Comparator core is active
     * - HYSMODE = 1 : Medium hysteresis (~25 mV) to reject commutation noise
     * - OUTEN   = 1 : Output routed to PA7 for physical oscilloscope diagnostics
     * - POWER   = 0 : Power Profile 0 (fastest response time <100 ns)
     * - RUNSTDBY= 1 : Comparator remains active in standby power modes */
    AC0.CTRLA = (uint8_t)0xC5U;

    return 0;
}

void AC0_MuxSet(uint8_t mode)
{
    uint8_t temp;
    temp = AC0.MUXCTRL;
    temp &= (uint8_t)(~((uint8_t)AC_MUXPOS_gm | (uint8_t)AC_MUXNEG_gm));
    temp |= mode;
    AC0.MUXCTRL = temp;
}

void AC0_Invert(bool config)
{     
    if (true == config)
    {
        AC0.MUXCTRL |=  (uint8_t)AC_INVERT_bm;
    }
    else
    {
        AC0.MUXCTRL &= (uint8_t)(~AC_INVERT_bm);
    }
}

bool AC0_Read(void)
{
    uint8_t nullValue = 0U;
    return ((AC0.STATUS & (uint8_t)AC_CMPSTATE_bm) != nullValue);
}

void AC0_CallbackRegister(ac_cb_t comparator_cb)
{
    AC0_cb = comparator_cb;
}

void AC0_DACRefValueSet (uint8_t value)
{ 
    AC0.DACREF = value;
}
