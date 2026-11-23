/**
 * =================================================================================================
 * @file dac0.c
 * @brief 10-bit Digital-to-Analog Converter (DAC0) Baseline Driver
 * @ingroup dac
 * @version DAC Driver Version 1.0.0
 *
 * =================================================================================================
 * ARCHITECTURAL ROLE & USAGE NOTE:
 * =================================================================================================
 * DAC0 provides a programmable internal analog voltage level.
 *
 * NOTE ON COMPARATOR OVERCURRENT TRIP POINT:
 *  - In this system architecture, AC0 utilizes its dedicated internal 8-bit DACREF register
 *    (directly calibrated and updated via AC0.DACREF in main.c c_calibration).
 *  - DAC0 is initialized here as a baseline subsystem with output pin routing (OUTEN) disabled,
 *    preventing external pin contention on Port D.
 * =================================================================================================
 */

#include "../dac0.h"

/**
 * @brief Initializes DAC0 with zero output and external pin drive disabled.
 * @return 0 on success.
 */
int8_t DAC0_Initialize(void)
{
    /* DATA Register: Initial 10-bit value = 0 */
	DAC0.DATA = 0x0;

    /* Control Register A:
     * - ENABLE = 1 : DAC core is enabled
     * - OUTEN  = 0 : External pin output disabled (used exclusively for internal references)
     * - RUNSTDBY = 0 : Standby operation disabled */
	DAC0.CTRLA = 0x1;

    return 0;
}

void DAC0_Enable(void)
{
    DAC0.CTRLA |= DAC_ENABLE_bm;
}

void DAC0_Disable(void)
{
    DAC0.CTRLA &= ~DAC_ENABLE_bm;
}

void DAC0_SetOutput(dac_resolution_t value)
{
    value     = value << DAC_DATA_gp;
	value     = value & 0xFFC0;
    DAC0.DATA = value;
}

uint8_t DAC0_GetResolution(void)
{
    return 10;
}
