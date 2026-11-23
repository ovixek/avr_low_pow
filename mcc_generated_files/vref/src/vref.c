/**
 * =================================================================================================
 * @file vref.c
 * @brief Voltage Reference (VREF) Driver for Analog Comparator & DAC Subsystems
 * @ingroup vref
 * @version VREF Driver Version 1.0.1
 *
 * =================================================================================================
 * ARCHITECTURAL ROLE & HARDWARE RATIONALE:
 * =================================================================================================
 * The VREF peripheral provides stable, internal, temperature-compensated bandgap reference voltages.
 *
 * WHY 4.096V IS CHOSEN FOR ACREF:
 *  1. EXACT INTEGER SCALING (16 mV / LSB):
 *     The 8-bit DACREF register has 256 discrete levels: 4096 mV / 256 = exactly 16.000 mV per LSB.
 *     This allows threshold calculations in main.c (c_calculate_trigger_voltage) to execute
 *     using integer shifts and multiplies without floating-point math libraries.
 *  2. DYNAMIC RANGE COVERAGE:
 *     Motor current sense amplifiers typically output signals between 0.2V (quiescent zero-load)
 *     and 3.3V (hard stall). A 4.096V reference comfortably covers the entire range without clipping.
 *  3. ALWAYSON ENABLED:
 *     Ensures the bandgap circuit remains continuously powered and thermally settled, preventing
 *     the ~20 microsecond wake-up settling glitch from causing false overcurrent comparator trips.
 * =================================================================================================
 */

#include "../vref.h"

/**
 * @brief Initializes the internal voltage reference for the Analog Comparator subsystem.
 * @return 0 on success.
 */
int8_t VREF_Initialize(void)
{
    /* Configure Analog Comparator Voltage Reference (ACREF):
     * - VREF_ALWAYSON_bp = 1 : Keep reference generator always active (zero startup delay)
     * - VREF_REFSEL_4V096_gc : Select 4.096V internal bandgap reference */
    VREF.ACREF = 1 << VREF_ALWAYSON_bp   /* Enable reference always ON for AC: enabled */
	             | VREF_REFSEL_4V096_gc; /* Internal 4.096V reference */

	return 0;
}
