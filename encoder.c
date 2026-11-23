/**
 * =================================================================================================
 * @file encoder.c
 * @brief Quadrature Encoder State Machine & Speed Measurement Implementation
 * @target Microchip AVR128DA48
 *
 * =================================================================================================
 * HARDWARE INTERFACE:
 * =================================================================================================
 *  - Channel A : PB0 (Configured via PIN_MANAGER as IO_PB0 with internal pull-up)
 *  - Channel B : PB1 (Configured via PIN_MANAGER as gpio with internal pull-up)
 *
 * =================================================================================================
 * 4X GRAY CODE DECODING STATE MACHINE:
 * =================================================================================================
 * The two quadrature channels form a 2-bit Gray code: State = (ChA << 1) | ChB.
 *
 * Valid State Transitions:
 *  - Forward (+1):  00 (0) -> 01 (1) -> 11 (3) -> 10 (2) -> 00 (0)
 *  - Reverse (-1):  00 (0) -> 10 (2) -> 11 (3) -> 01 (1) -> 00 (0)
 *  - No Move ( 0):  Current state equals previous state (contact bounce / noise)
 *  - Invalid ( 0):  Double-bit transitions (00 <-> 11 or 01 <-> 10) are physically impossible
 *                   at normal speeds and indicate severe noise or missed pulses; ignored.
 * =================================================================================================
 */

#include "encoder.h"
#include "mcc_generated_files/system/pins.h"
#include <avr/interrupt.h>

/* Encoder tick counter updated atomically by ISR */
static volatile int32_t encoder_ticks = 0;

/* Previous Gray code state persists between ISR calls */
static volatile uint8_t encoder_prev_state = 0;

/* Rolling window history buffer for RPM  */
static int32_t tick_history[RPM_WINDOW_SIZE] = {0};
static uint8_t history_idx = 0;


static void Encoder_ISR_Callback(void)
{
    /* Read current state of both channels */
    uint8_t chA = IO_PB0_GetValue() ? 1U : 0U;
    uint8_t chB = gpio_GetValue()   ? 1U : 0U;
    uint8_t curr_state = (uint8_t)((chA << 1) | chB);

    /* Determine direction based on state transition */
    switch (encoder_prev_state) {

        case 0:  /* Prev A=0, B=0 */
            switch (curr_state) {
                case 1: encoder_ticks++; break;  /* 0 to 1 Forward  */
                case 2: encoder_ticks--; break;  /* 0 to 2 Reverse  */
                default: break;                  /* 0 to 0 No move  */
                                                 /* 0 to 3 Invalid  */
            }
            break;

        case 1:  /* Prev A=0, B=1 */
            switch (curr_state) {
                case 3: encoder_ticks++; break;  /*  1 to 3 Forward  */
                case 0: encoder_ticks--; break;  /*  1 to 0 Reverse  */
                default: break;                  /*  1 to 1 No move  */
                                                 /*  1 to 2 Invalid  */
            }
            break;

        case 2:  /* Prev A=1, B=0 */
            switch (curr_state) {
                case 0: encoder_ticks++; break;  /*  2 to 0 Forward  */
                case 3: encoder_ticks--; break;  /*  2 to 3 Reverse  */
                default: break;                  /* 2 to 2 No move  */
                                                 /* 2 to 1 Invalid  */
            }
            break;

        case 3:  /* Prev A=1, B=1 */
            switch (curr_state) {
                case 2: encoder_ticks++; break;  /*  3 to 2 Forward  */
                case 1: encoder_ticks--; break;  /*  3 to 1 Reverse  */
                default: break;                  /*  3 to 3 No move  */
                                                 /*  3 to 0 Invalid  */
            }
            break;
        default: break;
    }

    /* Save current state for next ISR call */
    encoder_prev_state = curr_state;
}
/*-------------------------------------------------------------
 * PUBLIC FUNCTIONS API
 *-------------------------------------------------------------*/

void Encoder_Init(void)
{
    IO_PB0_SetPullUp();
   gpio_SetDigitalInput();
    gpio_SetPullUp();

    {
        uint8_t a = IO_PB0_GetValue() ? 1U : 0U;
        uint8_t b = gpio_GetValue()   ? 1U : 0U;
        encoder_prev_state = (uint8_t)((a << 1) | b);
    }

    /* Register the same ISR callback for both channels */
    IO_PB0_SetInterruptHandler(Encoder_ISR_Callback);   /* Ch A */ //PB0
    gpio_SetInterruptHandler(Encoder_ISR_Callback);      /* Ch B */ //PB1

    /* Enable both-edge interrupts on both channels*/
     IO_PB0_EnableInterruptForBothEdges();
     gpio_EnableInterruptForBothEdges();
}

int32_t Encoder_ComputeRPM(void)
{
    uint8_t sreg_save = SREG;
    cli();
    int32_t current_ticks = encoder_ticks;
    SREG = sreg_save;

    /*Rolling window */
    int32_t old_ticks = tick_history[history_idx];
    tick_history[history_idx] = current_ticks;
    history_idx = (history_idx + 1) % RPM_WINDOW_SIZE;

    /* RPM calculation:
     * RPM = (ticks_in_window / ticks_per_rev) x 600
     *   = (ticks_in_window x 600) / 1496.88
     */
    int32_t ticks_in_window = current_ticks - old_ticks;
    return (ticks_in_window * 6000) / ENCODER_TICKS_PER_REV;
}

int32_t Encoder_GetTicks(void)
{
    /* Atomic read */
    uint8_t sreg_save = SREG;
    cli();
    int32_t ticks = encoder_ticks;
    SREG = sreg_save;
    return ticks;
}
