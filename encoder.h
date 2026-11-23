/**
 * =================================================================================================
 * @file encoder.h
 * @brief Public API & Mathematical Model for Quadrature Encoder Decoding and Speed Estimation
 * @target Microchip AVR128DA48
 *
 * =================================================================================================
 * HARDWARE SENSOR SPECIFICATIONS:
 * =================================================================================================
 *  - Sensor Type     : Dual-channel Hall Effect Quadrature Encoder (FIT0521 Gearmotor)
 *  - Base Resolution : 11 Pulses Per Revolution (PPR) on the motor armature shaft
 *  - Gearbox Ratio   : 34.02:1 metal planetary/spur gear reduction
 *  - Pin Routing     : Channel A -> PB0 (IO_PB0 with internal pull-up)
 *                      Channel B -> PB1 (gpio with internal pull-up)
 *
 * =================================================================================================
 * 4X GRAY CODE DECODING & FIXED-POINT SCALING:
 * =================================================================================================
 * By tracking all rising and falling edges across both Channel A and Channel B (4X decoding):
 *   Edges per revolution = [Hall PPR] * [Gear Ratio] * [4 Edges/Cycle]
 *                        = 11 * 34.02 * 4
 *                        = 1496.88 counts per output shaft revolution.
 *
 * To avoid floating-point arithmetic overhead on an 8-bit AVR core, the count constant is scaled
 * by 10 (1496.88 * 10 = 14969) for integer arithmetic:
 *   #define ENCODER_TICKS_PER_REV  14969
 * =================================================================================================
 */

#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>

/* =================================================================================================
 * ENCODER CONFIGURATION & CONSTANTS
 * ================================================================================================= */

/**
 * @def ENCODER_TICKS_PER_REV
 * @brief Total encoder counts per output shaft revolution scaled by 10 (fixed-point integer math).
 * @details 11 PPR * 34.02 gear ratio * 4 edges = 1496.88 -> Scaled * 10 = 14969.
 */
#define ENCODER_TICKS_PER_REV  14969

/**
 * @def RPM_WINDOW_SIZE
 * @brief Rolling history buffer depth for moving-average rotational speed estimation.
 */
#define RPM_WINDOW_SIZE        10

/* =================================================================================================
 * PUBLIC API FUNCTIONS
 * ================================================================================================= */

/**
 * @brief Initializes PB0 and PB1 as digital inputs with pull-ups and configures dual-edge interrupts.
 * @details Reads the initial state of both pins to seed the Gray code state machine.
 */
void Encoder_Init(void);

/**
 * @brief Computes rotational speed (in RPM) using a rolling window of recent encoder position samples.
 * @return Signed rotational speed in Revolutions Per Minute (positive = forward, negative = reverse).
 */
int32_t Encoder_ComputeRPM(void);

/**
 * @brief Atomically reads the accumulated 32-bit encoder position tick counter.
 * @return Total signed ticks since initialization.
 */
int32_t Encoder_GetTicks(void);

#endif /* ENCODER_H */
