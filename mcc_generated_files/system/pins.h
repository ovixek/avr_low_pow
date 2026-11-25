/**
 * ============================================================================
 * @file pins.h
 * @brief Pin Manager and GPIO Definitions for AVR128DA48
 * ============================================================================
 */

#ifndef PINS_H_INCLUDED
#define PINS_H_INCLUDED

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * ============================================================================
 * Onboard Status LED (PC6) on Curiosity Nano (Active-Low)
 * ============================================================================
 */
#define LED_ON()                         do { PORTC.OUTCLR = PIN6_bm; } while(0)
#define LED_OFF()                        do { PORTC.OUTSET = PIN6_bm; } while(0)
#define LED_TOGGLE()                     do { PORTC.OUTTGL = PIN6_bm; } while(0)

#define IO_PC6_SetHigh()                 do { PORTC.OUTSET = PIN6_bm; } while(0)
#define IO_PC6_SetLow()                  do { PORTC.OUTCLR = PIN6_bm; } while(0)
#define IO_PC6_Toggle()                  do { PORTC.OUTTGL = PIN6_bm; } while(0)
#define IO_PC6_GetValue()                (VPORTC.IN & PIN6_bm)
#define IO_PC6_SetDigitalInput()         do { PORTC.DIRCLR = PIN6_bm; } while(0)
#define IO_PC6_SetDigitalOutput()        do { PORTC.DIRSET = PIN6_bm; } while(0)

/*
 * ============================================================================
 * PA2 Sleep & Wakeup Observation Pin
 * High = In Power-Down Sleep / Powering Down
 * Low  = Awake & Operating
 * ============================================================================
 */
#define SLEEP_OBSERVE_PIN_bm             PIN2_bm
#define SLEEP_OBSERVE_HIGH()             do { PORTA.OUTSET = PIN2_bm; } while(0)
#define SLEEP_OBSERVE_LOW()              do { PORTA.OUTCLR = PIN2_bm; } while(0)
#define SLEEP_OBSERVE_SetDigitalOutput() do { PORTA.DIRSET = PIN2_bm; } while(0)

/*
 * ============================================================================
 * PA7 AC0 Hardware Output Pin
 * Live comparator state: HIGH = Overcurrent (PD2 > DACREF), LOW = Normal (PD2 < DACREF)
 * ============================================================================
 */
#define AC0_OUT_PIN_bm                   PIN7_bm

/*
 * ============================================================================
 * USART1 Serial Communication Pins (ALT1 Pin Routing)
 * PC4: TXD (Digital Output, Idle HIGH)
 * PC5: RXD (Digital Input with internal pull-up)
 * Note: PC0 and PC1 are completely removed from UART and reserved for limit switches.
 * ============================================================================
 */
#define USART1_TXD_PIN_bm                PIN4_bm
#define USART1_RXD_PIN_bm                PIN5_bm

/*
 * ============================================================================
 * Limit Switch Input Pins
 * PC0: Limit Switch 1 (Digital Input -> EVSYS CH2 -> CCL LUT1 Event Input A)
 * PC1: Limit Switch 2 (Digital Input -> EVSYS CH3 -> CCL LUT1 Event Input B)
 * ============================================================================
 */
#define LIMIT_SW1_PIN_bm                 PIN0_bm
#define LIMIT_SW2_PIN_bm                 PIN1_bm

/*
 * ============================================================================
 * PB2 Asynchronous Wake-Up Pin
 * ============================================================================
 */
#define IO_PB2_GetValue()                (VPORTB.IN & PIN2_bm)
#define IO_PB2_SetDigitalInput()         do { PORTB.DIRCLR = PIN2_bm; } while(0)
#define IO_PB2_SetDigitalOutput()        do { PORTB.DIRSET = PIN2_bm; } while(0)
#define IO_PB2_SetPullUp()               do { PORTB.PIN2CTRL |= PORT_PULLUPEN_bm; } while(0)
#define IO_PB2_ResetPullUp()             do { PORTB.PIN2CTRL &= ~PORT_PULLUPEN_bm; } while(0)
#define IO_PB2_EnableFallingEdge()       do { PORTB.PIN2CTRL = (PORTB.PIN2CTRL & ~PORT_ISC_gm) | PORT_ISC_FALLING_gc; } while(0)
#define IO_PB2_EnableBothEdges()         do { PORTB.PIN2CTRL = (PORTB.PIN2CTRL & ~PORT_ISC_gm) | PORT_ISC_BOTHEDGES_gc; } while(0)

/*
 * ============================================================================
 * Motor Control Pins (PA4: PH, PA5: EN)
 * ============================================================================
 */
#define MOTOR_PH_PIN_bm                  PIN4_bm
#define MOTOR_EN_PIN_bm                  PIN5_bm

#define IO_PA4_SetHigh()                 do { PORTA.OUTSET = PIN4_bm; } while(0)
#define IO_PA4_SetLow()                  do { PORTA.OUTCLR = PIN4_bm; } while(0)
#define IO_PA5_SetHigh()                 do { PORTA.OUTSET = PIN5_bm; } while(0)
#define IO_PA5_SetLow()                  do { PORTA.OUTCLR = PIN5_bm; } while(0)

/*
 * ============================================================================
 * Function Prototypes
 * ============================================================================
 */
void PIN_MANAGER_Initialize(void);
void IO_PB2_SetInterruptHandler(void (* interruptHandler)(void));
void IO_PD2_SetInterruptHandler(void (* interruptHandler)(void));
void IO_PA7_SetInterruptHandler(void (* interruptHandler)(void));
void IO_PA4_SetInterruptHandler(void (* interruptHandler)(void));
void IO_PC6_SetInterruptHandler(void (* interruptHandler)(void));

#ifdef __cplusplus
}
#endif

#endif /* PINS_H_INCLUDED */