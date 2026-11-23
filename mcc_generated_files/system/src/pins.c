/**
 * =================================================================================================
 * @file pins.c
 * @brief Port Multiplexer (PORTMUX) & Pin Manager Configuration
 * @ingroup pinsdriver
 * @version Driver Version 1.2.0
 *
 * =================================================================================================
 * HARDWARE PIN ALLOCATION & PERIPHERAL ROUTING:
 * =================================================================================================
 *  - PA4 : Motor Phase / Direction (PH) -> Driven by GPIO (LOW = Forward, HIGH = Reverse)
 *  - PA5 : Motor Enable / PWM (EN)      -> Driven by TCD0 WOB with autonomous hardware fault shutoff
 *  - PA7 : AC0 Comparator Output        -> Diagnostic probe point
 *  - PB2 : Asynchronous Wake-Up Pin     -> Digital input with pull-up & falling edge interrupt
 *  - PC0 : Limit Switch 1               -> Active-high input to EVSYS CH2 (CCL LUT1 IN0)
 *  - PC1 : Limit Switch 2               -> Active-high input to EVSYS CH3 (CCL LUT1 IN1)
 *  - PC4 : USART1 TXD                   -> Serial transmit (ALT1 pin routing)
 *  - PC5 : USART1 RXD                   -> Serial receive with pull-up (ALT1 pin routing)
 *  - PC6 : Curiosity Nano Onboard LED   -> Output (Active-Low: LOW=ON, HIGH=OFF)
 *  - PC7 : User Reset/Toggle Button SW0 -> Input with internal pull-up & both-edge interrupt
 *  - PD2 : AC0 Positive Input 0 (AINP0) -> Analog input from motor current sense amplifier
 *  - All unused pins (Ports A, B, C, D, E, F) -> Configured with internal pull-ups for low power
 * =================================================================================================
 */

#include "../pins.h"
#include <avr/interrupt.h>
#include <stddef.h>

static void (*IO_PD2_InterruptHandler)(void) = NULL;
static void (*IO_PA7_InterruptHandler)(void) = NULL;
static void (*IO_PA4_InterruptHandler)(void) = NULL;
static void (*IO_PC6_InterruptHandler)(void) = NULL;
static void (*IO_PB2_InterruptHandler)(void) = NULL;

/**
 * @brief Enables weak pull-ups on all pins to prevent CMOS float leakage current during sleep.
 */
static void PORT_Pullups_Initialize(void)
{
    for (uint8_t i = 0; i < 8; i++) {
        *((uint8_t *)&PORTA + 0x10 + i) = PORT_PULLUPEN_bm;
        *((uint8_t *)&PORTB + 0x10 + i) = PORT_PULLUPEN_bm;
        *((uint8_t *)&PORTC + 0x10 + i) = PORT_PULLUPEN_bm;
        *((uint8_t *)&PORTD + 0x10 + i) = PORT_PULLUPEN_bm;
        *((uint8_t *)&PORTE + 0x10 + i) = PORT_PULLUPEN_bm;
        *((uint8_t *)&PORTF + 0x10 + i) = PORT_PULLUPEN_bm;
    }
}

/**
 * @brief Initializes port direction, pull-ups, input sense configurations, and PORTMUX routes.
 */
void PIN_MANAGER_Initialize(void)
{
    /* 1. Terminate all pins with internal pull-ups as baseline to eliminate floating leakage */
    PORT_Pullups_Initialize();

    /* 2. Initial Output Latches */
    PORTA.OUT = 0x00; /* Motor PH (PA4) and EN (PA5) initialized LOW (Motor stopped) */
    PORTB.OUT = 0x00;
    PORTC.OUT = PIN6_bm | PIN4_bm; /* PC6=1 (LED OFF on active-low board), PC4=1 (UART1 TX idle mark) */
    PORTD.OUT = 0x00;
    PORTE.OUT = 0x00;
    PORTF.OUT = 0x00;

    /* 3. Direction Registers */
    PORTA.DIR = PIN4_bm | PIN5_bm | PIN7_bm; /* PA4 (PH), PA5 (EN), PA7 (AC0 Out) */
    PORTB.DIR = 0x00;                        /* PB2 is input */
    PORTC.DIR = PIN2_bm | PIN3_bm | PIN4_bm | PIN6_bm; /* PC4 TX output; PC5 RX input */
    PORTD.DIR = 0x8;
    PORTE.DIR = 0xFF;
    PORTF.DIR = 0xFF;

    /* 4. PINxCTRL Registers for Active/Specialized Pins */
    /* PA4 & PA5: Normal digital outputs */
    PORTA.PIN4CTRL = 0x00;
    PORTA.PIN5CTRL = 0x00;
    PORTA.PIN7CTRL = 0x00; /* PA7: AC0 diagnostic output */

    /* PB2: Asynchronous Wake-Up Pin (Pull-up + Falling-edge interrupt) */
    PORTB.DIRCLR   = PIN2_bm;
    PORTB.PIN2CTRL = PORT_PULLUPEN_bm | PORT_ISC_INTDISABLE_gc;

    /* PC0 & PC1: Limit switches (Pull-down / Pull-off, both-edge sensing handled in Limit_switch()) */
    PORTC.PIN0CTRL = PORT_ISC_INTDISABLE_gc;
    PORTC.PIN1CTRL = PORT_ISC_INTDISABLE_gc;

    /* PC4: USART1 TXD, PC5: USART1 RXD */
    PORTC.PIN4CTRL = 0x00;
    PORTC.PIN5CTRL = 0x00;

    /* PC6: User LED (Output) */
    PORTC.PIN6CTRL = 0x00;

    /* PC7: User SW0 Button (Pull-up + both-edge interrupt) */
    PORTC.PIN7CTRL = PORT_PULLUPEN_bm | PORT_ISC_INTDISABLE_gc;

    /* PD2: AC0 Positive Input (Disable digital input buffer for analog pin to eliminate noise/leakage) */
   PORTD.PIN2CTRL = 0x4;

    /* 5. PORTMUX Initialization */
    PORTMUX.ACROUTEA    = 0x00;
    PORTMUX.CCLROUTEA   = 0x00;
    PORTMUX.EVSYSROUTEA = 0x00;
    PORTMUX.SPIROUTEA   = 0x00;
    PORTMUX.TCAROUTEA   = 0x00; /* PA0 WO0 removed */
    PORTMUX.TCBROUTEA   = 0x00;
    PORTMUX.TCDROUTEA   = PORTMUX_TCD0_DEFAULT_gc; /* WOB on PA5 */
    PORTMUX.TWIROUTEA   = 0x00;
    PORTMUX.USARTROUTEA = PORTMUX_USART1_ALT1_gc; /* Route USART1 to PC4 (TXD) and PC5 (RXD) */
    PORTMUX.USARTROUTEB = 0x00;
    PORTMUX.ZCDROUTEA   = 0x00;
}

/**
 * Interrupt Handlers & Registrations
 */
void IO_PB2_SetInterruptHandler(void (* interruptHandler)(void)) 
{
    IO_PB2_InterruptHandler = interruptHandler;
}

void IO_PD2_SetInterruptHandler(void (* interruptHandler)(void)) 
{
    IO_PD2_InterruptHandler = interruptHandler;
}

void IO_PA7_SetInterruptHandler(void (* interruptHandler)(void)) 
{
    IO_PA7_InterruptHandler = interruptHandler;
}

void IO_PA4_SetInterruptHandler(void (* interruptHandler)(void)) 
{
    IO_PA4_InterruptHandler = interruptHandler;
}

void IO_PC6_SetInterruptHandler(void (* interruptHandler)(void)) 
{
    IO_PC6_InterruptHandler = interruptHandler;
}

ISR(PORTA_PORT_vect)
{ 
    if(VPORTA.INTFLAGS & PORT_INT7_bm)
    {
       if (IO_PA7_InterruptHandler != NULL) IO_PA7_InterruptHandler(); 
    }
    if(VPORTA.INTFLAGS & PORT_INT4_bm)
    {
       if (IO_PA4_InterruptHandler != NULL) IO_PA4_InterruptHandler(); 
    }
    VPORTA.INTFLAGS = 0xff;
}

ISR(PORTB_PORT_vect)
{ 
    /* Check if PB2 triggered the wake-up interrupt */
    if (VPORTB.INTFLAGS & PORT_INT2_bm)
    {
        if (IO_PB2_InterruptHandler != NULL)
        {
            IO_PB2_InterruptHandler();
        }
    }
    /* Clear all PORTB interrupt flags */
    VPORTB.INTFLAGS = 0xff;
}

ISR(PORTD_PORT_vect)
{ 
    if(VPORTD.INTFLAGS & PORT_INT2_bm)
    {
       if (IO_PD2_InterruptHandler != NULL) IO_PD2_InterruptHandler(); 
    }
    VPORTD.INTFLAGS = 0xff;
}

ISR(PORTE_PORT_vect)
{ 
    VPORTE.INTFLAGS = 0xff;
}

ISR(PORTF_PORT_vect)
{ 
    VPORTF.INTFLAGS = 0xff;
}
