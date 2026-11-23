/**
 * =================================================================================================
 * @file main.c
 * @brief Autonomous Zero-CPU Motor Control, CIP Stall Detection & Low-Power Sleep Management
 * @target Microchip AVR128DA48 (Curiosity Nano Platform)
 * @clock  24 MHz System Clock (OSCHF = 24 MHz, Main Prescaler PEN = 0)
 *
 * =================================================================================================
 * ARCHITECTURAL DESIGN & CORE-INDEPENDENT PERIPHERAL (CIP) OVERVIEW:
 * =================================================================================================
 * This firmware establishes a hardware-arbitrated, fail-safe motor drive architecture integrated with
 * ultra-low-power Power-Down sleep management:
 *
 * 1. SLEEP & WAKE-UP LIFECYCLE:
 *    - Startup / Reset: MCU immediately enters Power-Down sleep mode after sending "GOOD NIGHT\r\n"
 *      over UART1 on pins PC4 (TXD) and PC5 (RXD), turning OFF the onboard LED (PC6), and shutting
 *      down all active peripherals.
 *    - Asynchronous Wake-Up: Pin PB2 is configured with internal pull-up and falling-edge sense.
 *      Pulling PB2 to GND asynchronously wakes the CPU from deep Power-Down mode.
 *    - Wake-Up Sequence: Turns ON the onboard LED (PC6), sends "GOOD MORNING\r\n" over UART1,
 *      restores VREF, DAC0, AC0, CCL, EVSYS, and Timers, calibrates current threshold, and starts
 *      the motor in Forward direction.
 *    - Active Stall Protection & Return to Sleep: When a continuous 200 ms stall overcurrent is
 *      detected by the CIP pipeline, TCD0 hardware cuts PWM instantly, the TCB0 interrupt signals
 *      the stall condition, and the MCU safely halts the motor, turns off peripherals, sends
 *      "GOOD NIGHT\r\n", turns OFF LED PC6, and transitions back to Power-Down sleep mode.
 *
 * 2. MOTOR DRIVER INTERFACE (PH/EN Mode - TI DRV8874):
 *    - PA4 (GPIO Output)    : Phase / Direction Control (PH) (LOW = Forward, HIGH = Reverse)
 *    - PA5 (TCD0 WOB Output): PWM Speed & Hardware Fault Shutoff Gate (EN, 64.2 kHz dual-slope PWM)
 *
 * 3. OVERCURRENT / STALL PIPELINE:
 *    [PD2 Shunt] -> AC0 -> CCL LUT0 Filter -> TCB0 (200 ms Timeout) -> EVSYS CH6 -> CCL LUT2/3 Latch
 *    -> EVSYS CH4 -> TCD0 Input A -> PA5 PWM forced LOW + TCB0 Interrupt sets stall flag.
 *
 * 4. DUAL LIMIT-SWITCH PIPELINE:
 *    PC0 & PC1 -> EVSYS CH2/3 -> CCL LUT1 OR Gate -> TCB1 (20 ms Debounce) -> CCL LUT2/3 Latch
 *    -> TCD0 Fault Input B -> PA5 PWM forced LOW.
 *
 * 5. USER RECOVERY & DIRECTION TOGGLE:
 *    PC7 (User Button SW0) debounced via TCB2 (50 ms single-shot) -> reverses motor and clears latch.
 * =================================================================================================
 */

#include "mcc_generated_files/system/system.h"
#include "usart1.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#ifndef F_CPU
#define F_CPU 24000000UL
#endif
#include <util/delay.h>
#include <stdbool.h>
#include <stdint.h>

/* =================================================================================================
 * HARDWARE CONFIGURATION & TIMING CONSTANTS
 * ================================================================================================= */
#define VREF_AC_MV                    4096U
#define AC_TRIGGER_VOLTAGE_MV_INIT    900U

/* TCD0 dual-slope 64.2 kHz PWM constants (48 MHz PLL clock) */
#define TOTAL_PERIOD                  373U
#define PWM_TOP                       93U
#define PWM_LOW                       0U

/* PH/EN Pin Definitions on PORTA */
#define MOTOR_PH_PIN_bm               PIN4_bm  /* PA4: Phase / Direction */
#define MOTOR_EN_PIN_bm               PIN5_bm  /* PA5: TCD0 WOB PWM Enable */

/* Timer Periods and Counts */
#define TCA0_1MS_PER_TICKS            93U      /* 93 counts @ 93.75 kHz = 1.0027 ms */
#define TCB0_200MS_CCMP_COUNTS        200U     /* 200 counts * 1.0027 ms = 200.5 ms stall timeout */
#define TCB1_20MS_DEBOUNCE_COUNTS     20U      /* 20 counts * 1.0027 ms = 20.05 ms debounce */
#define TCB2_50MS_DEBOUNCE_TICKS      4687U    /* 50 ms @ 93.75 kHz */

/* =================================================================================================
 * TYPE DEFINITIONS & GLOBAL STATE
 * ================================================================================================= */
typedef enum {
    DIR_FORWARD = 0, /* Initial state on restart: Hand Open  (PA4 = LOW)  */
    DIR_REVERSE = 1  /* Toggled state on PC7 press: Hand Close (PA4 = HIGH) */
} motor_dir_t;

static volatile motor_dir_t current_direction   = DIR_FORWARD;
static volatile bool        pb2_wakeup_requested = false;
static volatile bool        stall_detected       = false;

/* =================================================================================================
 * FUNCTION PROTOTYPES
 * ================================================================================================= */
void BOD_Disable(void);
void WDT_Disable(void);
void SLPCTRL_Init(void);
void PB2_Wakeup_Init(void);
void PB2_Wakeup_Callback(void);

void System_Enter_PowerDown_Sleep(void);
void System_Wakeup_And_Restore(void);
void MotorStop(void);
void Peripherals_Disable_For_Sleep(void);

void CCL_FaultLatch_Init(void);
void TCA0_ContinuousClock_Init(void);
void TCB0_StallTimeout_Init(void);
void TCB1_Debounce_Init(void);
void TCB2_SW0_Debounce_Init(void);
void SW0_Init(void);
void Limit_switch(void);
void MOTOR_start(void);
void MotorForward(void);
void MotorReverse(void);
void MotorApplyDirection(motor_dir_t dir);
void FaultReset_And_Restart(void);
uint8_t ac_calculate_trigger_voltage(uint16_t mV);
void ac_calibration(void);
void ac_trig_event_enable(void);
void ac_trig_event_disable(void);
void tcd_enable(void);

/* =================================================================================================
 * MAIN ENTRY POINT
 * ================================================================================================= */
int main(void)
{
    /* 1. Base System Hardware Initialization */
    SYSTEM_Initialize();

    /* 2. Completely disable BOD and WDT for minimum power consumption */
    BOD_Disable();
    WDT_Disable();
    SLPCTRL_Init();

    /* 3. Initialize PB2 Asynchronous Wake-Up Pin */
    PB2_Wakeup_Init();

    /* 4. Global Interrupts Enable */
    sei();

    /* 5. Startup / Reset -> Immediate Power-Down Mode:
     *    Sends "GOOD NIGHT\r\n" on PC4/PC5, turns off LED PC6, shuts down peripherals,
     *    and enters Power-Down sleep until pin PB2 triggers. */
    System_Enter_PowerDown_Sleep();

    /* 6. Main Super-Loop */
    while (1)
    {
        /* Asynchronous wake-up triggered by PB2 falling edge */
        if (pb2_wakeup_requested)
        {
            pb2_wakeup_requested = false;
            System_Wakeup_And_Restore();
        }

        /* 200 ms continuous overcurrent stall detected by CIP pipeline and limit switch press*/
        if (stall_detected)
        {
            stall_detected = false;
            System_Enter_PowerDown_Sleep();
        }
    }
}

/* =================================================================================================
 * LOW-POWER & SLEEP MANAGEMENT FUNCTIONS
 * ================================================================================================= */

/**
 * @brief Completely disables Brown-Out Detector (BOD) and Voltage Level Monitor (VLM).
 */
void BOD_Disable(void)
{
    ccp_write_io((void*)&(BOD.CTRLA), 0x00);
    BOD.INTCTRL  = 0x00;
    BOD.VLMCTRLA = 0x00;
}

/**
 * @brief Disables Watchdog Timer (WDT) to prevent unwanted wake-up/resets during sleep.
 */
void WDT_Disable(void)
{
    ccp_write_io((void*)&(WDT.CTRLA), 0x00);
}

/**
 * @brief Initializes Sleep Controller (SLPCTRL).
 */
void SLPCTRL_Init(void)
{
    ccp_write_io((void*)&(SLPCTRL.CTRLA), 0x00);
}

/**
 * @brief Configures PB2 as digital input with internal pull-up and falling edge asynchronous interrupt.
 */
void PB2_Wakeup_Init(void)
{
    PORTB.DIRCLR   = PIN2_bm;
    PORTB.PIN2CTRL = PORT_PULLUPEN_bm | PORT_ISC_FALLING_gc;
    IO_PB2_SetInterruptHandler(PB2_Wakeup_Callback);
}

/**
 * @brief Callback invoked by PORTB ISR when PB2 transitions LOW.
 */
void PB2_Wakeup_Callback(void)
{
    pb2_wakeup_requested = true;
}

/**
 * @brief Safely shuts off motor outputs to prevent static current draw.
 */
void MotorStop(void)
{
    /* Clear Phase (PA4) and Enable (PA5) to GND */
    PORTA.OUTCLR = MOTOR_PH_PIN_bm | MOTOR_EN_PIN_bm;
    PORTA.DIRSET = MOTOR_PH_PIN_bm | MOTOR_EN_PIN_bm;

    /* Disable TCD0 */
    TCD0.CTRLA &= ~TCD_ENABLE_bm;
}

/**
 * @brief Disables all active Core-Independent Peripherals before entering Power-Down sleep mode.
 */
void Peripherals_Disable_For_Sleep(void)
{
    /* Disconnect Event System channels */
    ac_trig_event_disable();

    /* Disable Analog Comparator */
    AC0.INTCTRL = 0x00;
    AC0.CTRLA   = 0x00;

    /* Disable DAC0 */
    DAC0.CTRLA  = 0x00;

    /* Disable Voltage Reference (ACREF) */
    VREF.ACREF  = 0x00;

    /* Disable Timers */
    TCA0.SINGLE.CTRLA &= ~TCA_SINGLE_ENABLE_bm;
    TCB0.CTRLA        &= ~TCB_ENABLE_bm;
    TCB1.CTRLA        &= ~TCB_ENABLE_bm;
    TCB2.CTRLA        &= ~TCB_ENABLE_bm;

    /* Disable CCL */
    CCL.CTRLA         &= ~CCL_ENABLE_bm;
    
     ac_trig_event_disable();//modi 1
     /* 7. Stop PLL 2X multiplier (ensures 0.0 uA from PLL clock domain) */
  
    ccp_write_io((void *)&(CLKCTRL.PLLCTRLA), 0x00);


}

/**
 * @brief Transitions the MCU into deep Power-Down Sleep Mode.
 */
void System_Enter_PowerDown_Sleep(void)
{
    /* 1. Safely stop motor drive */
    MotorStop();

    /* 2. Disable all active peripherals to eliminate active power consumption */
    Peripherals_Disable_For_Sleep();

    /* 3. Turn OFF onboard Status LED PC6 (Curiosity Nano active-low: HIGH = OFF) */
    LED_OFF();

    /* 4. Transmit "GOOD NIGHT\r\n" through UART1 on pins PC4 & PC5 */
    Terminal_Enable();
    printf("GOOD NIGHT\r\n");
    Terminal_Disable(); /* Flushes shift register and shuts down UART1 */

    /* 5. Ensure PB2 is armed for asynchronous wake-up */
    PORTB.DIRCLR   = PIN2_bm;
    PORTB.PIN2CTRL = PORT_PULLUPEN_bm | PORT_ISC_FALLING_gc;
    VPORTB.INTFLAGS = 0xFF; /* Clear any pending PORTB flags */

    /* 6. Enter Power-Down Sleep Mode */
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    sleep_mode();

    /* Execution halts here until an asynchronous interrupt on PB2 wakes the CPU */
}

/**
 * @brief Restores all peripherals and operational state upon waking up from Power-Down sleep.
 */
void System_Wakeup_And_Restore(void)
{
    /* 1. Keep onboard Status LED PC6 turned ON while CPU is awake */
    LED_ON();

    /* 2. Re-enable USART1 on PC4/PC5 and transmit "GOOD MORNING\r\n" */
   Terminal_Enable();
    printf("GOOD MORNING\r\n");

    /* 3. Restore Voltage Reference (4.096V) & DAC0 */
    VREF.ACREF = (1 << VREF_ALWAYSON_bp) | VREF_REFSEL_4V096_gc;
    DAC0.CTRLA = 0x01;

    /* 4. Restore Limit switches (PC0/PC1) and SW0 button (PC7) */
    SW0_Init();
    Limit_switch();

    /* 5. Restore CCL logic Look-Up Tables */
    CCL_FaultLatch_Init();

    /* 6. Restore Continuous Timers */
    TCA0_ContinuousClock_Init();
    TCB0_StallTimeout_Init();
    TCB1_Debounce_Init();
    TCB2_SW0_Debounce_Init();

    /* 7. Start Motor in DIR_FORWARD with CIP overcurrent protection armed */
    MOTOR_start();

    /* 8. Clear any latent stall flag */
    stall_detected = false;
}

/* =================================================================================================
 * CORE-INDEPENDENT PERIPHERAL (CIP) CONFIGURATION FUNCTIONS
 * ================================================================================================= */

void CCL_FaultLatch_Init(void)
{
    /* Disable CCL globally and all individual LUTs */
    CCL.CTRLA     &= ~CCL_ENABLE_bm;
    CCL.LUT0CTRLA &= ~CCL_ENABLE_bm;
    CCL.LUT1CTRLA &= ~CCL_ENABLE_bm;
    CCL.LUT2CTRLA &= ~CCL_ENABLE_bm;
    CCL.LUT3CTRLA &= ~CCL_ENABLE_bm;
    CCL.LUT4CTRLA &= ~CCL_ENABLE_bm;
    CCL.LUT5CTRLA &= ~CCL_ENABLE_bm;

    CCL.SEQCTRL0 = CCL_SEQSEL_DISABLE_gc; /* LUT0/LUT1: independent combinational gates */
    CCL.SEQCTRL1 = CCL_SEQSEL_RS_gc;      /* LUT2/LUT3: RS Latch */

    /* LUT0: Non-latched buffer of AC0 with CLK_PER digital filter */
    CCL.TRUTH0    = 0x02U;
    CCL.LUT0CTRLB = CCL_INSEL0_AC0_gc | CCL_INSEL1_MASK_gc;
    CCL.LUT0CTRLC = CCL_INSEL2_MASK_gc;
    CCL.LUT0CTRLA = CCL_CLKSRC_CLKPER_gc
                  | CCL_EDGEDET_DIS_gc
                  | CCL_FILTSEL_FILTER_gc
                  | CCL_ENABLE_bm;

    /* LUT1: Active-High OR combiner for Limit Switches PC0 and PC1 */
    CCL.TRUTH1    = 0x0EU;
    CCL.LUT1CTRLB = CCL_INSEL0_EVENTA_gc | CCL_INSEL1_EVENTB_gc;
    CCL.LUT1CTRLC = CCL_INSEL2_MASK_gc;
    CCL.LUT1CTRLA = CCL_CLKSRC_CLKPER_gc
                  | CCL_EDGEDET_DIS_gc
                  | CCL_FILTSEL_FILTER_gc
                  | CCL_ENABLE_bm;

    /* LUT2 (Set side of RS Latch): Trips on TCB0 stall OR TCB1 limit switch */
    CCL.TRUTH2    = 0x0EU;
    CCL.LUT2CTRLB = CCL_INSEL0_EVENTA_gc | CCL_INSEL1_EVENTB_gc;
    CCL.LUT2CTRLC = CCL_INSEL2_MASK_gc;
    CCL.LUT2CTRLA = CCL_CLKSRC_CLKPER_gc
                  | CCL_EDGEDET_DIS_gc
                  | CCL_FILTSEL_DISABLE_gc
                  | CCL_ENABLE_bm;

    /* LUT3 (Reset side of RS Latch): Cleared by Software Event SWEVENTA_CH1 */
    CCL.TRUTH3    = 0x02U;
    CCL.LUT3CTRLB = CCL_INSEL0_EVENTA_gc | CCL_INSEL1_MASK_gc;
    CCL.LUT3CTRLC = CCL_INSEL2_MASK_gc;
    CCL.LUT3CTRLA = CCL_CLKSRC_CLKPER_gc
                  | CCL_EDGEDET_DIS_gc
                  | CCL_FILTSEL_DISABLE_gc
                  | CCL_ENABLE_bm;

    /* Connect Channel 1 to LUT3 Event Input A and clear RS latch */
    EVSYS.CHANNEL1     = EVSYS_CHANNEL1_OFF_gc;
    EVSYS.USERCCLLUT3A = EVSYS_USER_CHANNEL1_gc;

    CCL.CTRLA = CCL_ENABLE_bm;
    EVSYS.SWEVENTA = EVSYS_SWEVENTA_CH1_gc;
}

/**
 * @brief Configures TCA0 as a continuous 1.0027 ms timebase for TCB0 and TCB1.
 * Note: PA0 pin output routing has been removed per specification.
 */
void TCA0_ContinuousClock_Init(void)
{
    /* 1. Disable TCA0 while configuring */
    TCA0.SINGLE.CTRLA &= ~TCA_SINGLE_ENABLE_bm;

    /* 2. Set counter, period (1.0027 ms), and compare value */
    TCA0.SINGLE.CNT   = 0x0000U;
    TCA0.SINGLE.PER   = TCA0_1MS_PER_TICKS;        /* 93 counts -> 1.0027 ms */
    TCA0.SINGLE.CMP0  = (TCA0_1MS_PER_TICKS / 2);

    /* 3. Single-Slope mode generating OVF every 1.0027 ms */
    TCA0.SINGLE.CTRLB = TCA_SINGLE_WGMODE_SINGLESLOPE_gc;

    /* 4. Clear flags and enable timer */
    TCA0.SINGLE.EVCTRL   = 0x00U;
    TCA0.SINGLE.INTFLAGS = TCA_SINGLE_OVF_bm | TCA_SINGLE_CMP0_bm;
    TCA0.SINGLE.CTRLA    = TCA_SINGLE_CLKSEL_DIV256_gc | TCA_SINGLE_ENABLE_bm;
}

void TCB0_StallTimeout_Init(void)
{
    TCB0.CTRLA &= ~TCB_ENABLE_bm;

    /* Time-Out Check Mode (200 ms continuous overcurrent stall):
     * - USERTCB0COUNT (CH5 = TCA0_OVF) clocks the counter every 1.0027 ms.
     * - USERTCB0CAPT  (CH0 = CCL_LUT0) provides the gating window. */
    TCB0.CTRLB    = TCB_CNTMODE_TIMEOUT_gc;
    TCB0.EVCTRL   = TCB_CAPTEI_bm;
    TCB0.CCMP     = TCB0_200MS_CCMP_COUNTS;
    TCB0.CNT      = 0x0000U;
    TCB0.INTFLAGS = TCB_CAPT_bm | TCB_OVF_bm;
    TCB0.INTCTRL  = TCB_CAPT_bm; /* Generates event and interrupt on timeout */

    TCB0.CTRLA = TCB_CLKSEL_EVENT_gc | TCB_ENABLE_bm;
}

void TCB1_Debounce_Init(void)
{
    TCB1.CTRLA &= ~TCB_ENABLE_bm;

    /* Time-Out Check Mode (20 ms hardware debounce for PC0/PC1 limit switches):
     * - USERTCB1COUNT (CH5 = TCA0_OVF) clocks counter every 1.0027 ms.
     * - USERTCB1CAPT  (CH7 = CCL_LUT1) provides the gating window. */
    TCB1.CTRLB    = TCB_CNTMODE_TIMEOUT_gc;
    TCB1.EVCTRL   = TCB_CAPTEI_bm;
    TCB1.CCMP     = TCB1_20MS_DEBOUNCE_COUNTS;
    TCB1.CNT      = 0x0000U;
    TCB1.INTFLAGS = TCB_CAPT_bm | TCB_OVF_bm;
    TCB1.INTCTRL  = TCB_CAPT_bm;

    TCB1.CTRLA = TCB_CLKSEL_EVENT_gc | TCB_ENABLE_bm;
}

void TCB2_SW0_Debounce_Init(void)
{
    TCB2.CTRLA &= ~TCB_ENABLE_bm;

    /* Periodic Interrupt mode used as a 50 ms single-shot debounce timer for PC7 */
    TCB2.CTRLB    = TCB_CNTMODE_INT_gc;
    TCB2.EVCTRL   = 0x00U;
    TCB2.CCMP     = TCB2_50MS_DEBOUNCE_TICKS;
    TCB2.CNT      = 0x0000U;
    TCB2.INTFLAGS = TCB_CAPT_bm | TCB_OVF_bm;
    TCB2.INTCTRL  = TCB_CAPT_bm;

    TCB2.CTRLA = TCB_CLKSEL_TCA0_gc;
}

void SW0_Init(void)
{
    PORTC.DIRCLR   = PIN7_bm;
    PORTC.PIN7CTRL = PORT_ISC_BOTHEDGES_gc | PORT_PULLUPEN_bm;
}

void Limit_switch(void)
{
    /* PC0 and PC1 as active-high limit switch inputs with pull-offs and both-edge interrupts */
    PORTC.DIRCLR   = PIN0_bm | PIN1_bm;
    PORTC.PIN0CTRL = PORT_ISC_BOTHEDGES_gc;
    PORTC.PIN1CTRL = PORT_ISC_BOTHEDGES_gc;

    _delay_us(10);

    /* Permanently route PC0 (CH2) and PC1 (CH3) to CCL_LUT1 Event Inputs A & B */
    EVSYS.CHANNEL2     = EVSYS_CHANNEL2_PORTC_PIN0_gc;
    EVSYS.USERCCLLUT1A = EVSYS_USER_CHANNEL2_gc;

    EVSYS.CHANNEL3     = EVSYS_CHANNEL3_PORTC_PIN1_gc;
    EVSYS.USERCCLLUT1B = EVSYS_USER_CHANNEL3_gc;
}

void tcd_enable(void)
{
    PORTA.PIN4CTRL &= ~PORT_INVEN_bm;
    PORTA.PIN5CTRL &= ~PORT_INVEN_bm;
    PORTA.DIRSET    = MOTOR_PH_PIN_bm | MOTOR_EN_PIN_bm;

    TCD0.INTFLAGS = 0xFFU;
    TCD0.STATUS   = 0xFFU;
    while (!(TCD0.STATUS & TCD_ENRDY_bm));
    TCD0.CTRLA   |= TCD_ENABLE_bm;
    while (!(TCD0.STATUS & TCD_CMDRDY_bm));
    TCD0.CTRLE    = TCD_RESTART_bm;
}

uint8_t ac_calculate_trigger_voltage(uint16_t mV)
{
    return (uint8_t)(((uint32_t)mV * 256U) / VREF_AC_MV);
}

//void ac_calibration(void)
//{
//    LED_ON(); /* Onboard LED ON */
//    uint8_t calibrating = 1;
//    uint16_t ac_trigger_voltage_mv = AC_TRIGGER_VOLTAGE_MV_INIT;
//    AC0.INTCTRL = AC_INTMODE_NORMAL_POSEDGE_gc;
//
//    while (calibrating)
//    {
//        AC0.DACREF = ac_calculate_trigger_voltage(ac_trigger_voltage_mv);
//
//        if (!(AC0.STATUS & AC_CMPSTATE_bm))
//        {
//            continue;
//        }
//
//        AC0.STATUS = AC_CMPIF_bm;
//
//        _delay_ms(100);
//        if (!(AC0.STATUS & AC_CMPIF_bm))
//        {
//            calibrating = 0;
//            LED_ON();
//        }
//    }
//
//    AC0.DACREF = ac_calculate_trigger_voltage(ac_trigger_voltage_mv);
//}

void ac_trig_event_enable(void)
{
    AC0.INTCTRL = AC_INTMODE_NORMAL_POSEDGE_gc | AC_CMP_bm;

    /* Channel 1: Software Event -> resets CCL RS Latch */
    EVSYS.CHANNEL1     = EVSYS_CHANNEL1_OFF_gc;
    EVSYS.USERCCLLUT3A = EVSYS_USER_CHANNEL1_gc;
    EVSYS.SWEVENTA     = EVSYS_SWEVENTA_CH1_gc;

    /* Reset timer counters */
    TCB0.CNT      = 0x0000U;
    TCB0.INTFLAGS = TCB_CAPT_bm | TCB_OVF_bm;
    TCB1.CNT      = (!(VPORTC.IN & (PIN0_bm | PIN1_bm))) ? (TCB1_20MS_DEBOUNCE_COUNTS + 1U) : 0x0000U;
    TCB1.INTFLAGS = TCB_CAPT_bm | TCB_OVF_bm;
    TCD0.INTFLAGS = 0xFFU;

    /* Channel 0: CCL_LUT0 (filtered AC0) -> TCB0 Event Capture Input */
    EVSYS.CHANNEL0      = EVSYS_CHANNEL0_CCL_LUT0_gc;
    EVSYS.USERTCB0CAPT  = EVSYS_USER_CHANNEL0_gc;

    /* Channel 2 & 3: Limit Switches PC0 & PC1 -> CCL_LUT1 Event Inputs A & B */
    EVSYS.CHANNEL2      = EVSYS_CHANNEL2_PORTC_PIN0_gc;
    EVSYS.USERCCLLUT1A  = EVSYS_USER_CHANNEL2_gc;
    EVSYS.CHANNEL3      = EVSYS_CHANNEL3_PORTC_PIN1_gc;
    EVSYS.USERCCLLUT1B  = EVSYS_USER_CHANNEL3_gc;

    /* Channel 5: TCA0 1.0027 ms Overflow -> Clocks TCB0 and TCB1 */
    EVSYS.CHANNEL5      = EVSYS_CHANNEL5_TCA0_OVF_LUNF_gc;
    EVSYS.USERTCB0COUNT = EVSYS_USER_CHANNEL5_gc;
    EVSYS.USERTCB1COUNT = EVSYS_USER_CHANNEL5_gc;

    /* Channel 7: CCL_LUT1 -> TCB1 Event Capture Input */
    EVSYS.CHANNEL7      = EVSYS_CHANNEL7_CCL_LUT1_gc;
    EVSYS.USERTCB1CAPT  = EVSYS_USER_CHANNEL7_gc;

    /* Channel 6: TCB0 200 ms Stall Timeout Pulse -> CCL_LUT2 Event Input A */
    EVSYS.CHANNEL6      = EVSYS_CHANNEL6_TCB0_CAPT_gc;
    EVSYS.USERCCLLUT2A  = EVSYS_USER_CHANNEL6_gc;

    /* Channel 8: TCB1 20 ms Limit Switch Pulse -> CCL_LUT2 Event Input B */
    EVSYS.CHANNEL8      = EVSYS_CHANNEL8_TCB1_CAPT_gc;
    EVSYS.USERCCLLUT2B  = EVSYS_USER_CHANNEL8_gc;

    /* Channel 4: CCL_LUT2 Latched HIGH -> TCD0 Fault Input A (Shuts down PWM) */
    EVSYS.CHANNEL4       = EVSYS_CHANNEL4_CCL_LUT2_gc;
    EVSYS.USERTCD0INPUTA = EVSYS_USER_CHANNEL4_gc;
}

void ac_trig_event_disable(void)
{
    AC0.INTCTRL          &= ~AC_CMP_bm;
    EVSYS.CHANNEL0        = EVSYS_CHANNEL0_OFF_gc;
    EVSYS.CHANNEL4        = EVSYS_CHANNEL4_OFF_gc;
    EVSYS.CHANNEL5        = EVSYS_CHANNEL5_OFF_gc;
    EVSYS.CHANNEL6        = EVSYS_CHANNEL6_OFF_gc;
    EVSYS.CHANNEL7        = EVSYS_CHANNEL7_OFF_gc;
    EVSYS.CHANNEL8        = EVSYS_CHANNEL8_OFF_gc;

    EVSYS.USERTCB0CAPT    = EVSYS_USER_OFF_gc;
    EVSYS.USERTCB0COUNT   = EVSYS_USER_OFF_gc;
    EVSYS.USERTCB1CAPT    = EVSYS_USER_OFF_gc;
    EVSYS.USERTCB1COUNT   = EVSYS_USER_OFF_gc;
    EVSYS.USERCCLLUT2A    = EVSYS_USER_OFF_gc;
    EVSYS.USERCCLLUT2B    = EVSYS_USER_OFF_gc;
    EVSYS.USERTCD0INPUTA  = EVSYS_USER_OFF_gc;
}

void MotorForward(void)
{
    PORTA.OUTCLR = MOTOR_PH_PIN_bm; /* PA4 (PH) = LOW */
    while (!(TCD0.STATUS & TCD_CMDRDY_bm));
    TCD0.CMPBSET = PWM_TOP;
    TCD0.CTRLE   = TCD_SYNCEOC_bm;
}

void MotorReverse(void)
{
    PORTA.OUTSET = MOTOR_PH_PIN_bm; /* PA4 (PH) = HIGH */
    while (!(TCD0.STATUS & TCD_CMDRDY_bm));
    TCD0.CMPBSET = PWM_TOP;
    TCD0.CTRLE   = TCD_SYNCEOC_bm;
}

void MotorApplyDirection(motor_dir_t dir)
{
    if (dir == DIR_FORWARD)
    {
        MotorForward();
    }
    else
    {
        MotorReverse();
    }
}

void MOTOR_start(void)
{
    ac_trig_event_disable();
    current_direction = DIR_FORWARD;
    tcd_enable();
    MotorApplyDirection(current_direction);

    /* Run initial current calibration */
//    ac_calibration();
    ac_trig_event_enable();
}

void FaultReset_And_Restart(void)
{
    /* 1. Check if a limit switch is actively held HIGH during reset */
    if (VPORTC.IN & PIN0_bm)
    {
        current_direction = DIR_REVERSE;
        EVSYS.USERCCLLUT2B = EVSYS_USER_OFF_gc;
    }
    else if (VPORTC.IN & PIN1_bm)
    {
        current_direction = DIR_FORWARD;
        EVSYS.USERCCLLUT2B = EVSYS_USER_OFF_gc;
    }
    else
    {
        current_direction = (current_direction == DIR_FORWARD) ? DIR_REVERSE : DIR_FORWARD;
        EVSYS.USERCCLLUT2B = EVSYS_USER_CHANNEL8_gc;
    }
    
    MotorApplyDirection(current_direction);

    /* 2. Reset the CCL RS latch via Software Event on Channel 1 */
    EVSYS.SWEVENTA = EVSYS_SWEVENTA_CH1_gc;

    /* 3. Reset TCB0 stall counter and TCB1 limit switch counter */
    TCB0.CNT      = 0x0000U;
    TCB0.INTFLAGS = TCB_CAPT_bm | TCB_OVF_bm;
    TCB1.CNT      = 0x0000U;
    TCB1.INTFLAGS = TCB_CAPT_bm | TCB_OVF_bm;

    /* 4. Clear latched TCD0 fault flags and un-gate PA5 PWM output */
    TCD0.INTFLAGS = 0xFFU;
    TCD0.STATUS   = 0xFFU;
    while (!(TCD0.STATUS & TCD_CMDRDY_bm));
    TCD0.CTRLE    = TCD_RESTART_bm;
    LED_ON();
}

/* =================================================================================================
 * INTERRUPT SERVICE ROUTINES
 * ================================================================================================= */

/* PORTC pin-change ISR: handles PC7 button and PC0/PC1 limit switch releases */
ISR(PORTC_PORT_vect)
{
    uint8_t flags = PORTC.INTFLAGS;

    /* Check if interrupt was caused by PC7 (User Button SW0) */
    if (flags & PIN7_bm)
    {
        PORTC.INTFLAGS = PIN7_bm;
        TCB2.CTRLA   &= ~TCB_ENABLE_bm;
        TCB2.CNT      = 0x0000U;
        TCB2.INTFLAGS = TCB_CAPT_bm | TCB_OVF_bm;
        TCB2.CTRLA   |= TCB_ENABLE_bm;
    }

    /* Check if PC0 or PC1 limit switches were released */
    if (flags & (PIN0_bm | PIN1_bm))
    {
        PORTC.INTFLAGS = PIN0_bm | PIN1_bm;

        if (!(VPORTC.IN & (PIN0_bm | PIN1_bm)))
        {
            EVSYS.USERCCLLUT2B = EVSYS_USER_CHANNEL8_gc;
        }
    }
}

/* TCB2 Compare Match ISR: fires after 50 ms of PC7 (SW0) stability */
ISR(TCB2_INT_vect)
{
    TCB2.INTFLAGS = TCB_CAPT_bm | TCB_OVF_bm;
    TCB2.CTRLA   &= ~TCB_ENABLE_bm;

    if (!(VPORTC.IN & PIN7_bm))
    {
        FaultReset_And_Restart();
    }
}

/* TCB1 Timeout ISR: fires when PC0 or PC1 is held unbroken for 20 ms */
ISR(TCB1_INT_vect)
{
    TCB1.INTFLAGS = TCB_CAPT_bm | TCB_OVF_bm;
    stall_detected = true;
    
}

/* AC0 telemetry ISR */
ISR(AC0_AC_vect)
{
    AC0.STATUS = AC_CMPIF_bm;
}

/* TCB0 capture ISR: fires when continuous 200 ms stall condition trips */
ISR(TCB0_INT_vect)
{
    TCB0.INTFLAGS = TCB_CAPT_bm | TCB_OVF_bm;

    /* Signal main loop to safely shut down motor, send GOOD NIGHT, and enter Power-Down sleep */
    stall_detected = true;
}