/**
 * ============================================================================
 * AVR128DA48 USART1 Serial Communication Driver - usart1.c
 * ============================================================================
 * Provides a robust hardware serial driver for USART1 on PA0 (TXD) and PA1 (RXD):
 *   - 115200 baud, 8 data bits, no parity, 1 stop bit (8N1).
 *   - Direct hardware register writes to prevent buffer deadlocks.
 *   - Standard I/O (stdio) stream binding: redirects printf() to USART1 on PA0.
 * ============================================================================
 */

#include "usart0.h"

#if defined(__GNUC__)

/**
 * Terminal_printCHAR()
 * Callback used by AVR libc's standard I/O stream system.
 * Routes characters passed to printf() or puts() directly to the USART1 hardware register.
 */
int Terminal_printCHAR(char character, FILE *stream)
{
    Terminal_Write((uint8_t)character);
    return 0;
}

/*
 * Set up custom standard I/O stream bound to Terminal_printCHAR.
 * Initialized with write privileges (_FDEV_SETUP_WRITE).
 */
FILE Terminal_stream = FDEV_SETUP_STREAM(Terminal_printCHAR, NULL, _FDEV_SETUP_WRITE);

#elif defined(__ICCAVR__)

/**
 * putchar()
 * Standard C library character output hook for IAR compiler.
 */
int putchar(int outChar)
{
    Terminal_Write(outChar);
    return outChar;
}
#endif

static volatile bool Terminal_tx_pending = false;

bool Terminal_IsTxReady(void)
{
    return (USART1.STATUS & USART_DREIF_bm) ? true : false;
}

bool USART1_IsTxReady(void)
{
    return Terminal_IsTxReady();
}

bool Terminal_IsRxReady(void)
{
    return (USART1.STATUS & USART_RXCIF_bm) ? true : false;
}

bool USART1_IsRxReady(void)
{
    return Terminal_IsRxReady();
}

bool Terminal_IsTxBusy(void)
{
    return !(USART1.STATUS & USART_TXCIF_bm);
}

bool Terminal_IsTxDone(void)
{
    return (USART1.STATUS & USART_TXCIF_bm) ? true : false;
}

bool USART1_IsTxDone(void)
{
    return Terminal_IsTxDone();
}

uint8_t Terminal_Read(void)
{
    while (!(USART1.STATUS & USART_RXCIF_bm))
        ;
    return USART1.RXDATAL;
}

uint8_t USART1_Read(void)
{
    return Terminal_Read();
}

uint8_t Terminal_GetData(void)
{
    return USART1.RXDATAL;
}

void Terminal_Write(const uint8_t data)
{
    /* Wait until hardware transmit data register is empty */
    while (!(USART1.STATUS & USART_DREIF_bm))
        ;
    /* Clear Transmit Complete flag before sending */
    USART1.STATUS = USART_TXCIF_bm;
    /* Load character into hardware transmit data register */
    USART1.TXDATAL = data;
    Terminal_tx_pending = true;
}

void USART1_Write(const uint8_t data)
{
    Terminal_Write(data);
}

void Terminal_WriteString(const char *str)
{
    if (str == NULL) return;
    while (*str)
    {
        Terminal_Write((uint8_t)*str++);
    }
}

void Terminal_Initialize(void)
{
    /* Ensure PORTMUX routes USART1 to default pins: PC4 (TXD) and PC5 (RXD) */
    PORTMUX.USARTROUTEA = (PORTMUX.USARTROUTEA & ~PORTMUX_USART1_gm) | PORTMUX_USART1_ALT1_gc;

    /* Configure PA0 as TXD output (idle HIGH) and PA1 as RXD input with weak pull-up */
    PORTC.DIRSET   = PIN4_bm;
    PORTC.OUTSET   = PIN4_bm;
    PORTC.PIN0CTRL = 0x00;
    PORTC.DIRCLR   = PIN5_bm;
    PORTC.PIN4CTRL = PORT_PULLUPEN_bm;

    /* Set baud rate register for 115200 baud (calculated from 24 MHz F_CPU -> BAUD = 833) */
    USART1.BAUD = (uint16_t)USART1_BAUD_RATE(115200);

    /* CTRLA: No interrupts enabled for polled transmission */
    USART1.CTRLA = 0x00;

    /* CTRLB: Enable Transmitter and Receiver in normal speed mode */
    USART1.CTRLB = USART_RXEN_bm | USART_TXEN_bm;

    /* CTRLC: Asynchronous mode, 8 data bits, no parity, 1 stop bit (8N1) */
    USART1.CTRLC = USART_CMODE_ASYNCHRONOUS_gc | USART_PMODE_DISABLED_gc | USART_SBMODE_1BIT_gc | USART_CHSIZE_8BIT_gc;

    /* Clear any pending status flags */
    USART1.STATUS = USART_TXCIF_bm | USART_RXCIF_bm;

    Terminal_tx_pending = false;

#if defined(__GNUC__)
    /* Redirect standard output stream (stdout) to USART1 terminal driver */
    stdout = &Terminal_stream;
#endif
}

void USART1_Initialize(void)
{
    Terminal_Initialize();
}

void Terminal_Enable(void)
{
    USART1.CTRLB |= USART_RXEN_bm | USART_TXEN_bm;
}

void USART1_Enable(void)
{
    Terminal_Enable();
}

void Terminal_EnableRx(void)
{
    USART1.CTRLB |= USART_RXEN_bm;
}

void USART1_ReceiveEnable(void)
{
    Terminal_EnableRx();
}

void Terminal_EnableTx(void)
{
    USART1.CTRLB |= USART_TXEN_bm;
}

void USART1_TransmitEnable(void)
{
    Terminal_EnableTx();
}

void Terminal_Disable(void)
{
    if (Terminal_tx_pending)
    {
        /* Wait for final bit to completely shift out of physical pin PA0 before disabling */
        while (!(USART1.STATUS & USART_TXCIF_bm))
            ;
        USART1.STATUS = USART_TXCIF_bm;
        Terminal_tx_pending = false;
    }
    USART1.CTRLB &= ~(USART_RXEN_bm | USART_TXEN_bm);
}

void USART1_Disable(void)
{
    Terminal_Disable();
}