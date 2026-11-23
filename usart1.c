/**
  @Company
    Microchip Technology Inc.

  @Description
    This Source file provides APIs.
    Generation Information :
    Driver Version    :   1.0.0
*/
/*
    (c) 2018 Microchip Technology Inc. and its subsidiaries. 
    
    Subject to your compliance with these terms, you may use Microchip software and any 
    derivatives exclusively with Microchip products. It is your responsibility to comply with third party 
    license terms applicable to your use of third party software (including open source software) that 
    may accompany Microchip software.
    
    THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER 
    EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY 
    IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS 
    FOR A PARTICULAR PURPOSE.
    
    IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, 
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND 
    WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP 
    HAS BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO 
    THE FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL 
    CLAIMS IN ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT 
    OF FEES, IF ANY, THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS 
    SOFTWARE.
*/

/*
 * ============================================================================
 * AVR128DA48 USART1 Serial Communication Driver - usart1.c
 * ============================================================================
 * Provides an interrupt-driven serial driver for USART1 (Curiosity Nano Virtual COM):
 *   - 115200 baud, 8 data bits, no parity, 1 stop bit (8N1).
 *   - Circular ring buffer (FIFO) for both transmit and receive.
 *   - GCC standard I/O (stdio) stream binding: redirects printf() to USART1.
 *   - Sleep Mode Wakeup Support:
 *     - Bit 4 RXSIE (Receiver Start Frame Interrupt Enable) in USART1.CTRLA
 *       allows an incoming start bit on the RX line to wake up the MCU from
 *       Standby sleep mode!
 * ============================================================================
 */

#include "usart1.h"

#if defined(__GNUC__)

/**
 * Terminal_printCHAR()
 * Callback used by AVR libc's standard I/O stream system.
 * Routes characters passed to printf() or puts() into the USART1 transmit FIFO.
 */
int Terminal_printCHAR(char character, FILE *stream)
{
    Terminal_Write(character);
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

/*
 * ============================================================================
 * Circular Ring Buffer Variables (FIFO)
 * ============================================================================
 */
/* Receive buffer storage: size = 8 bytes */
static uint8_t          Terminal_rxbuf[Terminal_RX_BUFFER_SIZE];
static volatile uint8_t Terminal_rx_head;     /* Write index (updated by RX ISR) */
static volatile uint8_t Terminal_rx_tail;     /* Read index (updated by Terminal_Read) */
static volatile uint8_t Terminal_rx_elements; /* Number of received bytes currently in FIFO */

/* Transmit buffer storage: size = 8 bytes */
static uint8_t          Terminal_txbuf[Terminal_TX_BUFFER_SIZE];
static volatile uint8_t Terminal_tx_head;     /* Write index (updated by Terminal_Write) */
static volatile uint8_t Terminal_tx_tail;     /* Read index (updated by TX ISR) */
static volatile uint8_t Terminal_tx_elements; /* Number of bytes queued for transmission */

/* Function pointers for custom runtime ISR callbacks */
void (*Terminal_rx_isr_cb)(void) = &Terminal_DefaultRxIsrCb;
void (*Terminal_tx_isr_cb)(void) = &Terminal_DefaultTxIsrCb;

/**
 * Terminal_DefaultRxIsrCb()
 * Default Receive Complete Callback (called from ISR(USART1_RXC_vect)):
 *   - Reads received byte from hardware data register USART1.RXDATAL.
 *   - Inserts byte into the circular ring buffer.
 *   - Increments buffer element count.
 */
void Terminal_DefaultRxIsrCb(void)
{
    uint8_t data;
    uint8_t tmphead;

    /* Read the received byte from hardware register (clears RXCIF flag) */
    data = USART1.RXDATAL;

    /* Calculate next head index using bitwise modulo wrap-around (mask = size - 1 = 7) */
    tmphead = (Terminal_rx_head + 1) & Terminal_RX_BUFFER_MASK;
        
    /* Check for buffer overflow condition */
    if (tmphead == Terminal_rx_tail) {
        /* Buffer is full: incoming byte is dropped to prevent corrupting unread data */
    } else {
        /* Advance head index */
        Terminal_rx_head = tmphead;
        
        /* Store received byte in FIFO buffer */
        Terminal_rxbuf[tmphead] = data;

        /* Increment unread elements count */
        Terminal_rx_elements++;
    }
}

/**
 * Terminal_DefaultTxIsrCb()
 * Default Data Register Empty Callback (called from ISR(USART1_DRE_vect)):
 *   - Triggered whenever USART1 transmit register is empty and ready for next byte.
 *   - Fetches next byte from FIFO and writes it to USART1.TXDATAL.
 *   - When FIFO becomes empty, disables DREIE to prevent continuous interrupt firing!
 */
void Terminal_DefaultTxIsrCb(void)
{
    uint8_t tmptail;

    /* Check if there are pending bytes to transmit */
    if (Terminal_tx_elements != 0) {
        /* Calculate next tail index with wrap-around */
        tmptail = (Terminal_tx_tail + 1) & Terminal_TX_BUFFER_MASK;

        /* Update tail index */
        Terminal_tx_tail = tmptail;

        /* Write byte to hardware TXDATAL register; begins serial transmission */
        USART1.TXDATAL = Terminal_txbuf[tmptail];
        
        /* Decrement pending elements counter */
        Terminal_tx_elements--;
    }

    /* If all queued bytes have been sent, disable Data Register Empty interrupt */
    if (Terminal_tx_elements == 0) {
        /* Clear DREIE bit in CTRLA so the CPU is not interrupted when idle */
        USART1.CTRLA &= ~(1 << USART_DREIE_bp);
    }
}

/**
 * Terminal_SetISRCb()
 * Registers custom RX or TX callback functions at runtime.
 */
void Terminal_SetISRCb(usart_callback cb, terminal_cb_t type)
{
    switch (type) {
        case Terminal_RX_CB:
                Terminal_rx_isr_cb = cb;
                break;
        case Terminal_TX_CB:
                Terminal_tx_isr_cb = cb;
                break;
        default:
                break;
    }
}

void Terminal_SetRXISRCb(usart_callback cb)
{
    Terminal_SetISRCb(cb, Terminal_RX_CB);
}

void Terminal_SetTXISRCb(usart_callback cb)
{
    Terminal_SetISRCb(cb, Terminal_TX_CB);
}

/**
 * ISR(USART1_RXC_vect)
 * Interrupt Service Routine for USART1 Receive Complete.
 * Fires when a complete serial byte has arrived into the receive register.
 */
ISR(USART1_RXC_vect)
{
    if (Terminal_rx_isr_cb != NULL)
    {
        (*Terminal_rx_isr_cb)();
    }
}

/**
 * ISR(USART1_DRE_vect)
 * Interrupt Service Routine for USART1 Data Register Empty.
 * Fires when TXDATAL is ready to accept a new character.
 */
ISR(USART1_DRE_vect)
{
    if (Terminal_tx_isr_cb != NULL)
    {
        (*Terminal_tx_isr_cb)();
    }
}

/**
 * ISR(USART1_TXC_vect)
 * Interrupt Service Routine for USART1 Transmit Complete.
 * Fires when the entire byte (including stop bits) has shifted completely out of the pin.
 */
ISR(USART1_TXC_vect)
{
    /* Clear the transmit complete flag by writing a 1 */
    USART1.STATUS |= USART_TXCIF_bm;
}

/**
 * Terminal_IsTxReady()
 * Returns true if there is available space in the TX ring buffer.
 */
bool Terminal_IsTxReady()
{
    return (Terminal_tx_elements != Terminal_TX_BUFFER_SIZE);
}

/**
 * Terminal_IsRxReady()
 * Returns true if at least one character is waiting in the RX ring buffer.
 */
bool Terminal_IsRxReady()
{
    return (Terminal_rx_elements != 0);
}

/**
 * Terminal_IsTxBusy()
 * Returns true if data is currently being shifted out of the hardware shift register.
 */
bool Terminal_IsTxBusy()
{
    return (!(USART1.STATUS & USART_TXCIF_bm));
}

/**
 * Terminal_IsTxDone()
 * Returns true if transmission of all bits is fully complete.
 */
bool Terminal_IsTxDone()
{
    return (USART1.STATUS & USART_TXCIF_bm);
}

/**
 * Terminal_Read()
 * Reads one byte from the receive circular buffer.
 * If the buffer is empty, it blocks (busy-waits) until an incoming byte arrives.
 *
 * Uses ENTER_CRITICAL / EXIT_CRITICAL to atomically update Terminal_rx_elements,
 * preventing race conditions with the receive interrupt (USART1_RXC_vect).
 */
uint8_t Terminal_Read(void)
{
    uint8_t tmptail;

    /* Busy-wait until at least one byte has been received */
    while (Terminal_rx_elements == 0)
            ;

    /* Calculate next tail index with circular wrap-around */
    tmptail = (Terminal_rx_tail + 1) & Terminal_RX_BUFFER_MASK;

    /* Update tail pointer */
    Terminal_rx_tail = tmptail;

    /* Atomically decrement element count to prevent interrupt race condition */
    ENTER_CRITICAL(R);
    Terminal_rx_elements--;
    EXIT_CRITICAL(R);

    /* Return character read from FIFO */
    return Terminal_rxbuf[tmptail];
}

/**
 * Terminal_Write(data)
 * Queues one byte into the transmit circular buffer.
 * If the buffer is full, it blocks until space is freed by the TX ISR.
 *
 * Enables the Data Register Empty interrupt (DREIE) to begin serial transmission.
 */
void Terminal_Write(const uint8_t data)
{
    uint8_t tmphead;

    /* Calculate next head index */
    tmphead = (Terminal_tx_head + 1) & Terminal_TX_BUFFER_MASK;

    /* Wait if buffer is completely full */
    while (Terminal_tx_elements == Terminal_TX_BUFFER_SIZE)
            ;

    /* Place byte in FIFO */
    Terminal_txbuf[tmphead] = data;

    /* Update head index */
    Terminal_tx_head = tmphead;

    /* Atomically increment elements count */
    ENTER_CRITICAL(W);
    Terminal_tx_elements++;
    EXIT_CRITICAL(W);

    /* Enable DRE interrupt: triggers ISR(USART1_DRE_vect) to begin sending */
    USART1.CTRLA |= (1 << USART_DREIE_bp);
}

/**
 * Terminal_Initialize()
 * Configures the USART1 peripheral:
 *   - Calculates and writes 16-bit fractional baud rate register for 115200 baud.
 *   - CTRLA: Enables RX complete, TX complete, and Receiver Start Frame interrupt (RXSIE).
 *   - CTRLB: Enables Receiver (RXEN) and Transmitter (TXEN) in normal speed mode.
 *   - CTRLC: Sets asynchronous mode, 8-bit character size, no parity, 1 stop bit.
 *   - Initializes FIFO indices to 0.
 *   - Binds stdout stream to Terminal_stream for printf() output.
 */
void Terminal_Initialize()
{
    /* Set baud rate register for 115200 baud (calculated from 24 MHz F_CPU) */
    USART1.BAUD = (uint16_t)USART1_BAUD_RATE(115200);
	
    /*
     * USART1.CTRLA - Control A Register: 0xD0 = 0b11010000
     * - Bit 7 RXCIE = 1 : Receive Complete Interrupt Enable.
     * - Bit 6 TXCIE = 1 : Transmit Complete Interrupt Enable.
     * - Bit 5 DREIE = 0 : Data Register Empty Interrupt disabled until bytes are queued.
     * - Bit 4 RXSIE = 1 : Receiver Start Frame Interrupt Enable.
     *                     Enables detection of incoming start bit to wake the device
     *                     from Standby sleep mode!
     * - Bits [3:0]      : RS485 / loopback disabled.
     */
    USART1.CTRLA = 0xD0;
	
    /*
     * USART1.CTRLB - Control B Register: 0xC0 = 0b11000000
     * - Bit 7 RXEN = 1  : Receiver enabled.
     * - Bit 6 TXEN = 1  : Transmitter enabled.
     * - Bit 4 SFDEN = 0 : Start Frame Detection mode normal.
     * - Bits [2:1] RXMODE = 0 : Normal speed mode (16x oversampling).
     */
    USART1.CTRLB = 0xC0;
	
    /*
     * USART1.CTRLC - Control C Register: 0x03 = 0b00000011
     * - Bits [7:6] CMODE = 0x00 : Asynchronous USART communication.
     * - Bits [5:4] PMODE = 0x00 : Parity disabled.
     * - Bit 3 SBMODE = 0        : 1 Stop Bit.
     * - Bits [2:0] CHSIZE = 0x03: 8-bit character size.
     */
    USART1.CTRLC = 0x03;
	
    /* Debug Control: 0x00 (USART halts when debugger halts CPU) */
    USART1.DBGCTRL = 0x00;
	
    /* Event Control: 0x00 (Event system input disabled) */
    USART1.EVCTRL = 0x00;
	
    /* RX Pulse Length Control: 0x00 (No pulse length filtering) */
    USART1.RXPLCTRL = 0x00;
	
    /* TX Pulse Length Control: 0x00 */
    USART1.TXPLCTRL = 0x00;

    /* Initialize ring buffer pointers and element counts to 0 */
    uint8_t x = 0;
    Terminal_rx_tail     = x;
    Terminal_rx_head     = x;
    Terminal_rx_elements = x;
    Terminal_tx_tail     = x;
    Terminal_tx_head     = x;
    Terminal_tx_elements = x;

#if defined(__GNUC__)
    /* Redirect standard output stream (stdout) to USART1 terminal driver */
    stdout = &Terminal_stream;
#endif
}

/**
 * Terminal_Enable()
 * Enables both receiver and transmitter circuits.
 */
void Terminal_Enable()
{
    USART1.CTRLB |= USART_RXEN_bm | USART_TXEN_bm;
}

/**
 * Terminal_EnableRx()
 * Enables USART1 receiver only.
 */
void Terminal_EnableRx()
{
    USART1.CTRLB |= USART_RXEN_bm;
}

/**
 * Terminal_EnableTx()
 * Enables USART1 transmitter only.
 */
void Terminal_EnableTx()
{
    USART1.CTRLB |= USART_TXEN_bm;
}

/**
 * Terminal_Disable()
 * Disables both receiver and transmitter to conserve power.
 */
void Terminal_Disable()
{
    USART1.CTRLB &= ~(USART_RXEN_bm | USART_TXEN_bm);
}

/**
 * Terminal_GetData()
 * Direct non-blocking read of raw data register USART1.RXDATAL.
 */
uint8_t Terminal_GetData()
{
    return USART1.RXDATAL;
}