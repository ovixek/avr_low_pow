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
 * AVR128DA48 USART1 Driver Header - usart1.h
 * ============================================================================
 * Defines macros, baud rate calculation formula, buffer configuration,
 * and API function prototypes for USART1 serial communications.
 * ============================================================================
 */

#ifndef USART1_H_INCLUDED
#define USART1_H_INCLUDED

#include <stdbool.h>
#include <stdio.h>
#include "mcc_generated_files/system/utils/compiler.h"
#include "mcc_generated_files/system/utils/atomic.h"
#include "mcc_generated_files/system/clock.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Baud rate register value calculation formula for Asynchronous Normal Mode:
 * Formula: BAUD = ((64 * f_CLK_PER) / (16 * BaudRate)) + 0.5
 *   - f_CLK_PER = 24,000,000 Hz (24 MHz OSCHF)
 *   - 64 = hardware 6-bit fractional multiplier
 *   - 16 = oversampling ratio (16 clock cycles per bit)
 *   - +0.5 = rounding up for integer truncation
 * For 115200 baud: BAUD = (64 * 24000000) / (16 * 115200) + 0.5 = ~833.3 -> 833 (0x0341)
 */
#define USART1_BAUD_RATE(BAUD_RATE) (((float)24000000 * 64 / (16 * (float)BAUD_RATE)) + 0.5)

/*
 * Circular Ring Buffer Configuration:
 * Buffer sizes MUST be powers of 2 (e.g. 8 = 2^3).
 * This enables the compiler to optimize modulo wrapping using a bitwise AND:
 *   index = (index + 1) & MASK
 * avoiding costly division instructions on the 8-bit AVR core.
 */
#define Terminal_RX_BUFFER_SIZE 8
#define Terminal_TX_BUFFER_SIZE 8
#define Terminal_RX_BUFFER_MASK (Terminal_RX_BUFFER_SIZE - 1) /* 0x07 (0b00000111) */
#define Terminal_TX_BUFFER_MASK (Terminal_TX_BUFFER_SIZE - 1) /* 0x07 (0b00000111) */

/* Callback type enumeration for registering custom ISR handlers */
typedef enum { Terminal_RX_CB = 1, Terminal_TX_CB } terminal_cb_t;
typedef void (*usart_callback)(void);

/**
 * Terminal_Initialize()
 * Initializes USART1 hardware registers, ring buffers, and binds stdout.
 */
void Terminal_Initialize(void);

/**
 * Terminal_Enable()
 * Enables both receiver and transmitter circuits.
 */
void Terminal_Enable(void);

/**
 * Terminal_EnableRx()
 * Enables USART1 receiver (RXEN = 1).
 */
void Terminal_EnableRx(void);

/**
 * Terminal_EnableTx()
 * Enables USART1 transmitter (TXEN = 1).
 */
void Terminal_EnableTx(void);

/**
 * Terminal_Disable()
 * Flushes queued data and disables receiver and transmitter.
 * Call with global interrupts enabled.
 */
void Terminal_Disable(void);

/**
 * Terminal_GetData()
 * Direct non-blocking read of raw data register USART1.RXDATAL.
 */
uint8_t Terminal_GetData(void);

/**
 * Terminal_IsTxReady()
 * Returns true if the transmit buffer has room for at least one more byte.
 */
bool Terminal_IsTxReady(void);

/**
 * Terminal_IsRxReady()
 * Returns true if the receive buffer contains at least one unread byte.
 */
bool Terminal_IsRxReady(void);

/**
 * Terminal_IsTxBusy()
 * Returns true if a byte is currently being shifted out over the serial wire.
 */
bool Terminal_IsTxBusy(void);

/**
 * Terminal_IsTxDone()
 * Returns true when transmission is completely finished.
 */
bool Terminal_IsTxDone(void);

/**
 * Terminal_Read()
 * Blocking read: waits until a character is available in the RX FIFO, then returns it.
 */
uint8_t Terminal_Read(void);

/**
 * Terminal_Write(data)
 * Queues a byte for transmission into the TX FIFO and enables DRE interrupt.
 */
void Terminal_Write(const uint8_t data);

/** Default Receive Complete ISR callback */
void Terminal_DefaultRxIsrCb(void);

/** Default Data Register Empty ISR callback */
void Terminal_DefaultTxIsrCb(void);

/** Registers a callback for either RX or TX interrupt */
void Terminal_SetISRCb(usart_callback cb, terminal_cb_t type);

/** Registers a custom RX Complete interrupt callback */
void Terminal_SetRXISRCb(usart_callback cb);

/** Registers a custom TX Data Register Empty interrupt callback */
void Terminal_SetTXISRCb(usart_callback cb);

#ifdef __cplusplus
}
#endif

#endif /* USART1_H_INCLUDED */