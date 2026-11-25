/**
 * ============================================================================
 * AVR128DA48 USART1 Driver Header - usart1.h
 * ============================================================================
 * Defines macros, baud rate calculation formula, and API prototypes for
 * USART1 serial communications on PA0 (TXD) and PA1 (RXD).
 * ============================================================================
 */

#ifndef USART1_H_INCLUDED
#define USART1_H_INCLUDED

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "mcc_generated_files/system/utils/compiler.h"
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

/**
 * Terminal_Initialize() / USART1_Initialize()
 * Initializes USART1 hardware registers on PA0 (TXD) and PA1 (RXD),
 * configures 115200 baud, 8N1, and binds stdout for printf() redirection.
 */
void Terminal_Initialize(void);
void USART1_Initialize(void);

/**
 * Terminal_Enable() / USART1_Enable()
 * Enables both receiver and transmitter circuits.
 */
void Terminal_Enable(void);
void USART1_Enable(void);

/**
 * Terminal_EnableRx() / USART1_ReceiveEnable()
 * Enables USART1 receiver (RXEN = 1).
 */
void Terminal_EnableRx(void);
void USART1_ReceiveEnable(void);

/**
 * Terminal_EnableTx() / USART1_TransmitEnable()
 * Enables USART1 transmitter (TXEN = 1).
 */
void Terminal_EnableTx(void);
void USART1_TransmitEnable(void);

/**
 * Terminal_Disable() / USART1_Disable()
 * Waits until all active transmission is completely finished on PA0,
 * then disables transmitter and receiver for power savings.
 */
void Terminal_Disable(void);
void USART1_Disable(void);

/**
 * Terminal_GetData()
 * Direct non-blocking read of raw data register USART1.RXDATAL.
 */
uint8_t Terminal_GetData(void);

/**
 * Terminal_IsTxReady() / USART1_IsTxReady()
 * Returns true if transmit data register is empty and ready for next byte.
 */
bool Terminal_IsTxReady(void);
bool USART1_IsTxReady(void);

/**
 * Terminal_IsRxReady() / USART1_IsRxReady()
 * Returns true if at least one unread byte has arrived.
 */
bool Terminal_IsRxReady(void);
bool USART1_IsRxReady(void);

/**
 * Terminal_IsTxBusy()
 * Returns true if a byte is currently being shifted out over the serial wire.
 */
bool Terminal_IsTxBusy(void);

/**
 * Terminal_IsTxDone() / USART1_IsTxDone()
 * Returns true when transmission of the entire frame (including stop bits) is complete.
 */
bool Terminal_IsTxDone(void);
bool USART1_IsTxDone(void);

/**
 * Terminal_Read() / USART1_Read()
 * Blocking read: waits until a character is available in the RX register, then returns it.
 */
uint8_t Terminal_Read(void);
uint8_t USART1_Read(void);

/**
 * Terminal_Write(data) / USART1_Write(data)
 * Transmits a byte over USART1 directly to the hardware data register.
 */
void Terminal_Write(const uint8_t data);
void USART1_Write(const uint8_t data);

/**
 * Terminal_WriteString(str)
 * Transmits a null-terminated string over USART1.
 */
void Terminal_WriteString(const char *str);

#ifdef __cplusplus
}
#endif

#endif /* USART1_H_INCLUDED */