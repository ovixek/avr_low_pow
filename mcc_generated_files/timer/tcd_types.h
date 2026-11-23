/**
 * TCD Generated Driver Types Header File
 *
 * @file tcd_types.h
 *
 * @defgroup tcd_types TCD Types
 *
 * @brief This file contains the data types for the TCD driver.
 *
 * @version TCD Types Version 1.0.0
 */

/*
© [2026] Microchip Technology Inc. and its subsidiaries.

    Subject to your compliance with these terms, you may use Microchip 
    software and any derivatives exclusively with Microchip products. 
    You are responsible for complying with 3rd party license terms  
    applicable to your use of 3rd party software (including open source  
    software) that may accompany Microchip software. SOFTWARE IS ?AS IS.? 
    NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS 
    SOFTWARE, INCLUDING ANY IMPLIED WARRANTIES OF NON-INFRINGEMENT,  
    MERCHANTABILITY, OR FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT 
    WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, 
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY 
    KIND WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF 
    MICROCHIP HAS BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE 
    FORESEEABLE. TO THE FULLEST EXTENT ALLOWED BY LAW, MICROCHIP?S 
    TOTAL LIABILITY ON ALL CLAIMS RELATED TO THE SOFTWARE WILL NOT 
    EXCEED AMOUNT OF FEES, IF ANY, YOU PAID DIRECTLY TO MICROCHIP FOR 
    THIS SOFTWARE.
*/

#ifndef TCD_TYPES_H_INCLUDED
#define TCD_TYPES_H_INCLUDED

/**
 * @ingroup tcd_types
 * @brief TCD compare channel selection.
 * This enumeration specifies the TCD compare channel
 * used for compare register operations.
 * - @c TCD_CHANNEL_A - Compare A channel
 * - @c TCD_CHANNEL_B - Compare B channel
 */
typedef enum
{
    TCD_CHANNEL_A,
    TCD_CHANNEL_B
} TCD_CHANNEL_t;

/**
 * @ingroup tcd_types
 * @brief TCD compare register type selection.
 * This enumeration specifies whether the compare SET
 * register or compare CLEAR register is selected.
 * - @c SET   - Writes to the compare SET register
 * - @c CLEAR - Writes to the compare CLEAR register
 */
typedef enum
{
    SET,
    CLEAR
} TCD_COMPARE_Type_t;

#endif /* TCD_TYPES_H_INCLUDED */
