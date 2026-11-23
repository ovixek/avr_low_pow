/**
 * TCD0 Generated Driver API Header File
 *
 * @file tcd0.h
 *
 * @defgroup tcd0 TCD0
 *
 * @brief This file contains the API prototypes for the TCD0 driver.
 *
 * @version TCD0 Driver Version 2.1.0
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


#ifndef TCD0_H_INCLUDED
#define TCD0_H_INCLUDED

#include "../system/utils/compiler.h"
#include "../system/ccp.h"
#include "./timer_interface.h"
#include "./tcd_types.h" 

extern const struct TMR_INTERFACE TCD0_Interface;

/**
 * @ingroup tcd0
 * @typedef void *TCD0_cb_t
 * @brief Function pointer to callback function called by the TCD. The default value is set to NULL which means that no callback function will be used.
 */ 
typedef void (*TCD0_cb_t)(void);
/**
 * @ingroup tcd0
 * @brief Initializes the TCD interface.
 * @param None.
 * @return None.
 */
void TCD0_Initialize();
/**
 * @ingroup tcd0
 * @brief Starts the 12-bit timer/counter for TCD interface.
 * @param None.
 * @return None.
 */
void TCD0_Start();
/**
 * @ingroup tcd0
 * @brief Stops the 12-bit timer/counter for TCD interface.
 * @param None.
 * @return None.
 */
void TCD0_Stop();
/**
 * @ingroup tcd0
 * @brief Sets the Wave mode for the TCD.
 * @pre Initialize the TCD before calling this API.
 * @param TCD_WGMODE_t mode:
 * - @c TCD_WGMODE_ONERAMP_gc   @c  - One ramp mode
 * - @c TCD_WGMODE_TWORAMP_gc   @c  - Two ramp mode
 * - @c TCD_WGMODE_FOURRAMP_gc  @c  - Four ramp mode
 * - @c TCD_WGMODE_DS_gc        @c  - Dual Slope mode
 * @return None.
 *
 */
void TCD0_ModeSet (TCD_WGMODE_t mode);
/**
 * @ingroup tcd0
 * @brief Sets the software command for the TCD.
 * @pre Initialize the TCD before calling this API.
 * @param uint8_t com:
 * - @c TCD_SYNCEOC_bm          @c  - Synchronize end of cycle strobe bit mask
 * - @c TCD_SYNC_bm             @c  - synchronize strobe bit mask
 * - @c TCD_RESTART_bm          @c  - Restart strobe bit mask
 * - @c TCD_SCAPTUREA_bm        @c  - Software Capture A Strobe bit mask
 * - @c TCD_SCAPTUREB_bm        @c  - Software Capture B Strobe bit mask
 * - @c TCD_DISEOC_bm           @c  - Disable at end of cycle bit mask
 * @return None.
 *
 */
void TCD0_SoftwareCommand (uint8_t com);
/**
 * @ingroup tcd0
 * @brief Interrupt Service Routine (ISR) callback function to be called if the Trigger Input Interrupt flag is set.
 * @param TCD0_cb_t cb - Callback function for the trigger input event.
 * @return None.
 */ 
void TCD0_SetTRIGIsrCallback(TCD0_cb_t cb);
/**
 * @ingroup tcd0
 * @brief ISR callback function to be called if the Counter Overflow flag is set.
 * @param TCD0_cb_t cb - Callback function for the counter overflow event.
 * @return None.
 */ 
void TCD0_SetOVFIsrCallback(TCD0_cb_t cb);

/**
 * @ingroup tcd0_normal
 * @brief Performs the tasks to be executed on timer events.
 * @param None.
 * @return None.
 */
void TCD0_Tasks();
/**
 * @ingroup tcd0
 * @brief Writes a compare count value to the selected TCD compare SET or CLEAR register.
 * @pre TCD0_CommandReady() must return true before calling this API.
 * @param TCD_CHANNEL_t channel - Selects the compare channel.
 * - @c TCD_CHANNEL_A - Compare A channel
 * - @c TCD_CHANNEL_B - Compare B channel
 * @param TCD_COMPARE_Type_t type - Selects the compare register type.
 * - @c SET - Writes to the compare SET register
 * - @c CLEAR - Writes to the compare CLEAR register
 * @param uint16_t count - Compare count value to write to the selected register.
 * @return None.
 */
void TCD0_CompareCountWrite(TCD_CHANNEL_t channel,TCD_COMPARE_Type_t type,
                                         uint16_t count);
/**
 * @ingroup tcd0
 * @brief Checks whether the TCD compare registers are ready for an update.
 * This API checks the Command Ready (CMDRDY) bit in the TCD STATUS register.
 * The application should call this API before calling
 * TCD0_CompareCountWrite().
 * @return Ready status of the TCD compare registers:
 * - @retval True  - The TCD is ready for a compare register update.
 * - @retval False - The TCD is not ready for a compare register update.
 */
bool TCD0_CommandReady(void);

#endif /* TCD0_H_INCLUDED */
