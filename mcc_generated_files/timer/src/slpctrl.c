/*
 * File:   slpctrl.c
 * Author: Asus
 *
 * Created on October 7, 2026, 1:28 PM
 */

#include "slpctrl.h"
/**
 * \brief Initialize Sleep Controller
 *
 * \return Initialization status.
 */
int8_t SLPCTRL_init()
{

	// SLPCTRL.CTRLA = 0 << SLPCTRL_SEN_bp /* Sleep enable: disabled */
	//		 | SLPCTRL_SMODE_IDLE_gc; /* Idle mode */

	return 0;
}

void SLPCTRL_set_sleep_mode(SLPCTRL_SMODE_t setmode)
{
	SLPCTRL.CTRLA = (SLPCTRL.CTRLA & ~SLPCTRL_SMODE_gm) | (setmode & SLPCTRL_SMODE_gm);
}
