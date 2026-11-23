#ifndef SLPCTRL_INCLUDED
#define SLPCTRL_INCLUDED

#include "../../system/utils/compiler.h" 

#ifdef __cplusplus
extern "C" {
#endif

int8_t SLPCTRL_init();

void SLPCTRL_set_sleep_mode(SLPCTRL_SMODE_t setmode);

#ifdef __cplusplus
}
#endif

#endif /* SLPCTRL_INCLUDED */
