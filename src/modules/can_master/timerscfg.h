#ifndef __TIMERSCFG_H__
#define __TIMERSCFG_H__
#include <stdint.h>

#define TIMEVAL uint32_t
#define TIMEVAL_MAX 0xFFFFFFFF
#define US_TO_TIMEVAL(us) ((us) * 1)
#define MS_TO_TIMEVAL(ms) (US_TO_TIMEVAL(ms) * 1000)

#endif