#ifndef   __UTIL_H__
#define   __UTIL_H__

#include "default.h"

#ifdef __UTIL_C__
	#define UTIL_EXT
#else
	#define UTIL_EXT extern
#endif


UTIL_EXT void   UTIL_DelayMS(UINT16 wMS);
UTIL_EXT void   UTIL_DelayUS(UINT16 wUS);
UTIL_EXT UINT8  UTIL_Hex2Asc(UINT8 x);
UTIL_EXT UINT8  UTIL_Asc2Hex(UINT8 x);
UTIL_EXT UINT16 UTIL_B2W(UINT8 *pBuf);

UTIL_EXT void Delay(__IO uint32_t nTime);
UTIL_EXT __IO uint32_t uwTimingDelay;
#endif


