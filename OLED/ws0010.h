#ifndef   __WS0010_H__
#define   __WS0010_H__

#include "stm32f4xx.h"


#ifdef __WS0010_C__
	#define WS0010_EXT
#else
	#define WS0010_EXT extern
#endif

WS0010_EXT void WS0010_Init(void);
WS0010_EXT void WS0010_Put_Str(uint8_t pos,uint8_t line,uint8_t* str);
WS0010_EXT void WS0010_Clear(void);
#endif
