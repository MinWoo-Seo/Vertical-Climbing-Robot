#ifndef   __SSD1306_H__
#define   __SSD1306_H__

#include "stm32f4xx.h"
#include "FontInclude.h"

#ifdef __SSD1306_C__
	#define SSD1306_EXT
#else
	#define SSD1306_EXT extern
#endif

SSD1306_EXT void SSD1306_Init(void);
SSD1306_EXT void SSD1306_Clear(void);
SSD1306_EXT void SSD1306_DrawLineV(uint8_t x,uint8_t y,uint8_t leng);
SSD1306_EXT void SSD1306_DrawLineH(uint8_t x,uint8_t y,uint8_t leng);
SSD1306_EXT void SSD1306_Rect(uint8_t x,uint8_t y,uint8_t width,uint8_t height);
SSD1306_EXT void SSD1306_SetFont(sFONT *pFont);
SSD1306_EXT void SSD1306_DrawChar(uint8_t x, uint8_t y,uint8_t ch);
SSD1306_EXT void SSD1306_DrawText(uint16_t x, uint16_t y, char *str);
#endif


