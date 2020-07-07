#define __UTIL_C__
#include "util.h"
#undef  __UTIL_C__

void UTIL_DelayMS(UINT16 wMS)
{
    register UINT16 i;

    for (i=0; i<wMS; i++)
        UTIL_DelayUS(1000);         // 1000us => 1ms
}

void UTIL_DelayUS(UINT16 wUS)
{
    volatile UINT32 Dly = (UINT32)wUS*6;
    for(; Dly; Dly--);
}


static char hex_ascii_tab[16] = {
	'0','1','2','3','4','5','6','7','8','9',
	'A','B','C','D','E','F'
};


UINT8 UTIL_Hex2Asc(UINT8 x)
{
	return (UINT8)(*(hex_ascii_tab + x));
}

UINT8 UTIL_Asc2Hex(UINT8 x)
{
	UINT8 i;
		
	for(i=0;i<16;i++)
	{
		if(*(hex_ascii_tab + i)==x) return i;
	}	
	return 0;
}

typedef union
{
  UINT8  Byte[2];
  UINT16 Word;
}BYTE2_UNION;

UINT16 UTIL_B2W(UINT8 *pBuf)
{
      BYTE2_UNION pos;
      
      pos.Byte[0] = pBuf[0];
      pos.Byte[1] = pBuf[1];
        
      return pos.Word;  
}
  
void Delay(__IO uint32_t nTime)
{ 
  uwTimingDelay = nTime;

  while(uwTimingDelay != 0);
}

void TimingDelay_Decrement(void)
{
  if (uwTimingDelay != 0x00)
  { 
    uwTimingDelay--;
  }
}
