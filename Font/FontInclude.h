#ifndef   __DEFFONT_H__
#define   __DEFFONT_H__

#include "stm32f4xx.h"

#define uint16_t unsigned short

typedef struct _tFont
{    
  const uint8_t *table;
  uint16_t Width;
  uint16_t Height;
  uint16_t count;
} sFONT;

#include "Gulim7_6x10_ASCII.h"
#include "Gulim10_8x16_ASCII.h"
#include "Gulim12_9x16_ASCII.h"
#include "Gulim15_11x20_ASCII.h"
#include "Gulim20_16x34_ASCII.h"

#endif

