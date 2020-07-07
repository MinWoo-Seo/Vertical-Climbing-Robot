
/**
 * @author Jihoon Jang
 */

// add.h header

#ifndef _APP_H_
#define _APP_H_

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_i2c.h"
#include "stm32f4xx_hal_uart.h"
//#include "define.h"
#include <math.h>
//#include "interrupt.h"

//extern FLAG flag;
extern unsigned long preTimeMPU;
extern double gotXangle;
extern double gotYangle;
extern double angleX, angleY;


#ifdef __GNUC__
  /* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)								/* for printf */
#endif /* __GNUC__ */

/* IMU Routines */
void getAngle(void);
void getAccel(double *accXangle, double *accYangle);
void getGyro(double *accXangle, double *accYangle);
void printAngle(void);

#endif	// add.h
