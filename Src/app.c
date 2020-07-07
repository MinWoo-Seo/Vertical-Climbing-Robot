
/**
 * @author Jihoon Jang
 */
 
// application source

#define I2C_DEVICE_ADDRESS 0xD0
#define RAD2DEG 57.2957786
#define PI 3.14159265
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)  //for printf
#include <math.h>
#include "app.h"
#include "main.h"
#include "stm32f4xx_it.h"
#include "stm32f4xx_hal_uart.h"
#include "stm32f4xx_hal_i2c.h"
extern I2C_HandleTypeDef hi2c1;
//#include "interrupt.h"
//FLAG flag = {0, 1};									// timer2 : false, angle_offset : true
unsigned long preTimeMPU;
uint8_t pData[1] = {0x01};
double gotXangle;
double gotYangle;
double angleX, angleY;
int flag_angle_offset=1;
extern UART_HandleTypeDef huart1;
extern unsigned long milliseconds;
// Standard Output String
/*
void SerialPutChar(uint8_t c);        // 1byte 보냄
void Serial_PutString(char *s);         // 여러 글 보냄

 void SerialPutChar(uint8_t Ch) // 1문자 보내기 함수
{
        while((USART3->SR & USART_SR_TXE) == RESET); //  USART_SR_TXE:0x0080, 송신 가능한 상태까지 대기   TX. DR 비웠는지
        // 비워있으면 while문 빠져 나옴
	USART3->DR = (Ch & 0x01FF);	// 전송                       //0x01FF 쓰레기값 버린거임
}

void Serial_PutString(char *str) // 여러문자 보내기 함수
{
	while (*str != '\0') // 종결문자가 나오기 전까지 구동, 종결문자가 나온후에도 구동시 메모리 오류 발생가능성 있음.
	{
		SerialPutChar(*str);	// 포인터가 가르키는 곳의 데이터를 송신
		str++; 			// 포인터 수치 증가
	}
}
*/

PUTCHAR_PROTOTYPE			// For printf Function
{
	/* Place your implementation of fputc here */
	/* e.g. write a character to the EVAL_COM1 and Loop until the end of transmission */
	HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xFFFF);   // USB
	return ch;
}

void printAngle(void)
{
	//if(flag.timer2){														// period : 100msec
	//	flag.timer2 = 0;
		//printf("X : %5.2f, Y : %5.2f \r\n", angleX, angleY);
                //SerialPutChar(angleX); 
             //  SerialPutChar(angleY+0x30);
	//}
}
float q1=0.90;    //0.98

void getAngle(void)
{
	double accXangle, accYangle;
	double gyroXrate, gyroYrate;
	double timePass;
	unsigned long present;
        //double A;
	//static const double k = 3.0;
	HAL_I2C_Mem_Write(&hi2c1, I2C_DEVICE_ADDRESS, 0x6B, I2C_MEMADD_SIZE_8BIT, pData, 1, 500);								// Set Mode
	getAccel(&accXangle, &accYangle);						// get accel value
	getGyro(&gyroXrate, &gyroYrate);						// get gyro value
	
	present = milliseconds;
	timePass = (present - preTimeMPU)/1000.0;
	preTimeMPU = present;
	//A = k/(k+timePass);
       // q^=1;
      //  A= 100000/(100001);
	//if(flag.angle_offset){
        
	if(flag_angle_offset){									// initial state
		flag_angle_offset = 0;
		gotXangle = accXangle;
		gotYangle = accYangle;
	}
	else{
		//gotXangle = A*(gotXangle + gyroXrate*timePass) + (1 - A)*accXangle;
		//gotYangle = A*(gotYangle + gyroYrate*timePass) + (1 - A)*accYangle;
                gotXangle = q1*(gotXangle + gyroXrate*timePass) + (1 - q1)*accXangle;
		gotYangle = q1*(gotYangle + gyroYrate*timePass) + (1 - q1)*accYangle;
	}
	angleX = gotXangle - 180.0;
	angleY = gotYangle - 180.0;
	//printAngle();
}

void getGyro(double *gyroXrate, double *gyroYrate)
{
	uint8_t buffer[6] = {0, };
	int16_t gyroX, gyroY, gyroZ;
	HAL_I2C_Mem_Read(&hi2c1, I2C_DEVICE_ADDRESS, 0x43, I2C_MEMADD_SIZE_8BIT, &buffer[0], 1, 500);
	HAL_I2C_Mem_Read(&hi2c1, I2C_DEVICE_ADDRESS, 0x44, I2C_MEMADD_SIZE_8BIT, &buffer[1], 1, 500);
	HAL_I2C_Mem_Read(&hi2c1, I2C_DEVICE_ADDRESS, 0x45, I2C_MEMADD_SIZE_8BIT, &buffer[2], 1, 500);
	HAL_I2C_Mem_Read(&hi2c1, I2C_DEVICE_ADDRESS, 0x46, I2C_MEMADD_SIZE_8BIT, &buffer[3], 1, 500);
	HAL_I2C_Mem_Read(&hi2c1, I2C_DEVICE_ADDRESS, 0x47, I2C_MEMADD_SIZE_8BIT, &buffer[4], 1, 500);
	HAL_I2C_Mem_Read(&hi2c1, I2C_DEVICE_ADDRESS, 0x48, I2C_MEMADD_SIZE_8BIT, &buffer[5], 1, 500);
	gyroX = (int)buffer[0] << 8 | (int)buffer[1];
	gyroY = (int)buffer[2] << 8 | (int)buffer[3];
	gyroZ = (int)buffer[4] << 8 | (int)buffer[5];
	*gyroXrate = (double)gyroX/131.0;
	*gyroYrate = -((double)gyroY/131.0);
}

void getAccel(double *accXangle, double *accYangle)
{
	uint8_t buffer[6] = {0, };
	int16_t accX, accY, accZ;
	HAL_I2C_Mem_Read(&hi2c1, I2C_DEVICE_ADDRESS, 0x3B, I2C_MEMADD_SIZE_8BIT, &buffer[0], 1, 500);
	HAL_I2C_Mem_Read(&hi2c1, I2C_DEVICE_ADDRESS, 0x3C, I2C_MEMADD_SIZE_8BIT, &buffer[1], 1, 500);
	HAL_I2C_Mem_Read(&hi2c1, I2C_DEVICE_ADDRESS, 0x3D, I2C_MEMADD_SIZE_8BIT, &buffer[2], 1, 500);
	HAL_I2C_Mem_Read(&hi2c1, I2C_DEVICE_ADDRESS, 0x3E, I2C_MEMADD_SIZE_8BIT, &buffer[3], 1, 500);
	HAL_I2C_Mem_Read(&hi2c1, I2C_DEVICE_ADDRESS, 0x3F, I2C_MEMADD_SIZE_8BIT, &buffer[4], 1, 500);
	HAL_I2C_Mem_Read(&hi2c1, I2C_DEVICE_ADDRESS, 0x40, I2C_MEMADD_SIZE_8BIT, &buffer[5], 1, 500);
	accX = (int)buffer[0] << 8 | (int)buffer[1];
	accY = (int)buffer[2] << 8 | (int)buffer[3];
	accZ = (int)buffer[4] << 8 | (int)buffer[5];
	*accXangle = (atan2(accY,accZ)+PI)*RAD2DEG;
	*accYangle = (atan2(accX,accZ)+PI)*RAD2DEG;
}

 // application source
 
 
