/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2019 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ssd1306.h"
#include "ws0010.h"
#include "app.h"
#include <stdio.h>
#include <math.h>
//#include "sd_hal_mpu6050.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#if 0
  BLDC1 : PC6  
  BLDC2 : PC7  
    
  SERVO1 : PE5  
  SERVO2 : PE6
    
  Right : PH10  //TIM5 ch1
  Left   : PH11  //TIM5 ch2
  
  ADC1 : PA1   
  ADC2 : PA2
  
  i2C1_SCL : PB8
  i2C1_SDA: PB9
    
USART1_TX : PA9
USART1_RX : PA10

USART3_TX : PC10
USART3_RX : PC11

#endif
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim9;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;

//osThreadId defaultTaskHandle;
/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM9_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM4_Init(void);
static void MX_ADC2_Init(void);
static void MX_TIM5_Init(void);
//void StartDefaultTask(void const * argument);

/* USER CODE BEGIN PFP */


char str[20];
char str1[20];
double servo;    //PE5, PE6   servo(50~270Hz :0~180도)

//unsigned long milliseconds;
unsigned long milliseconds;

uint8_t ADC1_Flag=0;
uint8_t ADC2_Flag=0;
uint8_t ADC_Change=0;

uint8_t MPU6050_Flag=0;
uint8_t MPU6050_Flag2=0;
int32_t Angle,Angle1;
int32_t SetAngle;

uint16_t ADC_Value;     //PA1
uint16_t Voltage, distance_cm;  // 입력전압 5v일 때

uint16_t BLDC_Velocity=0;  // BLDC 속도
uint16_t BLDC_Velocity1=1050-2;//=1100-2;  // BLDC 속도 처음 벽 탈 때
uint16_t BLDC_Velocity2 =1050-2;//=1700;

int bldc_V=0;

int bldc_V_flag=0;

int Servomotor_Angle1=50;  // 서보모터1 주파수
int Servomotor_Angle2=50;  // 서보모터 2주파수

uint16_t Real_Ang=0;               // 서보모터 각도
//uint16_t ADC_MPU_Flag=1;
uint8_t Wall_Ang_Flag1=1;     // 벽타는 과정 BLDC 속도 제어1
uint8_t Wall_Ang_Flag2=1;     // 벽타는 과정 BLDC 속도 제어2
signed short x1,y1;

int angle45_flag=0;             // 45도 이하면 플래그
int angle180=0;

int BLDC_TIME=0;
int BLDC_TIME1=0;
int BLDC_TIME2=0;
int BLDC_TIME3=0;
int BLDC_TIME4=0;

#if 0              //서보모터 1개일 때
int direction=90;      // 조향 모터  각도
int center=2;           // 조향 모터  중심
int Right_and_left=0; //  조향 모터 주파수
#endif

#if 1
int center=2;  
int Right_angle=90;
int Left_angle=90;
#endif


void SerialPutChar(uint8_t c);        // 1byte 보냄
 void SerialPutChar(uint8_t Ch) // 1문자 보내기 함수
{
        while((USART1->SR & USART_SR_TXE) == RESET); //  USART_SR_TXE:0x0080, 송신 가능한 상태까지 대기   TX. DR 비웠는지
        // 비워있으면 while문 빠져 나옴
	USART1->DR = (Ch & 0x01FF);	// 전송                       //0x01FF 쓰레기값 버린거임
}


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void ADC1_Init(void);                     //    ADC1 기능부분
void ADC2_Init(void);
void MPU6050_Init(void);                //    MPU6050 기능부분
void OLED_Init(int32_t Angle);                     //    OLED  기능부분
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
#if 0
  BLDC1 : PC6   // TIM3
  BLDC2 : PC7   // TIM3
    
  SERVO1 : PE5   //TIM9  ch1
  SERVO2 : PE6   //TIM9  ch2
    
  Right : PH10  //TIM5 ch1
  Left   : PH11  //TIM5 ch2
    
  ADC1 : PA1
  ADC2 : PA2
    
  i2C1_SCL : PB8
  i2C1_SDA: PB9
    
USART1_TX : PA9
USART1_RX : PA10

USART3_TX : PC10
USART3_RX : PC11

#endif
  
  
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_TIM9_Init();
  MX_USART3_UART_Init();
  MX_TIM4_Init();
  MX_ADC2_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */
  
  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  //osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  //defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
 

  /* Start scheduler */
  //osKernelStart();
  
  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
 SSD1306_Init();
 
 /////////////////////////////////////////////////////////////////////////////////   모터 설정
 HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);           //  BLDC 1 모터      84MHz  -> 500 Hz
 HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);           //  BLDC 2 모터      84MHz  -> 500 Hz
 
 HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_1);           //  SERVO1 모터     168MHz  -> 50 Hz
 HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_2);           //  SERVO2 모터     168MHz  -> 50 Hz
 
 
 //HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);           //  조향장치 모터     84MHz  -> 50 Hz 
 //HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);           //  조향장치 모터     84MHz  -> 50 Hz 

/////////////////////////////////////////////////////////////////////////////////
 
 ///////////////////////////////////////////////////////////////////////////////// UART1 설정
 HAL_NVIC_SetPriority(USART3_IRQn, 0, 0);               // 인터럽트 조건1
 HAL_NVIC_EnableIRQ(USART3_IRQn);                       // 인터럽트 조건2
 HAL_UART_Receive_IT(&huart3, (uint8_t *)str,1);    //인터럽트 조건3 , HAL 특성상 처음에 무조건 해줘야함
 /////////////////////////////////////////////////////////////////////////////////
 
  //HAL_NVIC_EnableIRQ(ADC_IRQn);
  //HAL_ADC_Start_IT(&hadc1);                                 
  HAL_TIM_Base_Start_IT(&htim2);  //   타이머 인터럽트를 통해 ADC1_Flag=1 로 변환
  HAL_TIM_Base_Start_IT(&htim4);  //   타이머 인터럽트를 통해 ADC1_Flag=1 로 변환
  TIM3->CCR1=1050-1;   
  TIM3->CCR2 =1050-1;
  
  Real_Ang=0;
  Servomotor_Angle1 = (int)((180-Real_Ang)*193/180+50+0.5);
  Servomotor_Angle2=(int)(Real_Ang*193/180+50+0.5);
                
  TIM9->CCR1 =Servomotor_Angle1;   
  TIM9->CCR2=Servomotor_Angle2;
  
  HAL_Delay(500);    // 처음에 servo모터 동작할 때, 적외선 작동 안되도록
  HAL_ADC_Start(&hadc2);
  HAL_ADC_Start(&hadc1);

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  
////////////////////////////////////////////////////   적외선 센서 부분

    if((ADC1_Flag==1)&&(MPU6050_Flag==0)&&(ADC2_Flag==0)){              // ADC1 핀 PA1  // 타이머2 인터럽트에 있음  // 90도가 되면 각도값 다시 추출
      ADC1_Init();                     // 벽 과의  거리 추출
     

       if(ADC_Value>800&&ADC_Value<=1900){   //1750           // 적외선 센서를 통해 거리를 감지하면 (약 1m 이하)
      //TIM3->CCR1 =1200;   
      //TIM3->CCR2 =1200;
       }
      if(ADC_Value>1900){              // 적외선 센서를 통해 거리를 감지하면 (30cm 이하)  1900
       HAL_TIM_Base_Stop_IT(&htim2);
       MPU6050_Flag=1;                // 벽타는 과정 시작하도록 플래그 켜줌
       //TIM3->CCR1 =1150;   
       TIM3->CCR2 =1200;
       HAL_Delay(1000);  // 벽에 붙는 시간  
      }// 타이머2 인터럽트를 끔으로써 ADC  종료
       

      ADC1_Flag=0;
     HAL_Delay(500);  // 벽에 붙는 시간  
    
    //  HAL_Delay(500);  // 벽에 붙는 시간  
    }

////////////////////////////////////////////////////    
////////////////////////////////////////////////////    기울기 센서 부분
    
  if(MPU6050_Flag==1&&ADC2_Flag==0){       // 벽타는 과정 시작
 
   MPU6050_Init();                // 기울어진 각도 추출

       if(Angle>-10&&Angle<10){                                 // 올라가는 속도가 너무 빨라 20도 이상만 되도 45도이상 됨
         
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////        BLDC 각도 제어   ////////////////////////////////////////////////////////////////// 
         //TIM9->CCR1=(uint16_t)((0)*193/180+50-0.5); // 각도값 넣기
       
         if(Angle<=0){
         Angle=0;
         }
        // angle180=(uint16_t)((Angle+1)*193/180+50); // 프로펠러 회전대 각도: 서보모터1 PWM 각도 조정  :차체 각도가 달라져도  벽 쪽으로 차체를 밀어 붙임
       
         if(angle180>243){
         angle180=242;// 각도 주파수가 최대치가 넘으면 최고치로 변경        //243이 180도
         }
        
         TIM9->CCR1=(uint16_t)((10+Angle)*193/180+50);      // 프로펠러 회전대 각도: 서보모터1 PWM 각도 조정  :차체 각도가 달라져도  벽 쪽으로 차체를 밀어 붙임
      
        //TIM9->CCR1=angle180;  // 각도값 넣기/
        //TIM9->CCR2=(uint16_t)((90)*193/180+50-0.5);  // 회전대2 각도 90도 유지
        TIM9->CCR2=(uint16_t)((90-Angle-16)*193/180+50-0.5);  // 회전대2 각도 90도 유지
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////        BLDC 속도 제어   ////////////////////////////////////////////////////////////////// 
        
        
        
        
        angle45_flag=1;  // 45도 보다 작으면 BLDC 속도를 올려주기 위함 , TIM4 인터럽트에 증가 있음
        TIM3->CCR1 =BLDC_Velocity1-2;  //BLDC_Velocity1=1300; 기본
        TIM3->CCR2 =(1200 -1);  //BLDC_Velocity2=1200; 기본
    
       }
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
       ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
       ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
       ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
       
       ////////////////////////////////////// 각도는 높게  힘은 세게     ////////////////////////////////////////////////////////////////
       
       ////////////////////////////////////// 각도는 높게  힘은 세게     ////////////////////////////////////////////////////////////////
       
       ////////////////////////////////////// 각도는 높게  힘은 세게     ////////////////////////////////////////////////////////////////
   
       ////////////////////////////////////// 뒷 바퀴 힘은 세게     ////////////////////////////////////////////////////////////////
     
       ////////////////////////////////////// 뒷 바퀴 힘은 세게    ////////////////////////////////////////////////////////////////
       
       ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
       ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
       ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
       
       
      else if(Angle>=10&&Angle<=25){                   /// 최소 45도
         
         
       // angle180=(uint16_t)((Angle)*193/180+50); // 프로펠러 회전대 각도: 서보모터1 PWM 각도 조정  :차체 각도가 달라져도  벽 쪽으로 차체를 밀어 붙임
       
        
        if(BLDC_Velocity1<=1850){
        angle45_flag=1;  // 45도 보다 작으면 BLDC 속도를 올려주기 위함 , TIM4 인터럽트에 증가 있음
        }
        
        /*
        if(Angle>20){
        angle45_flag=0;
        }*/
        
       TIM9->CCR1=(uint16_t)((30+Angle)*193/180+50);      // 프로펠러 회전대 각도: 서보모터1 PWM 각도 조정  :차체 각도가 달라져도  벽 쪽으로 차체를 밀어 붙임
       ///////30도 유지
       /// Servomotor_Angle1 = (int)((60-Angle)*193/180+50+0.5);
   
       
       TIM9->CCR2=(uint16_t)((90-Angle-30)*193/180+50-0.5);  // 회전대2 각도 90도 유지
       
       TIM3->CCR1 =BLDC_Velocity1-2;  //BLDC_Velocity1=1300; 기본
       
       
       
       
        if(Angle<=15){
          TIM3->CCR2 =(1200+(Angle*10));  //BLDC_Velocity2=1200; 기본  // 최대 1450
        }
        else{
        TIM3->CCR2 =(1835-1);  //BLDC_Velocity2=1200; 기본  // 최대 1450
        }
     //  if(Angle<23&&Angle>20){
      //  TIM3->CCR2 =1900;
    //    }
       }
       
       
       
       
    else if(Angle>25&& Angle<=52){             //  각도 80일때까지    //52          //최소 각도 60도 
      if(Angle<40){
                 angle45_flag=0;
      }
 
      TIM9->CCR1=(uint16_t)((50+Angle)*193/180+50);      // 프로펠러 회전대 각도: 서보모터1 PWM 각도 조정  :차체 각도가 달라져도  벽 쪽으로 차체를 밀어 붙임
      TIM9->CCR2=(uint16_t)((90-Angle-30)*193/180+50);      // 프로펠러 회전대 각도: 서보모터1 PWM 각도 조정  :차체 각도가 달라져도  벽 쪽으로 차체를 밀어 붙임

      if(Angle>45){
      TIM3->CCR1 =1850;  //BLDC_Velocity1=1300; 기본
      }
      //TIM3->CCR2 =(1200+(Angle*10));  //BLDC_Velocity2=1200; 기본  // 최대 1700
    //   if(Angle<30){
     //TIM3->CCR2 =(1200+(Angle*10));  //BLDC_Velocity2=1200; 기본  // 최대 1700
    //  }
      TIM3->CCR2 =(1835-1);  //BLDC_Velocity2= 1850-1
      
 
      
#if 0
      if(Angle<22)
      {
       TIM3->CCR1=1600;      
       TIM3->CCR2=1400;
      
      }
 /*    else if(Angle>=35){
      TIM3->CCR1=1900;      
      TIM3->CCR2 =1900;
     }*/
    
#endif
    }
    
    
        else if(Angle>52&&Angle<=85){                 // if//55
   //   MPU6050_Flag=0;// 벽타는 과정 종료 
      
 
      Real_Ang=0;// 0도 벽타는 중
  Servomotor_Angle1 = (int)((180-Real_Ang)*193/180+50+0.5);
  Servomotor_Angle2=(int)(Real_Ang*193/180+50+0.5);
      TIM9->CCR1 =Servomotor_Angle1;   
      TIM9->CCR2=Servomotor_Angle2;
      
      
     // BLDC_Velocity1=1700;
     // BLDC_Velocity2=1650;
      TIM3->CCR1 =BLDC_Velocity1;
       TIM3->CCR2 =BLDC_Velocity2;

       
      }
      
       else if(Angle>85&&Angle<=150){                 // if
         ADC1_Flag=0;
      MPU6050_Flag=0;// 벽타는 과정 종료
       Real_Ang=0;// 0도 벽타는 중
  Servomotor_Angle1 = (int)((180-Real_Ang)*193/180+50+0.5);
  Servomotor_Angle2=(int)(Real_Ang*193/180+50+0.5);
      TIM9->CCR1 =Servomotor_Angle1;   
      TIM9->CCR2=Servomotor_Angle2;
      
         BLDC_Velocity1=1700;
         BLDC_Velocity2=1650;
      TIM3->CCR1 =BLDC_Velocity1;
       TIM3->CCR2 =BLDC_Velocity2;
      
       HAL_TIM_Base_Stop_IT(&htim2);
       }
   /*    else if(Angle>90&&Angle<=180){                 
         //MPU6050_Flag=0;// 벽타는 과정 종료 
  } */
      
      
       
  }
       
      
      if(ADC2_Flag==1){                            // 후진 시
      //  ADC2_Flag=0;
        HAL_TIM_Base_Start_IT(&htim2);// ADC 시작 인터럽트
       ADC2_Init();
       
      
      if(ADC_Value>2800){              // 적외선 센서를 통해 거리를 감지하면 (30cm 이하)
        ADC2_Flag=0;
        HAL_Delay(3000);
        HAL_TIM_Base_Stop_IT(&htim2);// ADC 끝 인터럽트
        MPU6050_Flag2=1;
      }  
      }
      if(MPU6050_Flag2==1){
         MPU6050_Init();                //각도 추출
         
        if(Angle>=20){//&&Angle<65){
       
      Servomotor_Angle1 = (int)((90+Angle)*193/180+50);    // 원하는 각도로 선택      벽에 90도로 유지
      Servomotor_Angle2=(int)((180)*193/180+50);  // 실제 각도로 선택            바닥에 180도 유지
     // Servomotor_Angle2=(int)((180-Angle)*193/180+50+0.5);  // 실제 각도로 선택            바닥에 180도 유지
      TIM9->CCR1 =Servomotor_Angle1;   
      TIM9->CCR2=Servomotor_Angle2;
        
      
              BLDC_Velocity1=1250;
              BLDC_Velocity2=1250;
              
       TIM3->CCR1 =BLDC_Velocity1;   // BLDC속도1
       TIM3->CCR2 =BLDC_Velocity2;   // BLDC속도2
      // 타이머2 인터럽트를 끔으로써 ADC  종료
        }
      else if(Angle<20&&Angle>10){
      Real_Ang=0;// 0도 벽타는 중
        Servomotor_Angle1 = (int)((90+Angle)*193/180+50);  // 아래 방향으로
        Servomotor_Angle2=(int)(Real_Ang*193/180+50);       // 아래 방향으로
      TIM9->CCR1 =Servomotor_Angle1;   
      TIM9->CCR2=Servomotor_Angle2;
      
      
              BLDC_Velocity1=1150;
              BLDC_Velocity2=1150;
       TIM3->CCR1 =BLDC_Velocity1;   // BLDC속도1
       TIM3->CCR2 =BLDC_Velocity2;   // BLDC속도2
      }
      else if(Angle<=10){
      Real_Ang=0;// 0도 벽타는 중
        Servomotor_Angle1 = (int)((180-Real_Ang)*193/180+50);
        Servomotor_Angle2=(int)(Real_Ang*193/180+50);
      TIM9->CCR1 =Servomotor_Angle1;   
      TIM9->CCR2=Servomotor_Angle2;
      
                  BLDC_Velocity =1100-1;
                  MPU6050_Flag2=0;// 각도 플래그 끝
                  HAL_Delay(1000);
                  
               HAL_TIM_PWM_Stop(&htim3,TIM_CHANNEL_1);
               HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);
               
               
               HAL_TIM_PWM_Stop(&htim9, TIM_CHANNEL_1);           //  SERVO1 모터     168MHz  -> 50 Hz
               HAL_TIM_PWM_Stop(&htim9, TIM_CHANNEL_2);           //  SERVO2 모터     168MHz  -> 50 Hz
      }
      
      }
      
////////////////////////////////////////////////////    
 //Servomotor_Angle1 = (int)((180-Real_Ang)*193/180+50+0.5);  // 각도 반전
 //Servomotor_Angle2=(int)(Real_Ang*193/180+50+0.5);

    SSD1306_Init();  //OLED 초기화
      
   //HAL_Delay(100);
       
 
  }
  
  
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /**Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */
  /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }
  /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 8400-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 10000-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 84-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 2000-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 840-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 100-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 840-1;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 2000-1;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */
  HAL_TIM_MspPostInit(&htim5);

}

/**
  * @brief TIM9 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM9_Init(void)
{

  /* USER CODE BEGIN TIM9_Init 0 */

  /* USER CODE END TIM9_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM9_Init 1 */

  /* USER CODE END TIM9_Init 1 */
  htim9.Instance = TIM9;
  htim9.Init.Prescaler = 1680-1;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = 2000-1;
  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_PWM_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim9, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim9, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM9_Init 2 */

  /* USER CODE END TIM9_Init 2 */
  HAL_TIM_MspPostInit(&htim9);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_4|GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA0 PA4 PA3 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_4|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void ADC1_Init(void){

   HAL_ADC_Start(&hadc1);
   HAL_ADC_Stop(&hadc2);
   if(HAL_ADC_PollForConversion(&hadc1, 1000000)==HAL_OK){
      ADC_Value=HAL_ADC_GetValue(&hadc1);
    
    Voltage =(uint16_t)(ADC_Value * (3.3 * 100) / 4095);   // 5: 4095 =  Volatge : ADC_Value  
    //distance_cm = 40083*pow(ADC_Value,-0.849);
    distance_cm=(uint16_t)(-0.018 *ADC_Value+72.752); // 입력전압 5v일 때  //어짜피 양수
    //////////////////////////////////////////////////////////////////// USART1 출력부분
    //printf("ADC : %4d \r\n",ADC_Value);
    //printf("Voltage : %4d \r\n",Voltage);
    //printf("Distance : %4d \r\n",distance_cm);
    
    //////////////////////////////////////////////////////////////////// OLED 출력 부분
    SSD1306_DrawChar(41,16,distance_cm/100+0x30);
    SSD1306_DrawChar(47,15,distance_cm%100/10+0x30);
    SSD1306_DrawChar(53,15,distance_cm%10/1+0x30);
   }

}
void ADC2_Init(void){

   HAL_ADC_Start(&hadc2);
   HAL_ADC_Stop(&hadc1);
   if(HAL_ADC_PollForConversion(&hadc2, 1000000)==HAL_OK){
      ADC_Value=HAL_ADC_GetValue(&hadc2);
    
    Voltage =(uint16_t)(ADC_Value * (3.3 * 100) / 4095);   // 5: 4095 =  Volatge : ADC_Value  
    //distance_cm = 40083*pow(ADC_Value,-0.849);
    distance_cm=(uint16_t)(-0.018 *ADC_Value+72.752); // 입력전압 5v일 때  //어짜피 양수
    //////////////////////////////////////////////////////////////////// USART1 출력부분
    //printf("ADC : %4d \r\n",ADC_Value);
    //printf("Voltage : %4d \r\n",Voltage);
    //printf("Distance : %4d \r\n",distance_cm);
    
    //////////////////////////////////////////////////////////////////// OLED 출력 부분
    SSD1306_DrawChar(41,16,distance_cm/100+0x30);
    SSD1306_DrawChar(47,15,distance_cm%100/10+0x30);
    SSD1306_DrawChar(53,15,distance_cm%10/1+0x30);
   }

}
void MPU6050_Init(void){

  
       getAngle();	  // 각도 추출
      
       if(angleX>=0){
       angleX=angleX-3;
       }
       else{
       angleX =angleX+3;
       }
        Angle=(int32_t)(angleX);     // 정수형 변환
       //Angle1=(int32_t)(angleY+0.5);   // 정수형 변환
     ////printf("X : %5.2f, Y : %5.2f \r\n", angleX, angleY);   // USART1_Tx를 통해 보냄
    //printf("실제 각도           Angle              %3d \r\n", Angle);   // USART1_Tx를 통해 보냄

//SetAngle=Angle+4;
     OLED_Init(Angle);// OLED 각도 표현
     // servo=Angle+100+70;              // 서보모터 각도 추출
      
      

}
int asd;
void USART3_IRQHandler(void)           // 받으면 인터럽트 됨
{    
  if ( (USART3->SR & USART_SR_RXNE) )            // 데이터가 오면   // flag 지울 필요없이  데이터를 읽으면 flag값이 지워짐
	{
		char ch;
		ch = (uint16_t)(USART3->DR & (uint16_t)0x01FF);	// 수신된 문자 저장
		//SerialPutChar(ch); 
                
                if(ch==0x30){
               //printf("   0 \n");
                  BLDC_Velocity =1050;
               HAL_TIM_PWM_Stop(&htim3,TIM_CHANNEL_1);
               HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);
               
               
               HAL_TIM_PWM_Stop(&htim9, TIM_CHANNEL_1);           //  SERVO1 모터     168MHz  -> 50 Hz
               HAL_TIM_PWM_Stop(&htim9, TIM_CHANNEL_2);           //  SERVO2 모터     168MHz  -> 50 Hz
                  
                }

               else   if(ch==0x31){
               //printf("   2000 \n");
                    Real_Ang=45;
                  Servomotor_Angle1 = (int)((180-Real_Ang)*193/180+50+0.5);
                Servomotor_Angle2=(int)(Real_Ang*193/180+50+0.5);
                
                TIM9->CCR1 =Servomotor_Angle1;   
                TIM9->CCR2=Servomotor_Angle2;
                
           //   BLDC_Velocity1=1600;
          //BLDC_Velocity2=1400;
               TIM3->CCR1 =1675;
               TIM3->CCR2 =1675;
                
               }
                
                else if(ch==0x32){           // 꺼짐
                //printf("   1100 \n");
                  BLDC_Velocity=1175;
                   HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);           //  BLDC 1 모터      84MHz  -> 500 Hz
                   HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);           //  BLDC 2 모터      84MHz  -> 500 Hz
                
               
                  HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_1);           //  SERVO1 모터     168MHz  -> 50 Hz
                  HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_2);           //  SERVO2 모터     168MHz  -> 50 Hz
                
                }
                else if(ch==0x33){
                //printf("   1200 \n");
                  BLDC_Velocity=1180;
                   HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);           //  BLDC 1 모터      84MHz  -> 500 Hz
                   HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);           //  BLDC 2 모터      84MHz  -> 500 Hz
                
               
                  HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_1);           //  SERVO1 모터     168MHz  -> 50 Hz
                  HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_2);           //  SERVO2 모터     168MHz  -> 50 Hz
                }
                else if(ch==0x34){
                //printf("   1300 \n");
                  
                  
                   BLDC_Velocity=1200;
                   HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);           //  BLDC 1 모터      84MHz  -> 500 Hz
                   HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);           //  BLDC 2 모터      84MHz  -> 500 Hz
                
               
                  HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_1);           //  SERVO1 모터     168MHz  -> 50 Hz
                  HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_2);           //  SERVO2 모터     168MHz  -> 50 Hz
                  
                }
                else if(ch==0x35){             // 벽에서 버티는 것            // 벽에서 버티는 것            // 벽에서 버티는 것
                //printf("   1400 \n");
                   Real_Ang=30;
                  Servomotor_Angle1 = (int)((180-Real_Ang)*193/180+50+0.5);
                Servomotor_Angle2=(int)(Real_Ang*193/180+50+0.5);
                
                TIM9->CCR1 =Servomotor_Angle1;   
                TIM9->CCR2=Servomotor_Angle2;
                
           //   BLDC_Velocity1=1600;
          //BLDC_Velocity2=1400;
               //
               
                if( bldc_V_flag==0){
                  bldc_V=1800;
                 bldc_V_flag=1;}
                
               TIM3->CCR1 =bldc_V;
               TIM3->CCR2 =bldc_V;                

                }
                 else if(ch==0x36){            // 벽에서 버티는 것            // 벽에서 버티는 것            // 벽에서 버티는 것
                //printf("   1500 \n");
                   Real_Ang=30;
                  Servomotor_Angle1 = (int)((180-Real_Ang)*193/180+50+0.5);
                Servomotor_Angle2=(int)(Real_Ang*193/180+50+0.5);
                
                TIM9->CCR1 =Servomotor_Angle1;   
                TIM9->CCR2=Servomotor_Angle2;
                
           //   BLDC_Velocity1=1600;
          //BLDC_Velocity2=1400;
              
                if( bldc_V_flag==0){
                  bldc_V=1820;
                 bldc_V_flag=1;}
               TIM3->CCR1 =bldc_V;
               TIM3->CCR2 =bldc_V;
                
                  
                }
                 else if(ch==0x37){ /////// 속도 업

                bldc_V=bldc_V+10;
                
               TIM3->CCR1 =bldc_V;
               TIM3->CCR2 =bldc_V;
                
                  
                }
                else if(ch==0x38){   //////////속도 다운
                //printf("   1800 \n");


                bldc_V=bldc_V-10;
                
               TIM3->CCR1 =bldc_V;
               TIM3->CCR2 =bldc_V;
                
                }
                else if(ch==0x39){
                //printf("   1900 \n");
                    Real_Ang=45;
                  Servomotor_Angle1 = (int)((180-Real_Ang)*193/180+50+0.5);
                Servomotor_Angle2=(int)(Real_Ang*193/180+50+0.5);
                
                TIM9->CCR1 =Servomotor_Angle1;   
                TIM9->CCR2=Servomotor_Angle2;
                
           //   BLDC_Velocity1=1600;
          //BLDC_Velocity2=1400;
               TIM3->CCR1 =1675;
               TIM3->CCR2 =1675;
                
                }
                
                
                
                 else if(ch=='n'){
                //printf("전진 \n");
                ADC1_Flag=1;
                ADC2_Flag=0;
                ADC_Change=0; //전진
                 HAL_TIM_Base_Start_IT(&htim2);
                }
                 else if(ch=='m'){
                //printf("후진 \n");
                HAL_TIM_Base_Start_IT(&htim2);
                ADC1_Flag=0;
                ADC2_Flag=1;
                ADC_Change=1; //후진
                 HAL_TIM_Base_Start_IT(&htim2);
                }
               else if(ch=='a'){
                //printf(" 서보모터  0 \n");
                  Real_Ang=0;
                  Servomotor_Angle1 = (int)((180-Real_Ang)*193/180+50+0.5);
                Servomotor_Angle2=(int)(Real_Ang*193/180+50+0.5);
                
                TIM9->CCR1 =Servomotor_Angle1;   
                TIM9->CCR2=Servomotor_Angle2;
                }
                else if(ch=='s'){
                //printf("   서보모터 30 \n");
                  Real_Ang=30;
                  Servomotor_Angle1 = (int)((180-Real_Ang)*193/180+50+0.5);
                Servomotor_Angle2=(int)(Real_Ang*193/180+50+0.5);
                
                TIM9->CCR1 =Servomotor_Angle1;   
                TIM9->CCR2=Servomotor_Angle2;
                }
                else if(ch=='d'){
                //printf("   서보모터 45 \n");
                  Real_Ang=45;
                  Servomotor_Angle1 = (int)((180-Real_Ang)*193/180+50+0.5);
                Servomotor_Angle2=(int)(Real_Ang*193/180+50+0.5);
                
                TIM9->CCR1 =Servomotor_Angle1;   
                TIM9->CCR2=Servomotor_Angle2;
                }
                else if(ch=='f'){
                //printf("    서보모터 60 \n");
                  Real_Ang=60;
                  Servomotor_Angle1 = (int)((180-Real_Ang)*193/180+50+0.5);
                Servomotor_Angle2=(int)(Real_Ang*193/180+50+0.5);
                
                TIM9->CCR1 =Servomotor_Angle1;   
                TIM9->CCR2=Servomotor_Angle2;
                }
                else if(ch=='g'){
                //printf("   서보모터 90 \n");
                  Real_Ang=90;
                  Servomotor_Angle1 = (int)((180-Real_Ang)*193/180+50+0.5);
                Servomotor_Angle2=(int)(Real_Ang*193/180+50+0.5);
                
                TIM9->CCR1 =Servomotor_Angle1;   
                TIM9->CCR2=Servomotor_Angle2;
                
                }
                 else if(ch=='z'){
                //printf("   후진 1 \n");
                  Real_Ang=45;
                  Servomotor_Angle1 = (int)((180-Real_Ang)*193/180+50+0.5);
                Servomotor_Angle2=(int)(Real_Ang*193/180+50+0.5);
                
                TIM9->CCR1 =Servomotor_Angle1;   
                TIM9->CCR2=Servomotor_Angle2;
                
           //   BLDC_Velocity1=1600;
          //BLDC_Velocity2=1400;
                
                HAL_TIM_Base_Start_IT(&htim2);
                ADC1_Flag=0;
                ADC2_Flag=1;
                ADC_Change=1; //후진
                 HAL_TIM_Base_Start_IT(&htim2);
                 bldc_V=1430;
               TIM3->CCR1 =1430;
               TIM3->CCR2 =1430;
                }
                
                 else if(ch=='x'){
                 //printf("   후진 2 \n");
                  Real_Ang=45;
                  Servomotor_Angle1 = (int)((180-Real_Ang)*193/180+50+0.5);
                Servomotor_Angle2=(int)(Real_Ang*193/180+50+0.5);
                
                TIM9->CCR1 =Servomotor_Angle1;   
                TIM9->CCR2=Servomotor_Angle2;
                
                HAL_TIM_Base_Start_IT(&htim2);
                ADC1_Flag=0;
                ADC2_Flag=1;
                ADC_Change=1; //후진
                 HAL_TIM_Base_Start_IT(&htim2);
           //   BLDC_Velocity1=1600;
          //BLDC_Velocity2=1400;
                 bldc_V=1330;
               TIM3->CCR1 =1330;
               TIM3->CCR2 =1330;
                
                }
                 else if(ch=='c'){
                 //printf("   후진 3 \n");
                  Real_Ang=45;
                  Servomotor_Angle1 = (int)((180-Real_Ang)*193/180+50+0.5);
                Servomotor_Angle2=(int)(Real_Ang*193/180+50+0.5);
                
                TIM9->CCR1 =Servomotor_Angle1;   
                TIM9->CCR2=Servomotor_Angle2;
                
                HAL_TIM_Base_Start_IT(&htim2);
                ADC1_Flag=0;
                ADC2_Flag=1;
                ADC_Change=1; //후진
                 HAL_TIM_Base_Start_IT(&htim2);
                 bldc_V=1250;
               TIM3->CCR1 =1250;
               TIM3->CCR2 =1250;
                }
                 else if(ch=='v'){
                //printf("   서보모터 180 \n");
                  Real_Ang=180;
                  Servomotor_Angle1 = (int)((180-Real_Ang)*193/180+50+0.5);
                Servomotor_Angle2=(int)(Real_Ang*193/180+50+0.5);
                
                TIM9->CCR1 =Servomotor_Angle1;   
                TIM9->CCR2=Servomotor_Angle2;
                
                }
                  else if(ch=='b'){
                //printf(" 기본 주행 \n");
                 
                  Servomotor_Angle1 = (int)((180-180)*193/180+50+0.5);
                Servomotor_Angle2=(int)(90*193/180+50+0.5);
                
                TIM9->CCR1 =Servomotor_Angle1;   
                TIM9->CCR2=Servomotor_Angle2;
                
                TIM3->CCR2 =1300;
                
                }
                else if(ch=='q'){
                //printf("주행 1단계 \n");
                  Real_Ang=45;     
                  
                }
                else if(ch=='w'){
                //printf("주행 2단계 \n");
                  Real_Ang=60;   
                }
                else if(ch=='e'){
                //printf("주행 3단계 \n");
                  Real_Ang=70;
                }
                  else if(ch=='i'){
                //printf("벽 1단계 \n");
                  Real_Ang=60;
                BLDC_Velocity =1720;
                bldc_V=1720;
                ADC1_Flag=0;
                ADC2_Flag=0;
                ADC_Change=0; //전진
                }
                else if(ch=='o'){
                //printf("벽 2단계 \n");
                  Real_Ang=60;
                BLDC_Velocity =1745;/////////////////////////////////////////////이게 되는 거
                bldc_V=1745;
                ADC2_Flag=0;
                ADC_Change=0; //전진
                }
                else if(ch=='p'){
                //printf("벽 3단계 \n");
                 Real_Ang=60;
                BLDC_Velocity =1765;

                
                ADC1_Flag=0;
                ADC2_Flag=0;
                ADC_Change=0; //전진
                }
              else if(ch=='k'){
                //printf("ADC 끔 \n");
              //  ADC_MPU_Flag=0;
                 HAL_TIM_Base_Stop_IT(&htim2);
                 HAL_ADC_Stop(&hadc1);
                 HAL_ADC_Stop(&hadc2);
                 ADC1_Flag=0;
                 ADC2_Flag=0;
                }
               else if(ch=='l'){
                //printf("ADC 킴 \n");
              //  ADC_MPU_Flag=1;
                 HAL_TIM_Base_Start_IT(&htim2);
                 HAL_ADC_Start(&hadc1);
                 HAL_ADC_Start(&hadc2);
                }
                
                
                
                
                
#if 0      
                else if(ch=='u'){       ////////////////////////////  좌향
                  //int direction=90;
                  //int center=2;
                  //int Right_and_left=0;
                HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);           //  조향장치 모터     84MHz  -> 50 Hz 키고
                HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);           //  조향장치 모터     84MHz  -> 50 Hz 키고
                
                
                Right_angle=150;
                Left_angle=150;
               TIM5->CCR1 =(int)(Right_angle*193/180+50+0.5);
               TIM5->CCR2 =(int)(Left_angle*193/180+50+0.5);
                center=1;
                
                
  
                for (direction=90; direction>=55; direction--){
                    
                  Right_and_left=(int)(direction*193/180+50+0.5);
                  TIM5->CCR1 =Right_and_left;
               //   HAL_Delay(30);
                   }
                   center=1;
       
        
                }
                
                
                
                
                else if(ch=='y'){       ////////////////////////////  중앙
                  //int direction=90;
                  //int center=2;
                  //int Right_and_left=0;
               HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);           //  조향장치 모터     84MHz  -> 50 Hz 키고
                HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);           //  조향장치 모터     84MHz  -> 50 Hz 키고
                 
               if(center==1){
                Right_angle=90;
                Left_angle=90;
               TIM5->CCR1 =(int)(Right_angle*193/180+50+0.5);
               TIM5->CCR2 =(int)(Left_angle*193/180+50+0.5);
               }
                if(center==3){
                  
                Right_angle=90;
                Left_angle=90;
               TIM5->CCR1 =(int)(Right_angle*193/180+50+0.5);
               TIM5->CCR2 =(int)(Left_angle*193/180+50+0.5);
                }
                
               center=2;
               
               
               
                       if(center==1){
                   for (direction=55; direction<=105; direction++){
                     
                    Right_and_left=(int)(direction*193/180+50+0.5);
                  TIM5->CCR1 =Right_and_left;
                  
                   center=2;
                // HAL_Delay(30);
                   }
                 }
                 if(center==3){
                   for (direction=130; direction>=85; direction--){
                     
                     Right_and_left=(int)(direction*193/180+50+0.5);
                  TIM5->CCR1 =Right_and_left;
                  
                   center=2;
              
                   }
                 }

                   
                 
                }
                
                
                
                
                else if(ch=='t'){       ////////////////////////////  우향
                  //int direction=90;
                  //int center=2;
                  //int Right_and_left=0;
               HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);           //  조향장치 모터     84MHz  -> 50 Hz 키고
               HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);           //  조향장치 모터     84MHz  -> 50 Hz 키고

                Right_angle=30;
                Left_angle=30;
               TIM5->CCR1 =(int)(Right_angle*193/180+50+0.5);
               TIM5->CCR2 =(int)(Left_angle*193/180+50+0.5);
               center=3;
               
               
               

                for ( direction=90; direction<=130; direction++){
                  Right_and_left=(int)(direction*193/180+50+0.5);
                  TIM5->CCR1 =Right_and_left;
                  
              //  HAL_Delay(30);
                  center=3;
                }

                
                }
             
#endif  
                
                
                
                
                TIM3->CCR1 =BLDC_Velocity-1;   
                TIM3->CCR2 =BLDC_Velocity-1;
                
                Servomotor_Angle1 = (int)((180-Real_Ang)*193/180+50+0.5);
                Servomotor_Angle2=(int)(Real_Ang*193/180+50+0.5);
                
                TIM9->CCR1 =Servomotor_Angle1;   
                TIM9->CCR2=Servomotor_Angle2;
        }
        // DR 을 읽으면 SR.RXNE bit(flag bit)는 clear 된다 
}
void OLED_Init(int32_t Angle){
  if(Angle<0){      // -값
         Angle= -(Angle+0x01);

       SSD1306_DrawChar(35,3,0x2D);   //"  - 부호"
       SSD1306_DrawChar(41,3,Angle%1000/100+0x30);
       SSD1306_DrawChar(47,3,Angle%100/10+0x30);
       SSD1306_DrawChar(53,3,Angle%10/1+0x30);

       }
       else{

       SSD1306_DrawChar(35,3,0x00);   
       SSD1306_DrawChar(41,3,Angle%1000/100+0x30);
       SSD1306_DrawChar(47,3,Angle%100/10+0x30);
       SSD1306_DrawChar(53,3,Angle%10/1+0x30);
       }

}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
#if 0
void StartDefaultTask(void const * argument)
{

  /* USER CODE BEGIN 5 */
  int cnt = 0;
  char tmp[25];
  //memset(tmp,0,25);
  /* Infinite loop */
  //WS0010_Put_Str(0,1,"CYSCO ZZANG");
  //while(1)
 // {
   // SSD1306_Init();
    sprintf(tmp,"%20d",cnt++);
    WS0010_Put_Str(0,0,(uint8_t*)tmp);

    printf("이상한 반복문 안으로 들어왔음 \r\n");
    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_0,GPIO_PIN_RESET);
    osDelay(1000);
    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_0,GPIO_PIN_SET);
    osDelay(1000);

    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,GPIO_PIN_RESET);
    osDelay(250);
    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,GPIO_PIN_SET);
    osDelay(250);

    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_RESET);
    osDelay(250);
    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_SET);
    osDelay(250);    
    
    
    
    
    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_0,GPIO_PIN_RESET);
    osDelay(250);
    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,GPIO_PIN_RESET);
    osDelay(250);
    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_RESET);
    osDelay(250);    
  //}
  /* USER CODE END 5 */ 
}
#endif

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */
   if (htim->Instance == TIM2) {  
     if(ADC_Change==0){
       ADC1_Flag=1;}
     
     
     else{
       ADC2_Flag=1;}
     
  // printf("타이머2 끝남 \r\n");
  // SSD1306_Init();/////////////////////////  OLED 화면
   }
      else if(htim->Instance == TIM4){		// timer5
		milliseconds++;
                  BLDC_TIME++;
                  BLDC_TIME1++;
                 // BLDC_TIME2++;
                 // BLDC_TIME3++;
                //  BLDC_TIME4++;
                  if(BLDC_TIME==10){
                  BLDC_TIME=0;
                  
                if(angle45_flag==1 && BLDC_Velocity1<1800){
          BLDC_Velocity1 += 2;  //  BLDC_Velocity1초기값 1100-2  // 1초에 1000번 이므로 1초에 2000씩 증가  그리고 한번 인터럽트 일어날 때 마다 2씩증가
          TIM3->CCR2 =1250;  //BLDC_Velocity2=1200; 기본
	}
       }
       
       
       if(BLDC_TIME1==500){  //1.5초
        BLDC_TIME1=0;
                  
        if(angle45_flag==1 && BLDC_Velocity1<1950&& BLDC_Velocity1>=1800){
          BLDC_Velocity1 += 10;  //  BLDC_Velocity1초기값 1100-2  // 1초에 1000번 이므로 1초에 2000씩 증가  그리고 한번 인터럽트 일어날 때 마다 2씩증가
          //TIM3->CCR2 +=10;
	}

       } 
       
       
      }
  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
