#define __WS0010_C__
    #include "WS0010.h"
#undef  __WS0010_C__

/********Porting Start**********/
#define SCL_GPIO_PORT                GPIOF
#define SCL_GPIO_PIN                 GPIO_PIN_1
#define SCL_GPIO_CLK_ENABLE()        __HAL_RCC_GPIOF_CLK_ENABLE()

#define SDO_GPIO_PORT                GPIOF
#define SDO_GPIO_PIN                 GPIO_PIN_3
#define SDO_GPIO_CLK_ENABLE()        __HAL_RCC_GPIOF_CLK_ENABLE()

#define SDI_GPIO_PORT                GPIOF
#define SDI_GPIO_PIN                 GPIO_PIN_5
#define SDI_GPIO_CLK_ENABLE()        __HAL_RCC_GPIOF_CLK_ENABLE()

#define CS_GPIO_PORT                 GPIOF
#define CS_GPIO_PIN                  GPIO_PIN_7
#define CS_GPIO_CLK_ENABLE()         __HAL_RCC_GPIOF_CLK_ENABLE()
/********Porting End**********/

#define SCL_H                        HAL_GPIO_WritePin(SCL_GPIO_PORT,SCL_GPIO_PIN,GPIO_PIN_SET)
#define SCL_L                        HAL_GPIO_WritePin(SCL_GPIO_PORT,SCL_GPIO_PIN,GPIO_PIN_RESET)

#define SDO_H                        HAL_GPIO_WritePin(SDO_GPIO_PORT,SDO_GPIO_PIN,GPIO_PIN_SET)
#define SDO_L                        HAL_GPIO_WritePin(SDO_GPIO_PORT,SDO_GPIO_PIN,GPIO_PIN_RESET)

#define SDI_H                        HAL_GPIO_WritePin(SDI_GPIO_PORT,SDI_GPIO_PIN,GPIO_PIN_SET)
#define SDI_L                        HAL_GPIO_WritePin(SDI_GPIO_PORT,SDI_GPIO_PIN,GPIO_PIN_RESET)

#define CS_H                         HAL_GPIO_WritePin(CS_GPIO_PORT ,CS_GPIO_PIN ,GPIO_PIN_SET)
#define CS_L                         HAL_GPIO_WritePin(CS_GPIO_PORT ,CS_GPIO_PIN ,GPIO_PIN_RESET)
  
static void Gpio_Init(void);
static void UTIL_DelayMS(uint16_t wMS);
static void UTIL_DelayUS(uint16_t wUS);

static void position_cursor(uint8_t pos, uint8_t line);
static void Cmd_Write(uint8_t cmd);
static void Data_Write(uint8_t data);

void WS0010_Init(void)
{
    Gpio_Init();
    
    //OLED INIT
    UTIL_DelayMS(500);
    
    Cmd_Write(0x38); //Function Set 
    Cmd_Write(0x17); //Display On/Off Control : Entire Display on, Cursor off, Cursor Blinking Off 
    WS0010_Clear();
    Cmd_Write(0x0C);
    Cmd_Write(0x02); //Return Home
    Cmd_Write(0x06); //Entry Mode : increment, No Shift 
    WS0010_Clear();
}

static void Gpio_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    SCL_GPIO_CLK_ENABLE();
    SDO_GPIO_CLK_ENABLE();
    SDI_GPIO_CLK_ENABLE();
    CS_GPIO_CLK_ENABLE();
    
    SCL_H;
    SDO_H;
    SDI_H;
    CS_H;
    
    GPIO_InitStruct.Pin = SCL_GPIO_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(SCL_GPIO_PORT, &GPIO_InitStruct);
    
    GPIO_InitStruct.Pin = SDO_GPIO_PIN;
    HAL_GPIO_Init(SDO_GPIO_PORT, &GPIO_InitStruct);
    
    GPIO_InitStruct.Pin = SDI_GPIO_PIN;
    HAL_GPIO_Init(SDI_GPIO_PORT, &GPIO_InitStruct);
    
    GPIO_InitStruct.Pin = CS_GPIO_PIN;
    HAL_GPIO_Init(CS_GPIO_PORT, &GPIO_InitStruct);
}

static void UTIL_DelayMS(uint16_t wMS)
{
    register uint16_t i;

    for (i=0; i<wMS; i++)
        UTIL_DelayUS(1000);         // 1000us => 1ms
}

static void UTIL_DelayUS(uint16_t wUS)
{
    volatile uint32_t Dly = (uint32_t)wUS*17;
    for(; Dly; Dly--);
}

static void Cmd_Write(uint8_t cmd)
{
    CS_L;
    UTIL_DelayUS(1);
    SCL_L;
    UTIL_DelayUS(1);
    
    SDI_L; //RS low = cmd
    UTIL_DelayUS(1);
    SCL_H; 
    
    UTIL_DelayUS(1);
    SCL_L;
    
    SDI_L; //RW low = write
    UTIL_DelayUS(1);
    SCL_H; 

    for(int i=7; i>=0; i--)
    {
        UTIL_DelayUS(1);
        SCL_L;
        UTIL_DelayUS(1);
    
        if(cmd&(1<<i))
            SDI_H;
        else
            SDI_L;
        
        UTIL_DelayUS(1);
        SCL_H;
    }
    
    UTIL_DelayUS(1);
    CS_H;
    UTIL_DelayMS(1);
}

static void Data_Write(uint8_t data)
{
    CS_L;
    UTIL_DelayUS(1);
    SCL_L;
    UTIL_DelayUS(1);
    
    SDI_H; //RS high = data
    UTIL_DelayUS(1);
    SCL_H; 
    
    UTIL_DelayUS(1);
    SCL_L;
    
    SDI_L; //RW low = write
    UTIL_DelayUS(1);
    SCL_H; 

    for(int i=7; i>=0; i--)
    {
        UTIL_DelayUS(1);
        SCL_L;
        UTIL_DelayUS(1);
    
        if(data&(1<<i))
            SDI_H;
        else
            SDI_L;
        
        UTIL_DelayUS(1);
        SCL_H;
    }
    
    UTIL_DelayUS(1);
    CS_H;
    UTIL_DelayMS(1);
}

static void position_cursor(uint8_t pos, uint8_t line)
{
    Cmd_Write(0x02);
    Cmd_Write(0x80 | (line?0x40:0x00) | (pos & 0x3F));
}

void WS0010_Clear(void)
{
    Cmd_Write(0x01); 
    UTIL_DelayMS(6);
}

void WS0010_Put_Str(uint8_t pos,uint8_t line,uint8_t* str)
{
    position_cursor(pos,line);
    while( (*str) != 0 )
      Data_Write(*(str++));
}
