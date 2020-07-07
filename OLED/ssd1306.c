#define __SSD1306_C__
    #include "ssd1306.h"
#undef  __SSD1306_C__
#include "stm32f4xx_hal_spi.h"
#include "main.h"
#include "app.h"

#define MAX_COL        64
#define MAX_PAGE        6

#define CS_H            HAL_GPIO_WritePin(GPIOI,GPIO_PIN_0,GPIO_PIN_SET)
#define CS_L            HAL_GPIO_WritePin(GPIOI,GPIO_PIN_0,GPIO_PIN_RESET)

#define DC_H            HAL_GPIO_WritePin(GPIOI,GPIO_PIN_2,GPIO_PIN_SET)
#define DC_L            HAL_GPIO_WritePin(GPIOI,GPIO_PIN_2,GPIO_PIN_RESET)

#define RES_H           HAL_GPIO_WritePin(GPIOI,GPIO_PIN_4,GPIO_PIN_SET)
#define RES_L           HAL_GPIO_WritePin(GPIOI,GPIO_PIN_4,GPIO_PIN_RESET)

static void SSD1306_WriteData(uint8_t data);
static void SSD1306_WriteCmd(uint8_t cmd);
static void Spi_Init(void);
static void Gpio_Init(void);
static void UTIL_DelayMS(uint16_t wMS);
static void UTIL_DelayUS(uint16_t wUS);
static void SPI_SendByte(uint8_t data);
static void Set_Page_Address(uint8_t add);
static void Set_Column_Address(uint8_t add);
static void Display_Refresh(void);
static void SetPoint(uint8_t x,uint8_t y);
static void ClearPoint(uint8_t x,uint8_t y);

static SPI_HandleTypeDef SpiHandle;
static uint8_t FrameBuffer[MAX_PAGE][MAX_COL];
static sFONT *SSD1306_pFont;
 int i=3;
extern int16_t g_x, g_y, g_z, a_x, a_y, a_z;
extern int16_t Accelerometer_X; /*!< Accelerometer value X axis */
extern int16_t Accelerometer_Y; /*!< Accelerometer value Y axis */
extern int16_t Accelerometer_Z; /*!< Accelerometer value Z axis */
extern int16_t Gyroscope_X;     /*!< Gyroscope value X axis */
extern int16_t Gyroscope_Y;     /*!< Gyroscope value Y axis */
extern int16_t Gyroscope_Z; 
extern void SerialPutChar(uint8_t Ch); // 1문자 보내기 함수
extern signed short ADC_Value,ADC_Value1;   // 값
void SSD1306_Init(void)
{
    Gpio_Init();
    Spi_Init();
    
    CS_H;
    RES_H;
    UTIL_DelayMS(1);
    RES_L;
    UTIL_DelayMS(1);
    RES_H;
    
    SSD1306_WriteCmd(0xAE);
    
    SSD1306_WriteCmd(0xD5);
    SSD1306_WriteCmd(0x80);
    
    SSD1306_WriteCmd(0xA8);
    SSD1306_WriteCmd(0x2F);
    
    SSD1306_WriteCmd(0xD3);
    SSD1306_WriteCmd(0x00);
    
    SSD1306_WriteCmd(0x40);
    
    SSD1306_WriteCmd(0x8D);
    SSD1306_WriteCmd(0x14);
    
    SSD1306_WriteCmd(0xA1);
    
    SSD1306_WriteCmd(0xC8);
    
    SSD1306_WriteCmd(0xDA);
    SSD1306_WriteCmd(0x12);
    
    SSD1306_WriteCmd(0x81);
    SSD1306_WriteCmd(0xFF);
    
    SSD1306_WriteCmd(0xD9);
    SSD1306_WriteCmd(0x22);
    
    SSD1306_WriteCmd(0xD8);
    SSD1306_WriteCmd(0x00);
    
    SSD1306_WriteCmd(0xA4);
    
    SSD1306_WriteCmd(0xA6);

    SSD1306_WriteCmd(0x20); //Set Memory Addressing Mode
    SSD1306_WriteCmd(0x00); //Horizontal Addressing Mode
    
    SSD1306_WriteCmd(0x21); //Set Column Address
    SSD1306_WriteCmd(32);   //Column Start : 32
    SSD1306_WriteCmd(95);   //Column End   : 95
    
    SSD1306_WriteCmd(0x22); //Set Page Address
    SSD1306_WriteCmd(2);    //Page Start : 2
    SSD1306_WriteCmd(7);    //Page End   : 7    
    
    SSD1306_WriteCmd(0xAF);    
    
    //SSD1306_Clear();

   
    //uint8_t height = 2;
   // SSD1306_SetFont(&Gulim7);
  //  SSD1306_DrawText(i,height,"SEO");

    //i=i+2;
   // height += Gulim7.Height;
   // SSD1306_SetFont(&Gulim7);
  //  SSD1306_DrawText(3,height,"MIN");

   // height += Gulim10.Height;
    SSD1306_SetFont(&Gulim7);
  //  SSD1306_DrawText(3,height,"WOO");
  // SSD1306_DrawChar(20,20,angleX+0x30);
    
  // SSD1306_DrawChar(10,10,ADC_Value%1000/100+0x30);
   //SSD1306_DrawChar(20,10,ADC_Value%100/10+0x30);
  // SSD1306_DrawChar(30,10,ADC_Value%10/1+0x30);
    
    
    SSD1306_DrawText(3,3,"Angle");
    SSD1306_DrawText(3,15,"distan");
    SSD1306_Rect(0,0,63,47);
    //printf("X : %5.2f, Y : %5.2f \r\n", angleX, angleY);
    //SerialPutChar(0x30);  // " ."

    
}

void SSD1306_DrawLineH(uint8_t x,uint8_t y,uint8_t leng)
{
    for(int i=x; i<=x+leng; i++)
        SetPoint(i,y);
    
    Display_Refresh();
}

void SSD1306_DrawLineV(uint8_t x,uint8_t y,uint8_t leng)
{
    for(int i=y; i<=y+leng; i++)
        SetPoint(x,i);
    
    Display_Refresh();
}

void SSD1306_SetFont(sFONT *pFont)
{
    SSD1306_pFont = pFont;
}

void SSD1306_Rect(uint8_t x,uint8_t y,uint8_t width,uint8_t height)
{
    SSD1306_DrawLineV(x,y,height);
    SSD1306_DrawLineV(x+width,y,height);
    SSD1306_DrawLineH(x,y,width);
    SSD1306_DrawLineH(x,y+height,width);
}

void SSD1306_DrawChar(uint8_t x, uint8_t y,uint8_t ch)
{
    uint8_t w,xa,ya;
    
    xa = ya = 0;
    
    for(int i=0; i<SSD1306_pFont->count; i++)
    {
        uint8_t data = SSD1306_pFont->table[ch*SSD1306_pFont->count + i];
        
        if( SSD1306_pFont->Width%8 == 0 )
            w = 8;
        else if( ((i+1)%(SSD1306_pFont->Width/8+1)) == 0 )
            w = SSD1306_pFont->Width%8;
        else 
            w = 8;
        
        for(int ix=0; ix<w; ix++)
        {
            if(data&0x80)       SetPoint(x+xa,y+ya);
            else                ClearPoint(x+xa,y+ya);
            
            data <<= 1;
            
            if(xa == (SSD1306_pFont->Width-1))
            {
                ya++;
                xa = 0;
                break;
            }
            else xa++;  
        }
    }
}

void SSD1306_DrawText(uint16_t x, uint16_t y, char *str)
{
    uint16_t cnt = 0;

    while(*str)
    {
         SSD1306_DrawChar(x+cnt*SSD1306_pFont->Width,y,*str++);
         cnt++;
    }
}

static void SetPoint(uint8_t x,uint8_t y)
{
    if(x > MAX_COL-1)      return;
    if(y > (MAX_PAGE*8)-1) return;
    
    FrameBuffer[y/8][x] |= (1<<(y%8));
}

static void ClearPoint(uint8_t x,uint8_t y)
{
    if(x > MAX_COL-1)      return;
    if(y > (MAX_PAGE*8)-1) return;
    
    FrameBuffer[y/8][x] &= ~(1<<(y%8));
}

static void Display_Refresh(void)
{
    Set_Page_Address(0);
    Set_Column_Address(0);
    for(int y=0; y<MAX_PAGE; y++)
        for(int x=0; x<MAX_COL; x++)
            SSD1306_WriteData(FrameBuffer[y][x]);
}

void SSD1306_Clear(void)
{
    for(int y=0; y<MAX_PAGE; y++)
        for(int x=0; x<MAX_COL; x++)
            FrameBuffer[y][x] = 0;
    
    Display_Refresh();
}

static void Set_Page_Address(uint8_t add)
{// 0 <= add <= 7
    add=0xb0|add;
    SSD1306_WriteCmd(add);
}

static void Set_Column_Address(uint8_t add)
{
    SSD1306_WriteCmd((0x10|(add>>4))+0x02);
    SSD1306_WriteCmd((0x0f&add));
}

static void SSD1306_WriteData(uint8_t data)
{
    CS_L;
    UTIL_DelayUS(1);
    DC_H;
    UTIL_DelayUS(1);
    SPI_SendByte(data);
    UTIL_DelayUS(1); 
    CS_H;
    UTIL_DelayUS(1);
}

static void SSD1306_WriteCmd(uint8_t cmd)
{
    CS_L;
    UTIL_DelayUS(1);
    DC_L;
    UTIL_DelayUS(1);
    SPI_SendByte(cmd);
    UTIL_DelayUS(1); 
    CS_H;
    UTIL_DelayUS(1);  
}

static void Spi_Init(void)
{
    __HAL_RCC_SPI2_CLK_ENABLE();
    
    SpiHandle.Instance               = SPI2;
    SpiHandle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
    SpiHandle.Init.Direction         = SPI_DIRECTION_2LINES;
    SpiHandle.Init.CLKPhase          = SPI_PHASE_2EDGE;
    SpiHandle.Init.CLKPolarity       = SPI_POLARITY_HIGH;
    SpiHandle.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
    SpiHandle.Init.CRCPolynomial     = 7;
    SpiHandle.Init.DataSize          = SPI_DATASIZE_8BIT;
    SpiHandle.Init.FirstBit          = SPI_FIRSTBIT_MSB;
    SpiHandle.Init.NSS               = SPI_NSS_SOFT;
    SpiHandle.Init.TIMode            = SPI_TIMODE_DISABLE;
    SpiHandle.Init.Mode = SPI_MODE_MASTER;
    
    HAL_SPI_Init(&SpiHandle);
    
    __HAL_SPI_ENABLE(&SpiHandle);
}

static void Gpio_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    __HAL_RCC_GPIOI_CLK_ENABLE();
    
    GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_2 | GPIO_PIN_4; //P0 = \CS, P2 = D/C, P4 = /RESET
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
    HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);
    
    GPIO_InitStruct.Pin       = GPIO_PIN_1 | GPIO_PIN_3; //P1 = sck, P3 = MOSI
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FAST;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
    HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);
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

static void SPI_SendByte(uint8_t data)
{
    while( ((SPI2->SR) & (SPI_FLAG_TXE)) != (SPI_FLAG_TXE) );
    SPI2->DR = data;
}