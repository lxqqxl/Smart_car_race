 /*!
  *     COPYRIGHT NOTICE
  *     Copyright (c) 2013,山外科技
  *     All rights reserved. 
  *     技术讨论：山外论坛 http://www.vcan123.com
  *
  *     除注明出处外，以下所有内容版权均属山外科技所有，未经允许，不得用于商业用途，
  *     修改内容时必须保留山外科技的版权声明。
  *
  * @file       VCAN_LED.h
  * @brief      led驱动头文件
  * @author     山外科技
  * @version    v5.0
  * @date       2013-07-9
  */

#ifndef __VCAN_LED_H__
#define __VCAN_LED_H__


/*! 枚举LED编号 */
typedef enum
{
  LED0,
  LED1,
  LED2,
  LED3,
  LED_MAX,
}LED_e;

/*! 枚举LED亮灭状态 */
typedef enum LED_status
{
  LED_ON  =   0,            //灯亮(对应低电平)
  LED_OFF =   1             //灯暗(对应高电平)
}LED_status; 


extern void    led_init(LED_e);            //初始化LED端口
extern void    led(LED_e,LED_status);      //设置LED灯亮灭
extern void    led_turn(LED_e);            //设置LED灯亮灭反转
extern uint8 lanzhou96x64[768];
 void OLED_Init(void);
 void OLED_CLS(void);
 void OLED_P6x8Str(uint8 x,uint8 y,uint8 ch[]);
 void OLED_P8x16Str(uint8 x,uint8 y,uint8 ch[]);
 void OLED_P14x16Str(uint8 x,uint8 y,uint8 ch[]);
 void OLED_Print(uint8 x, uint8 y, uint8 ch[]);
 void OLED_PutPixel(uint8 x,uint8 y);
 void OLED_Rectangle(uint8 x1,uint8 y1,uint8 x2,uint8 y2,uint8 gif);
 void OLED_Set_Pos(uint8 x, uint8 y);
 void OLED_WrDat(uint8 data);
 void Draw_LibLogo(void);
 void Draw_Landzo(uint8 img[60][80]);
 void Draw_BMP(uint8 x0,uint8 y0,uint8 x1,uint8 y1,uint8 bmp[]);
 void OLED_Fill(uint8 dat);
 void Dly_ms(uint16 ms);
void OLED_RefreshGRAM(void);
void OLED_DrawPoint(uint8 x,uint8 y,uint8 mode);
void OLED_Print_Num(uint8_t y, uint8_t x, uint16_t num);
#endif  //__VCAN_LED_H__