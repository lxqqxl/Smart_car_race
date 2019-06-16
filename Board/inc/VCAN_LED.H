 /*!
  *     COPYRIGHT NOTICE
  *     Copyright (c) 2013,ɽ��Ƽ�
  *     All rights reserved. 
  *     �������ۣ�ɽ����̳ http://www.vcan123.com
  *
  *     ��ע�������⣬�����������ݰ�Ȩ����ɽ��Ƽ����У�δ������������������ҵ��;��
  *     �޸�����ʱ���뱣��ɽ��Ƽ��İ�Ȩ������
  *
  * @file       VCAN_LED.h
  * @brief      led����ͷ�ļ�
  * @author     ɽ��Ƽ�
  * @version    v5.0
  * @date       2013-07-9
  */

#ifndef __VCAN_LED_H__
#define __VCAN_LED_H__


/*! ö��LED��� */
typedef enum
{
  LED0,
  LED1,
  LED2,
  LED3,
  LED_MAX,
}LED_e;

/*! ö��LED����״̬ */
typedef enum LED_status
{
  LED_ON  =   0,            //����(��Ӧ�͵�ƽ)
  LED_OFF =   1             //�ư�(��Ӧ�ߵ�ƽ)
}LED_status; 


extern void    led_init(LED_e);            //��ʼ��LED�˿�
extern void    led(LED_e,LED_status);      //����LED������
extern void    led_turn(LED_e);            //����LED������ת
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