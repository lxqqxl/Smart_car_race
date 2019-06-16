#include "usart_file.h"
#include "control.h"
#include "img_array.h"
#include "path.h"
#include "usart_file.h"
#include  "info_deal_save.h"
#include "my_UI.h"
#include "direction.h"
#include "speed_new.h"

uint8 imgbuff[CAMERA_SIZE];                             //����洢����ͼ�������
uint32 total_time=0;
//uint8 img[CAMERA_H][CAMERA_W];

//�������� z120 
void PORTA_IRQHandler();
void DMA0_IRQHandler();
uint8 my_debug_flag=0;
void my_delay(uint32_t z)
{
    while(z--);
}
void draw_mark_line()
{
    uint16 x=0,y=0;
    Site_t site;
    for(y=0;y<IMG_H;y+=10)
    {
        for(x=0;x<IMG_W;x++)
        {
            site.x=x;
            site.y=y;
            LCD_point(site,RED);
        }
    }
    for(y=0;y<IMG_H;y++)
    {
        for(x=0;x<IMG_W;x+=10)
        {
            site.x=x;
            site.y=y;
            LCD_point(site,RED);
        }
    }
}
//��ÿ����                                                         
void ring_chose_init()//���ڻ�·ѡ��ĳ�ʼ�������뿪�أ�
{//���֧���ĸ���·
  gpio_init(PTE7,GPI,0);
  gpio_init(PTE10,GPI,0);
  gpio_init(PTE11,GPI,0);
  gpio_init(PTE12,GPI,0);   
  port_init_NoALT(PTE7,PULLUP);
  port_init_NoALT(PTE10,PULLUP);
  port_init_NoALT(PTE11,PULLUP);
  port_init_NoALT(PTE12,PULLUP);
  //gpio_get(PTE10)==1;//��ȡ���뿪�ص�ֵ
}
void motor_dir_init()//D5 D6 A7 A5 (���� ���� ���� ����)
{/************************************************************************
  ���õ������
  */////////////////////////////////////////
  gpio_init(PTD5,GPI,0);
  gpio_init(PTA7,GPI,0); 
  port_init_NoALT(PTD5,PULLUP);
  port_init_NoALT(PTD5,PULLUP);
}
void init_fun()
{
    key_init(KEY_MAX);//������ʼ��  
    duoji_init();//���pwm��ʼ��
    LCD_init();  //������ʼ��    
    init_Par_from_FLASH();//�ڲ�flash��ʼ�������ڴ���������ڵ��εĲ���
    //usart_init();//���ڳ�ʼ��
    camera_init(imgbuff);//�����ʼ��
   //�����жϷ�����
    set_vector_handler(PORTA_VECTORn , PORTA_IRQHandler);   //���� PORTA ���жϷ�����Ϊ PORTA_IRQHandler
    set_vector_handler(DMA0_VECTORn , DMA0_IRQHandler);     //���� DMA0 ���жϷ�����Ϊ PORTA_IRQHandler
}

void check_start_key()
{
    if(key_get(KEY_U)==0||key_get(KEY_R)==0||key_get(KEY_START)==0||key_get(KEY_A)==0)
    {
        DELAY_MS(1000);
        run_start_flag=~run_start_flag;
        total_time=0;
    }
    else if(key_get(KEY_D)==0||key_get(KEY_L)==0)
    {
        DELAY_MS(500);
        if(key_get(KEY_D)==0||key_get(KEY_L)==0)
        {
            if(my_debug_flag) 
              my_debug_flag=0;
            else my_debug_flag=1;
        }
    }
}
/*!
 *  @brief      main����
 *  @since      v5.0
 *  @note       ɽ������ͷ LCD ---------------------����ʵ��
 */
;
extern  uint8 DMA_Over_Flg;  

void  main(void) 
{
  //����������****һ��Ҫ�ȶ�my_cfg.h�е�ע��**��***������������
    run_start_flag=0;
    init_fun();//���ֳ�ʼ��  (���� ��� ���� flash ����ͷ �ж�����)
    renew_UI();//�����ʼ������ͨ���������Σ�ֱ�����������   
    init_ctrl();//��ʼ���ٶȿ��ƣ��ж� �����
    PID_init(); 
    //DELAY_MS(2000);//��ʱ���뷢��
     uart_init (UART4, 115200); 
    run_start_flag=1;//��ʼ�ܵı�־λ
    total_time=0;//��¼���д����������߼��ʱ�õ���������һ��ʱ���ڼ��������  
    //gpio_init(PTD9,GPO,0);
     //gpio_init(PTC14,GPO,0);
     gpio_init(PTE7,GPI,0);
     gpio_init(PTE12,GPI,0);
    //motor_dir_init();
    //change_angle(3670);//�������λ�ã����ڲ��ԣ�
    while(1)
    {       // uart_putchar (UART4, 11);  
        //cmd_deal();        //���ڴ����ڷ��������� ������������         
        if(DMA_Over_Flg ==1 )
        {//PTC14_OUT=~PTC14_OUT;
            DMA_Over_Flg=0;        
            camera_get_img();  //����ͷ��ȡͼ��  
            ctrl_main();       //����ͼ�񣬼�������Ƶ���ں���     
            total_time++;//��¼���ƵĴ���
        }
    }
}

/*!
 *  @brief      PORTA�жϷ�����
 *  @since      v5.0
 */
void PORTA_IRQHandler()
{
    uint8  n;    //���ź�
    uint32 flag;

    while(!PORTA_ISFR);
    flag = PORTA_ISFR;
    PORTA_ISFR  = ~0;                                   //���жϱ�־λ

    n = 29;                                             //���ж�
    if(flag & (1 << n))                                 //PTA29�����ж�
    {
        camera_vsync();
    }
#if ( CAMERA_USE_HREF == 1 )                            //ʹ�����ж�
    n = 28;
    if(flag & (1 << n))                                 //PTA28�����ж�
    {
        camera_href();
    }
#endif


}

/*!
 *  @brief      DMA0�жϷ�����
 *  @since      v5.0
 */
void DMA0_IRQHandler()
{
    camera_dma();
}


//����ͼ����λ����ʾ
//��ͬ����λ������ͬ���������ʹ�� yy_����ͷ���ڵ��� ���
//���ʹ��������λ��������Ҫ�޸Ĵ���
void sendimg(void *imgaddr, uint32 imgsize)
{
    uint8 cmd[4] = {0, 255, 1, 0 };    //yy_����ͷ���ڵ��� ʹ�õ�����

    uart_putbuff(VCAN_PORT, (uint8_t *)cmd, sizeof(cmd));    //�ȷ�������

    uart_putbuff(VCAN_PORT, imgaddr, imgsize); //�ٷ���ͼ��
}

