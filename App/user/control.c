#include "path.h"
#include "control.h"
#include "includes.h"
#include "speed_new.h"
#include "direction.h"
#include "path.h"
#include  "info_deal_save.h"  
#include "ring.h"
uint8 run_start_flag=0;//�Ƿ�ʼ��
uint8 img_deal_flag=0;//��ʼ�ɼ�����ͼ��
////////////////////////////////ȫ�ֱ�������////////////////////////////////////

uint16 Bline_left[CAMERA_H];	 //����ߴ������
uint16 Bline_right[CAMERA_H];	 //�ұ��ߴ������
uint16 Pick_table[CAMERA_H];	 //�����ߴ������

uint16 left_pos[CAMERA_H];	 //����ߴ������
uint16 right_pos[CAMERA_H];	 //�ұ��ߴ������
uint16 center_pos[CAMERA_H];	 //�����ߴ������

uint8  Pick_flag[CAMERA_H];//�����Ƿ��ҵ����߱�־����
uint8  Deal_flag[CAMERA_H];//���������Ƿ���Ч��־����
uint16 lost_already=0;
uint8 Pick_line=0;
uint16 maxBline_diff;
//uint8 PickCenter_flag=0;	//��ȡ���߱�־
uint8 Lost_Line_count=0;
uint8 Lost_left_count=0;
uint8 Lost_right_count=0;
uint8 Near_lost=0;//������ʧ��־
uint8 Shi_zi_line=0;
uint8 Shi_zi_num=0;
uint8 Shi_zi_flag=0;
uint8 stop_num=0;
int Position_diff=0;
uint8 Out_flag;		//�����־����Чͼ���־


uint8 last_vline;
uint8 valid_line=0;//�����Ч��
uint8 judge_vl;//�����жϵ���Ч��
uint8 last_lost=55;	//��һ����ʧ��
uint16 Bline_diff=0;//�����߾���

/***************************************************************/
int judge_xielv[CAMERA_H-5];	//б���ж����� 
uint8 zhidao_count_flag=0;	//ֱ���жϱ�־
uint8 last_zhidao_flag=0;
uint8 lost_w_count=0;//��ɫ��ʧ�б���
uint8 lost_b_count=0;//��ɫ��ʧ�б���

int even_diff=0;	//������ƽ��ƫ��
int even_diff_near=0;
float D_differen=0.0;   //
int jiaodu_num=dj_center;	//�Ƕ�ֵ
int last_turn=dj_center;	//��һ��ת��ֵ
int dj_pid_num=dj_center;	//����Ƕ�ֵ

int dj_act_jiaodu=dj_center;//���ʵ�ʽǶ�

uint8 set_flag=1;//���ñ�־λ
uint8 gl_zhangai_flag=0;
uint8 gl_zhangai_start=0;
uint8 gl_zhangai_end=0;

int ring_num=0;
int no_ring_num=0;
int RING_FLAG=0;
int HIGHT_RING_FLAG=0;
int one_flag=0;
void control_hander()
{
    if((PIT_TFLG(1)&PIT_TFLG_TIF_MASK)!=0)
    {
      if(run_start_flag)
      { //PTC14_OUT=1;
            control_speed();
       // PTC14_OUT=0;    
      }
    }
    PIT_TFLG(1)|=PIT_TFLG_TIF_MASK;       //���־
}
void duoji_hander()
{
    if((PIT_TFLG(2)&PIT_TFLG_TIF_MASK)!=0)
    {
        img_deal_flag=1;
    }
    PIT_TFLG(2)|=PIT_TFLG_TIF_MASK;       //���־
}
//���� ����ʼ�����Ƶ����ڣ�pid�����ڣ�
void init_control_circle()
{   
    set_vector_handler(PIT1_VECTORn,control_hander);   // �����жϷ��������ж���������
    set_vector_handler(PIT2_VECTORn,duoji_hander);   // �����жϷ��������ж���������
    pit_init_ms(PIT_PORT,PIT_CNT);
    pit_init_ms(PIT_PORT_time,PIT_time);//����ͼ��ɼ�����
    set_irq_priority((IRQn_Type)PIT1_VECTORn,2);
    set_irq_priority((IRQn_Type)PIT2_VECTORn,2);
}
void enable_pit_int(PITn_e pitn)
{
    enable_irq((IRQn_Type)((int)pitn + PIT0_IRQn));            //���ж�
}
void disable_pit_int(PITn_e pitn)
{
    disable_irq((IRQn_Type)((int)pitn + PIT0_IRQn));            //���ж�
}


void init_ctrl()
{
    init_control_circle(); //��ʼ��PIT1�����ڿ����ٶȲɼ����ڣ�û���ж�
    MOTOR_init();//��ʼ�����
    enable_pit_int(PIT2);//��ʼͼ��ɼ�
#ifdef OPEN_CIRCLE_CONTROL //����������Ʋ����������ٶȵ��ж�
      //qianjin();           //����ֱ����
#else
    enable_pit_int(PIT1);  //��PIT1�ж��л�ȡ������ֵ������ס�ٶ�
//    lptmr_pulse_counter(BMQ_L_PIN);//PORTA19 Ϊ�������ź����룬���ڱ���������
#endif
//    InitPar();//��ʼ������//���ڴ�������� ��ֵ��ȫ�ֱ���
   
}
void pirntf_path_type()
{
    switch(gl_path_type)
    {
    case Short_zhidao: printf("Short_zhidao\n");break;
    case lean_zhidao: printf("lean_zhidao\n");break;
    case Shi_zi: printf("Shi_zi\n");break;
    case Zhang_ai: printf("Zhang_ai\n");break;
    case Xiao_S: printf("Xiao_S\n");break;
    case Zhong_S: printf("Zhong_S\n");break;
    case Da_S: printf("Da_S\n");break;
    case Xiao_wan: printf("Xiao_wan\n");break;
    case Zhong_wan: printf("Zhong_wan\n");break;
    case Da_wan: printf("Da_wan\n");break;
    case T_Da_wan: printf("T_Da_wan\n");break;
    }
}
//�Ķ���ȡ�����У��Ķ��˵������Ķ���,�Ķ���ҡͷ���Ƶ��

/*******************************************************************************
�������ƣ�stable_del
��������: ��ǰ�����ݴ�����Ƴ���
��������
*******************************************************************************/
int big_fing_ring=0;
int last_duoji_e=0;
int loop_ok_num=0;//Բ���˲�����
void stable_del()
{   
        if(CloseLoopFlag==0)//�˲�֮ǰ���жϻ�·���ҹյ��׼ȷ��
        {
           FindInflectionPoint();//��һ���һ�· ���� �յ���
          // CloseLoopFlag==1;
        }
		xielv_lvbo();//�����ֵ���Ʒ��˲� ȥ����Ч��
		lvbo(5);//�˲�
      	ti_jiaozheng(START_Y,valid_line);//����ͼ�������ʧ��     
		bDistance();//������ȷ��˲� ȥ����Ч��
        if(Shi_zi_flag)
        {
            shizi_find_line();//��������//�������ж�Բ��©��
            bDistance();//������ȷ��˲� ȥ����Ч��
        }    
        if(LoopFlag)//����ǻ�·�Ͱ���һ����
        {
//          loop_ok_num++;
//          if(loop_ok_num>2)
//          {
           if(PTE7_IN)
          with_left_to_center();
          else 
          with_right_to_center();
//          }         
        }
        else//ֻҪ���ǻ�·���Լ��������
        {
          loop_ok_num=0;
          getBlineCenter();//��ϳ����� 
        }         
      	averageLvBo();//��ֵ�˲�
         center_buxian();//Ҳ�˲�
        if(Out_flag==1&&Shi_zi_flag==0)//�����˽����ж�
        {
//          jiaodu_num=last_turn;//�����ϴ�ת��
//          way_control();//���Ʒ���
        }
        else
        {                     
            If_LStraight();  //�ж��Ƿ�Ϊֱ��
            check_and_stop(total_time,mark_stop());//�����߼��
            get_even_diff_s();//��ȡ��Ч����������ƫ��ƽ��ֵ����ȡ��
            duoji_PD(even_diff);//�������ͨ�����PD��ȡ���ת��  even_diff���������                         
           
            way_control(); //���ת��
#ifdef OPEN_CIRCLE_CONTROL  //����ǿ�������
            qianjin();
#else             //�ջ���pid����ס
            if(run_start_flag)//��ʼ�ܵı��λ
            {
//                speed_ctl();  //�����ˣ����ж������
            }
            else
            {
//                stop_car();//ͣ��
            }
#endif           
        }

        if(total_time%80==0&&my_debug_flag==1)//������
        {
            printf("valid_line=%d\n",valid_line);
            printf("even_diff=%d\n",even_diff);
            printf("zhidao_count=%d\n",gl_zhidao_count);
            pirntf_path_type();
            if(Shi_zi_flag)
            {
                printf("sz=1\n");
            }
            if(zhidao_count_flag!=0)
            {
              printf("zd=%d\n",zhidao_count_flag);
            }
        }
}
/*******************************************************************************
�������ƣ�clearDelPar
��������: ���㴦�����
��������
*******************************************************************************/
void clearDelPar()
{
	lost_already=0;
	Lost_Line_count=0;
	Lost_left_count=0;
	Lost_right_count=0;
	Pick_line=0;
	Shi_zi_line=0;
	Bline_diff=0;
	maxBline_diff=0;
	Shi_zi_flag=0;
	lost_w_count=0;
	lost_b_count=0;
        valid_line=0;
        gl_zhangai_start=0;
        gl_zhangai_end=0;
        gl_zhangai_flag=0;
        guai_dian_count=0;
        donw_guan_dian=0;
}
/******************************* ������  **************************************/
void ctrl_main()
{
    uint32 line;
    /////////////////////ͼ��ɼ����֣��ҵ���������///////////////////////////////
    clearDelPar();//���㴦�����
    for(line=0;line<IMG_H;line++)//��ű���������������
    {
        Bline_left[line]=V/2;
        Bline_right[line]=V/2;
        Pick_flag[line]=0;//������־����
        Deal_flag[line]=0;//�����־����        
        left_pos[line]=V/2;
        right_pos[line]=V/2;
        center_pos[line]=0;        
    }
    for(line=START_Y+2;line<IMG_H;line++) //��ȡ�������ĵ㲢��������
    {
        if(line==START_Y+2)
        { 
            Near_lost=PickCenter_near();
        }
        else
        {
            PickCenter_diff(line);//Ѱ�����ұ��� �������е����꣬�ҵ��е����꣩ 
            //�ռ���·����Ҫ������
            left_pos[line]=Bline_left[line];//������ߵ�����
            right_pos[line]=Bline_right[line];//�����ұߵ�����
            center_pos[line]=(left_pos[line]+right_pos[line])/2;//�ϳ���������
        }
//////////////////////Բ���б�ʽ�����б�ʮ��֮ǰ�б��Ƿ�ΪԲ����//////////////////      
//         if(lost_w_count>=3&&ret_flag(line,30,9,18))//30,9,18
//        {  
//             RING_FLAG=1;
//             flag=1;
//        }
//��ǰ�������ж�Բ��  
////////////////////////////////ʮ���б�ʽ///////////////////////////////////////
         if(Shi_zi_flag==0&&lost_w_count>=8)//����Ч�ж�ʧ֮ǰ�ж϶��������ж�ʮ�ֵ�
        {           
              Shi_zi_flag=line;
              if(line>10)
                valid_line=line-10;
              else
                valid_line=0;
              break;
        }
///////////////////////////��Ч���ж�//////////////////////////////////////////
        if((Pick_flag[line]&LEFT_LOST_B)||(Pick_flag[line]&RIGHT_LOST_B))
          {
              if((Pick_flag[line-1]&LEFT_LOST_B)||(Pick_flag[line-1]&RIGHT_LOST_B))
                  lost_b_count++;
          }
          if(line<25)
          {
              if(maxBline_diff<Bline_diff)
                  maxBline_diff=Bline_diff;
          }
          if(lost_already==0)
          {
              if((lost_b_count>3||(Bline_left[line]<5)||(Bline_right[line]>(V-5)))&&valid_line==0)
              {
                  lost_already=1;
                  valid_line=line-3;
              }
          }
        if(line==IMG_H-1&&valid_line==0)
        {
            valid_line=IMG_H-1-lost_b_count;
        }
     }
    
     stable_del();//����ͼ��

}   
