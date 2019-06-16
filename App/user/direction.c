#include "direction.h"
#include  "MK60_FTM.h"
#include  "info_deal_save.h"
#include "system_MK60DZ10.h"
#include "path.h"
#include "speed_new.h"
#include "control.h"
#include "my_UI.h"
#include "my_cfg.h"
#include "ring.h"
#define LOCA_DEADLINE 2
#define LOCA_MAX dj_left_max-dj_center
#define LOCA_MIN dj_right_max-dj_center
float loca_Kp=0,loca_Kd=0,loca_Ki=0;
float loca_PreIntegral=0,loca_PreError=0,loca_PreU=0;

void duoji_init()
{
    ftm_pwm_init(DUO_JI,DUO_JI_CH,60,dj_center);
}
void change_angle(uint32 duty)
{
    dj_act_jiaodu=duty;
    ftm_pwm_duty(DUO_JI,DUO_JI_CH,duty);
}
//�Զ����صĳ�ʼ��
void init_dircetion()
{
    //��ʼ�������ر���
    duoji_init();
}
//   ��һ��������PID�����ṹ�壬�������£�
struct _pid{
    float SetSpeed;            //�����趨ֵ
    float ActualSpeed;        //����ʵ��ֵ
    float err;                //����ƫ��ֵ
    float err_last;            //������һ��ƫ��ֵ
    float Kp,Ki,Kd;            //������������֡�΢��ϵ��
    float voltage;          //�����ѹֵ������ִ�����ı�����
    float integral;            //�������ֵ
}duoji_pid;


float get_Kp_from_path()
{ 
    float kp=0;
    
     //**ע��**�±ߵ�һ���ж���ͨ��һ���Ǹ��ݲ�ͬ�ĵ�·���ͻ�ȡһ��������kp��
     //���KP�Ǳ�֤�����Ļ����������±��н��ܣ��ٶ�����ʱ����Թ̶�һ��������֮ǰһ���̶�������2.5��    
    if(zhidao_count_flag&&guai_dian_count==0)
    {
		kp = direct_kp_array[Short_zhidao];
		gl_path_type=Short_zhidao;
    }  
    else if(valid_line>52&&guai_dian_count==0)//��ֱ����
    {
		kp=direct_kp_array[lean_zhidao];
		gl_path_type=lean_zhidao;
    }
    else if(stop_car_line)//��·
    {
        kp=0;
        valid_line=IMG_H;
    }
    else if(LoopFlag)//��·
    {
        kp=direct_kp_array[Loop_road];
    }
    else if(Shi_zi_flag)
    {
        kp=direct_kp_array[Shi_zi];
        gl_path_type=Shi_zi;
    }
    else if(valid_line>=48&&guai_dian_count)
    {
        if(gl_zhidao_count>40)//СS��
        { 
			kp=direct_kp_array[Xiao_S];
			gl_path_type=Xiao_S;
//			diff_speed=-300;
        }
        if(gl_zhidao_count>25)//��S��
		{
			kp=direct_kp_array[Zhong_S];
		    gl_path_type=Zhong_S;
		}  
		else//��S��
		{
            kp=direct_kp_array[Da_S];
			gl_path_type=Da_S;
		}       
     }
     else//����
     {
         if(valid_line>45)//45
         {              
         	kp=direct_kp_array[Xiao_wan];
            gl_path_type=Xiao_wan;
         }
         else if(valid_line>37)//37
         {
			kp=direct_kp_array[Zhong_wan];
            gl_path_type=Zhong_wan;
         }
         else if(valid_line>30)//30
         {
            kp=direct_kp_array[Da_wan];
            gl_path_type=Da_wan;
         }
         else
         {
            kp=direct_kp_array[T_Da_wan];
            gl_path_type=T_Da_wan;
        }
    }
    return kp+kp_val+(IMG_H-valid_line)*(IMG_H-valid_line)/22;//22 33 35 
}
//float get_Kp_from_path()
//{ 
//    float kp=0;
//    
//     //**ע��**�±ߵ�һ���ж���ͨ��һ���Ǹ��ݲ�ͬ�ĵ�·���ͻ�ȡһ��������kp��
//     //���KP�Ǳ�֤�����Ļ����������±��н��ܣ��ٶ�����ʱ����Թ̶�һ��������֮ǰһ���̶�������2.5��    
//    if(zhidao_count_flag&&guai_dian_count==0)
//    {
//		kp = -16;
//		gl_path_type=Short_zhidao;
//    }  
//    else if(valid_line>52&&guai_dian_count==0)//��ֱ����
//    {
//		kp=-13;
//		gl_path_type=lean_zhidao;
//    }
//    else if(LoopFlag)//��·
//    {
//        kp=80;
//    }
//   
//    return kp+20+(IMG_H-valid_line)*(IMG_H-valid_line)/22;//22 33 35 
//}

float get_duoji_Kp()
{
    return get_Kp_from_path();//������Ч�л�ȡkp�����˼�����ʽ����ȽϺ���
}
//ͳһ��ʼ��������������Kp,Ki,Kd�������������Թ��̵��У�����Ҫ��Ŀ���Ч��������ͨ��������������ֱ�ӽ��е��ڡ�
//����������д�����㷨���������£�
void duoji_PD(int err)
{
    duoji_pid.Kp=get_duoji_Kp();//��ȡ��̬��KP
//    duoji_pid.Kd=myPar_num.DJ_KP[5];
    duoji_pid.err=-err;//��ֵ
    jiaodu_num=(int)(duoji_pid.Kp*duoji_pid.err+direct_Kd*(duoji_pid.err-duoji_pid.err_last));//���PD��ʽ
    duoji_pid.err_last=duoji_pid.err;//��¼���
    if(jiaodu_num>LOCA_MAX)
    {
        jiaodu_num=LOCA_MAX;
    }
    if(jiaodu_num<LOCA_MIN)
    {
        jiaodu_num=LOCA_MIN;
    }
}
/*******************************************************************************
�������ƣ�way_control
��������: 
������
*******************************************************************************/
void way_control()
{
    duoji_control(jiaodu_num);
    last_turn=jiaodu_num;
}