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
//对舵机相关的初始化
void init_dircetion()
{
    //初始化舵机相关变量
    duoji_init();
}
//   第一步：定义PID变量结构体，代码如下：
struct _pid{
    float SetSpeed;            //定义设定值
    float ActualSpeed;        //定义实际值
    float err;                //定义偏差值
    float err_last;            //定义上一个偏差值
    float Kp,Ki,Kd;            //定义比例、积分、微分系数
    float voltage;          //定义电压值（控制执行器的变量）
    float integral;            //定义积分值
}duoji_pid;


float get_Kp_from_path()
{ 
    float kp=0;
    
     //**注意**下边的一堆判断是通过一堆是根据不同的道路类型获取一个基础的kp，
     //这个KP是保证不抖的基本参数，下边有介绍，速度慢的时候可以固定一个常数，之前一个固定参数到2.5米    
    if(zhidao_count_flag&&guai_dian_count==0)
    {
		kp = direct_kp_array[Short_zhidao];
		gl_path_type=Short_zhidao;
    }  
    else if(valid_line>52&&guai_dian_count==0)//在直到上
    {
		kp=direct_kp_array[lean_zhidao];
		gl_path_type=lean_zhidao;
    }
    else if(stop_car_line)//环路
    {
        kp=0;
        valid_line=IMG_H;
    }
    else if(LoopFlag)//环路
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
        if(gl_zhidao_count>40)//小S弯
        { 
			kp=direct_kp_array[Xiao_S];
			gl_path_type=Xiao_S;
//			diff_speed=-300;
        }
        if(gl_zhidao_count>25)//中S弯
		{
			kp=direct_kp_array[Zhong_S];
		    gl_path_type=Zhong_S;
		}  
		else//大S弯
		{
            kp=direct_kp_array[Da_S];
			gl_path_type=Da_S;
		}       
     }
     else//大弯
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
//     //**注意**下边的一堆判断是通过一堆是根据不同的道路类型获取一个基础的kp，
//     //这个KP是保证不抖的基本参数，下边有介绍，速度慢的时候可以固定一个常数，之前一个固定参数到2.5米    
//    if(zhidao_count_flag&&guai_dian_count==0)
//    {
//		kp = -16;
//		gl_path_type=Short_zhidao;
//    }  
//    else if(valid_line>52&&guai_dian_count==0)//在直到上
//    {
//		kp=-13;
//		gl_path_type=lean_zhidao;
//    }
//    else if(LoopFlag)//环路
//    {
//        kp=80;
//    }
//   
//    return kp+20+(IMG_H-valid_line)*(IMG_H-valid_line)/22;//22 33 35 
//}

float get_duoji_Kp()
{
    return get_Kp_from_path();//根据有效行获取kp，试了几个方式这个比较好用
}
//统一初始化变量，尤其是Kp,Ki,Kd三个参数，调试过程当中，对于要求的控制效果，可以通过调节这三个量直接进行调节。
//第三步：编写控制算法，代码如下：
void duoji_PD(int err)
{
    duoji_pid.Kp=get_duoji_Kp();//获取动态的KP
//    duoji_pid.Kd=myPar_num.DJ_KP[5];
    duoji_pid.err=-err;//误差赋值
    jiaodu_num=(int)(duoji_pid.Kp*duoji_pid.err+direct_Kd*(duoji_pid.err-duoji_pid.err_last));//舵机PD公式
    duoji_pid.err_last=duoji_pid.err;//纪录误差
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
函数名称：way_control
函数功能: 
参数：
*******************************************************************************/
void way_control()
{
    duoji_control(jiaodu_num);
    last_turn=jiaodu_num;
}