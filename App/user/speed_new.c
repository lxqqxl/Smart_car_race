#include "common.h"
#include "include.h"
#include "speed_new.h"
#include  "info_deal_save.h"
#include "control.h"
#include "usart_file.h"
#include "direction.h"
#include "ring.h"
#include "path.h"
#include "math.h"
#include "my_cfg.h"
struct motor pidl={0};
struct motor pidr={0};
extern int stop_car_flag;
/*************************
电机PID初始化
*************************/
void PID_init(void)
{
	//左电机PID初始化
	pidl.P = speed_P;
	pidl.I = speed_I;
	pidl.D = speed_D;
	//右电机PID初始化
	pidr.P = speed_P;
	pidr.I = speed_I;
	pidr.D = speed_D;
}
void MOTOR_init()
{
	PID_init();
	//电机以及舵机初始化
	ftm_pwm_init(MOTOR_FTM, MOTOR1_PWM,MOTOR_HZ,0);      //初始化 电机 PWM
	ftm_pwm_init(MOTOR_FTM, MOTOR2_PWM,MOTOR_HZ,0);      //初始化 电机 PWM
	ftm_pwm_init(MOTOR_FTM, MOTOR3_PWM,MOTOR_HZ,0);      //初始化 电机 PWM
	ftm_pwm_init(MOTOR_FTM, MOTOR4_PWM,MOTOR_HZ,0);      //初始化 电机 PWM
	//正交编码，已经外部上拉
	ftm_quad_init(FTM1);                                 //FTM1 正交解码初始化（所用的管脚可查 port_cfg.h ）
	ftm_quad_init(FTM2);                                 //FTM2 正交解码初始化（所用的管脚可查 port_cfg.h ）
}
/******************************
        获取脉冲数
********************************/
void Getpluse(void)
{
	pidl.pluse = ftm_quad_get(FTM2);    //获取FTM2 正交解码 的脉冲数
	pidr.pluse = ftm_quad_get(FTM1);    //获取FTM1 正交解码 的脉冲数
	pidl.pluse =  pidl.pluse*6;
	pidr.pluse = - pidr.pluse*6;
	ftm_quad_clean(FTM2);
	ftm_quad_clean(FTM1);                 //清空脉冲计数器计算值（马上清空，这样才能保证计数值准确）
}

//float trends_motor_P(float p,int16 e)//动态 电机P(测试中)
//{
//    float ret=0.0;
//    if(abs(e)>10)
//    {    
//        ret=p+5;
//    }
//    else if(abs(e)>6)
//    {
//        ret=p-2;
//    }
//    else if(abs(e)>4)
//    {
//        ret=p-3;
//    }
//    else
//    {
//        ret=p-2;   
//    }
//
//    return ret;
//}
/********************************
    电机PID计算
**********************************/
void motorPID111()
{
    int16 error0,error1,error2;
    float P,I,D;
    //左电机
    error0 = error1 = error2 = 0;
    error0 = pidl.setPoint - (pidl.pluse);//I    
    error1 = error0 - pidl.lastError;//P
    error2 = error0 - 2*pidl.lastError + pidl.preError;//D
    pidl.preError = pidl.lastError;
    pidl.lastError = error0;
  
    P=pidl.P;    I=pidl.I;      D=pidl.D;
    pidl.PWM+= (int16)(P*error1 + I*error0 + D*error2);
    if(pidl.PWM > 9999)  pidl.PWM = 9999;
    if(pidl.PWM < -9999   )  pidl.PWM = -9999;

    //右电机
    error0 = error1 = error2 = 0;
    error0 = pidr.setPoint - (pidr.pluse);//I    
    error1 = error0 - pidr.lastError;//P
    error2 = error0 - 2*pidr.lastError + pidr.preError;//D
    pidr.preError = pidr.lastError;
    pidr.lastError = error0;    
    P=pidr.P;    I=pidr.I;      D=pidr.D;
    pidr.PWM+= (int16)(P*error1 + I*error0 + D*error2);
    if(pidr.PWM > 9999)  pidr.PWM = 9999;        
    if(pidr.PWM < -9999)  pidr.PWM = -9999;
}
//void motorPID111()
//{
//  int16 error0,error1,error2;
//  float P,I,D;
//  uint16 bangbang_val=2000,bangbang_error=800;
//  //左电机
//  error0 = error1 = error2 = 0;
//  error0 = pidl.setPoint - (pidl.pluse);//I
//  error1 = error0 - pidl.lastError;//P
//  error2 = error0 - 2*pidl.lastError + pidl.preError;//D
//  pidl.preError = pidl.lastError;
//  pidl.lastError = error0;
//  
//    P=pidl.P;    I=pidl.I;      D=pidl.D;
//    if(error0>bangbang_error)
//    {
//        pidl.PWM += bangbang_val;
//    }
//    else if(error0 <=-bangbang_error)
//    {
//        pidl.PWM -= bangbang_val;
//    }
//    else
//    {
//        pidl.PWM+= (int16)(P*error1 + I*error0 + D*error2);
//    }
//    if(pidl.PWM > 9999)  pidl.PWM = 9999;
//    if(pidl.PWM < -9999   )  pidl.PWM = -9999;
//
//    //右电机
//    error0 = error1 = error2 = 0;
//    error0 = pidr.setPoint - (pidr.pluse);//I
//    error1 = error0 - pidr.lastError;//P
//    error2 = error0 - 2*pidr.lastError + pidr.preError;//D
//    pidr.preError = pidr.lastError;
//    pidr.lastError = error0;
//    if(error0>bangbang_error)
//    {
//        pidr.PWM += bangbang_val;
//    }
//    else if(error0 <=-bangbang_error)
//    {
//        pidr.PWM -= bangbang_val;
//    }
//    else
//    {
//        pidr.out  =pidr.out + (int16)(P*error1 + I*error0 + D*error2);
//        pidr.PWM = pidr.out; 
//    }  
//    if(pidr.PWM > 9999)  pidr.PWM = 9999;        
//    if(pidr.PWM < -9999)  pidr.PWM = -9999;
//}

void motor_out()//1 3 方向
{
   if( 0)
   {
      M1(0);
      M2(0);
      M3(0);
      M4(0);
    ///  PTD9_OUT=0;
   }
   else{

    if(pidl.PWM >= 0)
    {
      M1(pidl.PWM);
      M2(0);
    }
    else
    {
      
      M2(-pidl.PWM);
      M1(0);
    }

    if(pidr.PWM >=0 )
    {
      M3(0);
      M4(pidr.PWM);
    }
    else
    {
      M4(0);
      M3(-pidr.PWM);
    }
   }
}
/********************************
    设置电机目标输出
*********************************/
int Vspeed=0;
int more_speed=0;
int scp=40;
void set_ideal_speed(int16 error)
{
	int left_speed,right_speed; 
	int max_speed=zhidao_speed,min_speed=CD_speed;
	int speed_diff_mark=7;//误差限速
	int img_max_error=33;//图像最大误差
	left_speed = right_speed = 0;
	if(zhidao_count_flag==2)//直道将最高速度调高
	{
	  max_speed=160;
      min_speed=100;
	}
	else if(zhidao_count_flag==1)
	{
	  max_speed=140;
      min_speed=100;
	}
    else if(LoopFlag)
    {
       max_speed=80;//适当减速
       min_speed=40;//适当减速      
    }
    
if(Shi_zi_flag&&PTE12_IN==0)

{   
      max_speed=80;//适当减速
       min_speed=80;//适当减速     
}
//     if(stop_car_flag==1)
//    {
//        max_speed=zhidao_speed=0,min_speed=CD_speed=0;      
//    }

    
	if(ramp_flag==1&&(abs(even_diff-0)<=2))//坡道重新调整最高速度
	{
		max_speed=80;
		min_speed=70;
	}
	if(gl_zhangai_flag)//障碍重新调整最高速度
	{
	    max_speed=80;
	    min_speed=70;
	} 
	
			
		if(even_diff>=img_max_error)
		{
			even_diff=img_max_error;
		}
		//公式=最高速度-误差的平方*速度差/最大误差的平方  得到的就是期望值速度
		more_speed=(int16)((even_diff*even_diff)*(max_speed-min_speed))/(img_max_error*img_max_error);
		Vspeed=(int)(max_speed - more_speed);//用2次方程调参
	                                               
	if(((even_diff <= -speed_diff_mark) || (even_diff >= speed_diff_mark))&&valid_line<56)
	{
		//误差和有效行同时满足时小车处于拐弯处    
		//  even_diff 大于零 左拐
		//  even_diff 小于零 右拐  
		if(even_diff <= -speed_diff_mark)                     //右拐
		{
			right_speed= (int8)((Vspeed * 70/100) - (even_diff*even_diff*diff_speed/2800)*valid_line/56);
			left_speed = (Vspeed * 100/100) ;
			if(left_speed<right_speed)
				right_speed=left_speed;
			if(right_speed < (Vspeed * 40/100) )
			{
				right_speed =  (Vspeed * 40/100);
			}
		} 
		else                                     //左拐
		{

			right_speed  = (Vspeed * 100/100);
			left_speed= (int8)((Vspeed * 70/100) - (even_diff*even_diff*diff_speed/2800)*valid_line/56);
			if(right_speed<left_speed)
				left_speed=right_speed;
			if(left_speed < (Vspeed * 40/100))
			{
				left_speed =  (Vspeed * 40/100);
			}
		}
	}
	else //if((even_diff > -speed_diff_mark) && (even_diff < speed_diff_mark))
	{
		left_speed   = Vspeed;
		right_speed  = Vspeed;
	}   
   pidl.setPoint = (int16)left_speed;//left_speed
   pidr.setPoint = (int16)right_speed;
//   if(stop_car_flag)
//   {
//    while(1);
//   }
// 	    pidl.setPoint = (int16)50;//left_speed
// 		pidr.setPoint = (int16)50;
        if(stop_car_flag==1)
    {scp--;
    if(scp<0)
    {
        pidl.setPoint = 0;//left_speed
   pidr.setPoint = 0;
    }}
//         if(max_speed<100)
//       PTD9_OUT=0;
//    else PTD9_OUT=1;

 
}
void control_speed()
{
    set_ideal_speed(jiaodu_num);
    Getpluse();
    motorPID111();
    motor_out();
}
void qianjin()
{
    int pid_out_L=OPEN_CIRCLE_SPEED;
    int pid_out_R=OPEN_CIRCLE_SPEED;
     if(jiaodu_num>0)
     {
        pid_out_L-=jiaodu_num*8;
        if(pid_out_L<0) pid_out_L=0;
        if(pid_out_R<0) pid_out_R=0;        
     }

    ftm_pwm_duty(MOTOR_FTM,MOTOR1_PWM ,pid_out_L);
    ftm_pwm_duty(MOTOR_FTM,MOTOR2_PWM,0);

    ftm_pwm_duty(MOTOR_FTM,MOTOR3_PWM ,pid_out_R);
    ftm_pwm_duty(MOTOR_FTM,MOTOR4_PWM,0);
}