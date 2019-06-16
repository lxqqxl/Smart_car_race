#include "path.h"
#include "control.h"
#include "includes.h"
#include "speed_new.h"
#include "direction.h"
#include "path.h"
#include  "info_deal_save.h"  
#include "ring.h"
uint8 run_start_flag=0;//是否开始跑
uint8 img_deal_flag=0;//开始采集处理图像
////////////////////////////////全局变量定义////////////////////////////////////

uint16 Bline_left[CAMERA_H];	 //左边线存放数组
uint16 Bline_right[CAMERA_H];	 //右边线存放数组
uint16 Pick_table[CAMERA_H];	 //中心线存放数组

uint16 left_pos[CAMERA_H];	 //左边线存放数组
uint16 right_pos[CAMERA_H];	 //右边线存放数组
uint16 center_pos[CAMERA_H];	 //中心线存放数组

uint8  Pick_flag[CAMERA_H];//该行是否找到黑线标志数组
uint8  Deal_flag[CAMERA_H];//处理数据是否有效标志数组
uint16 lost_already=0;
uint8 Pick_line=0;
uint16 maxBline_diff;
//uint8 PickCenter_flag=0;	//提取中线标志
uint8 Lost_Line_count=0;
uint8 Lost_left_count=0;
uint8 Lost_right_count=0;
uint8 Near_lost=0;//近处丢失标志
uint8 Shi_zi_line=0;
uint8 Shi_zi_num=0;
uint8 Shi_zi_flag=0;
uint8 stop_num=0;
int Position_diff=0;
uint8 Out_flag;		//出界标志，无效图像标志


uint8 last_vline;
uint8 valid_line=0;//最大有效行
uint8 judge_vl;//用于判断的有效行
uint8 last_lost=55;	//上一场丢失行
uint16 Bline_diff=0;//两黑线距离

/***************************************************************/
int judge_xielv[CAMERA_H-5];	//斜率判断数组 
uint8 zhidao_count_flag=0;	//直道判断标志
uint8 last_zhidao_flag=0;
uint8 lost_w_count=0;//白色丢失行变量
uint8 lost_b_count=0;//黑色丢失行变量

int even_diff=0;	//中心线平均偏差
int even_diff_near=0;
float D_differen=0.0;   //
int jiaodu_num=dj_center;	//角度值
int last_turn=dj_center;	//上一次转向值
int dj_pid_num=dj_center;	//舵机角度值

int dj_act_jiaodu=dj_center;//舵机实际角度

uint8 set_flag=1;//设置标志位
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
    PIT_TFLG(1)|=PIT_TFLG_TIF_MASK;       //清标志
}
void duoji_hander()
{
    if((PIT_TFLG(2)&PIT_TFLG_TIF_MASK)!=0)
    {
        img_deal_flag=1;
    }
    PIT_TFLG(2)|=PIT_TFLG_TIF_MASK;       //清标志
}
//功能 ；初始化控制的周期（pid的周期）
void init_control_circle()
{   
    set_vector_handler(PIT1_VECTORn,control_hander);   // 设置中断服务函数到中断向量表里
    set_vector_handler(PIT2_VECTORn,duoji_hander);   // 设置中断服务函数到中断向量表里
    pit_init_ms(PIT_PORT,PIT_CNT);
    pit_init_ms(PIT_PORT_time,PIT_time);//用于图像采集周期
    set_irq_priority((IRQn_Type)PIT1_VECTORn,2);
    set_irq_priority((IRQn_Type)PIT2_VECTORn,2);
}
void enable_pit_int(PITn_e pitn)
{
    enable_irq((IRQn_Type)((int)pitn + PIT0_IRQn));            //开中断
}
void disable_pit_int(PITn_e pitn)
{
    disable_irq((IRQn_Type)((int)pitn + PIT0_IRQn));            //开中断
}


void init_ctrl()
{
    init_control_circle(); //初始化PIT1，用于控制速度采集周期，没开中断
    MOTOR_init();//初始化电机
    enable_pit_int(PIT2);//开始图像采集
#ifdef OPEN_CIRCLE_CONTROL //如果开环控制不开启控制速度的中断
      //qianjin();           //开环直接跑
#else
    enable_pit_int(PIT1);  //在PIT1中断中获取编码器值并控制住速度
//    lptmr_pulse_counter(BMQ_L_PIN);//PORTA19 为计数器信号输入，用于编码器计数
#endif
//    InitPar();//初始化参数//从内存读出数据 赋值给全局变量
   
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
//改动了取中线行，改动了调整，改动了,改动了摇头舵机频率

/*******************************************************************************
函数名称：stable_del
函数功能: 以前的数据处理控制程序
参数：无
*******************************************************************************/
int big_fing_ring=0;
int last_duoji_e=0;
int loop_ok_num=0;//圆环滤波变量
void stable_del()
{   
        if(CloseLoopFlag==0)//滤波之前先判断环路（找拐点更准确）
        {
           FindInflectionPoint();//第一次找环路 按照 拐点找
          // CloseLoopFlag==1;
        }
		xielv_lvbo();//跳变差值限制法滤波 去除无效行
		lvbo(5);//滤波
      	ti_jiaozheng(START_Y,valid_line);//矫正图像的梯形失真     
		bDistance();//赛道宽度法滤波 去除无效行
        if(Shi_zi_flag)
        {
            shizi_find_line();//重新找线//里面有判断圆环漏判
            bDistance();//赛道宽度法滤波 去除无效行
        }    
        if(LoopFlag)//如果是环路就按照一边走
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
        else//只要不是环路就自己拟合中线
        {
          loop_ok_num=0;
          getBlineCenter();//拟合出中线 
        }         
      	averageLvBo();//均值滤波
         center_buxian();//也滤波
        if(Out_flag==1&&Shi_zi_flag==0)//出界了进入判断
        {
//          jiaodu_num=last_turn;//保持上次转角
//          way_control();//控制方向
        }
        else
        {                     
            If_LStraight();  //判断是否为直道
            check_and_stop(total_time,mark_stop());//起跑线检测
            get_even_diff_s();//获取有效行内中心线偏离平均值（获取误差）
            duoji_PD(even_diff);//传入误差通过舵机PD获取舵机转角  even_diff大于零左拐                         
           
            way_control(); //舵机转向
#ifdef OPEN_CIRCLE_CONTROL  //如果是开环控制
            qianjin();
#else             //闭环用pid控制住
            if(run_start_flag)//开始跑的标记位
            {
//                speed_ctl();  //不用了，在中断里干了
            }
            else
            {
//                stop_car();//停车
            }
#endif           
        }

        if(total_time%80==0&&my_debug_flag==1)//调试用
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
函数名称：clearDelPar
函数功能: 清零处理参数
参数：无
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
/******************************* 主函数  **************************************/
void ctrl_main()
{
    uint32 line;
    /////////////////////图像采集部分，找到两条边线///////////////////////////////
    clearDelPar();//清零处理参数
    for(line=0;line<IMG_H;line++)//存放边线中线数组清零
    {
        Bline_left[line]=V/2;
        Bline_right[line]=V/2;
        Pick_flag[line]=0;//赛道标志清零
        Deal_flag[line]=0;//处理标志清零        
        left_pos[line]=V/2;
        right_pos[line]=V/2;
        center_pos[line]=0;        
    }
    for(line=START_Y+2;line<IMG_H;line++) //提取各行中心点并处理意外
    {
        if(line==START_Y+2)
        { 
            Near_lost=PickCenter_near();
        }
        else
        {
            PickCenter_diff(line);//寻找左右边线 （输入行的坐标，找到列的坐标） 
            //收集环路所需要的坐标
            left_pos[line]=Bline_left[line];//保存左边的坐标
            right_pos[line]=Bline_right[line];//保存右边的坐标
            center_pos[line]=(left_pos[line]+right_pos[line])/2;//合成中心坐标
        }
//////////////////////圆环判别式（在判别十字之前判别是否为圆环）//////////////////      
//         if(lost_w_count>=3&&ret_flag(line,30,9,18))//30,9,18
//        {  
//             RING_FLAG=1;
//             flag=1;
//        }
//以前在这里判断圆环  
////////////////////////////////十字判别式///////////////////////////////////////
         if(Shi_zi_flag==0&&lost_w_count>=8)//在有效行丢失之前判断丢白线数判断十字道
        {           
              Shi_zi_flag=line;
              if(line>10)
                valid_line=line-10;
              else
                valid_line=0;
              break;
        }
///////////////////////////有效行判断//////////////////////////////////////////
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
    
     stable_del();//处理图像

}   
