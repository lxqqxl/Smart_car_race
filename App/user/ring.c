#include "ring.h"
#include  "common.h"
#include "include.h"
#include "path.h"
#include "control.h"

int   MidPri         = 40;
int   LastLine       = 0;
float AvaliableLines = 0;
int   LeftLose       = 0;
int   RightLose      = 0; 
int   AllLose        = 0;
int   LeftLoseStart  = 0;//记录左边丢线的开始行
int   RightLoseStart = 0;//记录右边边丢线的开始行
int   WhiteStart     = 0;


/*********define for GetBlackEndParam**********/
int BlackEndMR      = 0;
int BlackEndML      = 0;
int BlackEndLL      = 0;
int BlackEndRR      = 0;
int BlackEndL       = 0;
int BlackEndM       = 0;
int BlackEndR       = 0;
int BlackEndMaxMax  = 0;
int BlackEndMax     = 0;
int DropRow         = 0;

/*********define for FindInflectionPoint()**********/

int RightInflectionPointRow=0;
int RightInflectionPointCol=0;
int LeftInflectionPointRow=0;
int LeftInflectionPointCol=0;
unsigned char RightInflectionPointFlag=0;
unsigned char LeftInflectionPointFlag=0;
unsigned char LeftInflectionPointSecondFlag=0;
unsigned char RightInflectionPointSecondFlag=0;
unsigned char LoopFlag=0;
unsigned char LoopRightOkFlag=0;
unsigned char LoopLeftOkFlag=0;
unsigned int StartRow=0;
unsigned int StartCol=0;
unsigned char MilldleBlack=0;
unsigned int LoopTop=0;
unsigned int LoopRightBorderLose=0;
unsigned int LoopLeftBorderLose=0;
int LoopBorttomFlag=0;
int LoopBorttomRow=0;
int LoopMilldleRow=0;
unsigned int LoopMilldleFlag=0;
unsigned int LoopTopRow=0;
unsigned int LoopLeft=0;
unsigned int MilldlePonit=0;
unsigned int LoopRight=0;
unsigned int LoopRightR=0;
unsigned int LoopLeftL=0;
int BigLoopLeftUp[60];
int BigLoopRightUp[60];
int BigLooptUp[80];

unsigned char LoopRightControlFlag=0;

unsigned char  CloseLoopFlag=0;//是否开启圆环标志位

unsigned int  process_flag=0;

unsigned int  enter_loop=0;//进入环路
unsigned int  out_loop=0;//出环路

int get_img_point(uint16 h,uint16 w);//声明
//找左右拐点，识别环
//一直都要开着

void FindInflectionPoint()
{
	char i=0;
	int distance=0;
    int shizi_loop_black_point=0;
	int g_point_flag=0;

	//变量清零
	RightInflectionPointRow=0;//右边的拐点的行坐标
	RightInflectionPointCol=0;//右边的拐点的列坐标
	LeftInflectionPointRow=0;//左边的拐点的行坐标
	LeftInflectionPointCol=0;//左边的拐点的列坐标
	RightInflectionPointFlag=0;//找到左拐点的标志位
	LeftInflectionPointFlag=0;//找到右拐点的标志位

	StartRow=0;//从底部往上扫描圆环中间的黑圆的起始行
	StartCol=0;//从底部往上扫描圆环中间的黑圆的起始列

	LoopBorttomFlag=0;//找到圆环中间的黑圆的底部的标志位
	LoopBorttomRow=0;//圆环中间的黑圆的底部的行坐标
	LoopMilldleRow=0;//圆环中间的黑圆的顶部的行坐标
	LoopMilldleFlag=0;//找到圆环中间的黑圆的顶部的标志位
	LoopTopRow=0;//找到外环的顶部
	MilldleBlack=0;//圆环中间的黑圆的顶部和圆环中间的黑圆的底部的行坐标的平均值
	LoopFlag=0;//找到环形赛道类型的标志

	//左拐点
	for(i=3;i<=32;i++) 
	{
		if((left_pos[i]!=40)&&(left_pos[i]!=79)) 
		{     
			if((left_pos[i]-left_pos[i+1]<0))//找到拐点
			{
				LeftInflectionPointRow=i;//记录该拐点的行           
				LeftInflectionPointCol=left_pos[i];//记录该拐点的列           
				LeftInflectionPointFlag=1;//标记找到左拐点              
				break;//找到退出                                  
			}
		}                                                                                                                                                                                                                                            
	} 

	//右拐点 
	for(i=3;i<=32;i++)//不能扫描太远，否则会误判
	{
		if((right_pos[i]!=40)&&(right_pos[i]!=1)) //连续三行不丢线
		{     
			if((right_pos[i]-right_pos[i+1]>0))//找到右边线有拐点
			{         
				RightInflectionPointRow=i;//记录拐点的行
				RightInflectionPointCol=right_pos[i];//记录拐点的列
				RightInflectionPointFlag=1;//标记找到左拐点              
				break;//找到退出
			}      
		} 
	}
	//在十字有可能误判成环路 误判的时候左右拐点的坐标 大概在2附近 和69附近
	//
	//可以同时找到两个拐点，开始识别环路(或许可以考虑一下，如果只找到一个拐点的情况，这样就能更容易识别到环路)

	if(LeftInflectionPointFlag&&RightInflectionPointFlag)//同时找到两个拐点
	{
		StartCol=(unsigned int)((LeftInflectionPointCol+RightInflectionPointCol)/2);// 取左右拐点的列坐标平均值    
		StartRow=(unsigned int)((LeftInflectionPointRow+RightInflectionPointRow)/2);//取左右拐点的行坐标的平均值    
        if(abs(StartCol-40)<4)
        {
            g_point_flag=1;
        }
        else
        {
            g_point_flag=0;
        }
		for(i=StartRow;i<60&&g_point_flag;i++)//固定一列（StartCol）从开始的行往上扫描，寻找环路中间的圆的特征（找到一个白到黑的跳变，然后多行黑的然后到白）
		{
			if(!get_img_point(i,StartCol)&&get_img_point(i+1,StartCol))//跳变点的位置
			{
				LoopBorttomRow=i;//记录第一次跳变的行 （此时找到了白变黑的位置并记录）
				distance=LoopBorttomRow-StartRow;
				if(distance>15&&distance<35&&LoopBorttomRow<50)
				{
					LoopBorttomFlag=1;//置位标志   ;           
					break; //到此已经找到两个拐点和一个跳变点
				}
				else if(LoopBorttomRow>52&&StartRow<6)
				{
				//           Shi_zi_flag=1;
				//           break; 
				}

			}     
		} 
		//以上找到了 拐点（就是赛道宽度突变的位置） 白黑 黑白两个跳变点的位置 并且记录了
		/********************************************************************/    
		if(LoopBorttomFlag)//在这之前已经找到圆环的两个跳变点--判断赛道宽度的突变
		{//
            shizi_loop_black_point=beleved_shizi_loop();
			if(ABS(left_pos[StartRow+2]-right_pos[StartRow+2])-35>=4//4,8这两个参数需要调整
			&&ABS(left_pos[StartRow+4]-right_pos[StartRow+4])-40>=8)//赛道宽度突变
			{
				LoopFlag=1;//环形赛道标志位置
				LoopRightControlFlag=1;//得到环路标志位就给方向
				enter_loop=1;
			}                           
		}
	}
}




       

