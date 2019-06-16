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
int   LeftLoseStart  = 0;//��¼��߶��ߵĿ�ʼ��
int   RightLoseStart = 0;//��¼�ұ߱߶��ߵĿ�ʼ��
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

unsigned char  CloseLoopFlag=0;//�Ƿ���Բ����־λ

unsigned int  process_flag=0;

unsigned int  enter_loop=0;//���뻷·
unsigned int  out_loop=0;//����·

int get_img_point(uint16 h,uint16 w);//����
//�����ҹյ㣬ʶ��
//һֱ��Ҫ����

void FindInflectionPoint()
{
	char i=0;
	int distance=0;
    int shizi_loop_black_point=0;
	int g_point_flag=0;

	//��������
	RightInflectionPointRow=0;//�ұߵĹյ��������
	RightInflectionPointCol=0;//�ұߵĹյ��������
	LeftInflectionPointRow=0;//��ߵĹյ��������
	LeftInflectionPointCol=0;//��ߵĹյ��������
	RightInflectionPointFlag=0;//�ҵ���յ�ı�־λ
	LeftInflectionPointFlag=0;//�ҵ��ҹյ�ı�־λ

	StartRow=0;//�ӵײ�����ɨ��Բ���м�ĺ�Բ����ʼ��
	StartCol=0;//�ӵײ�����ɨ��Բ���м�ĺ�Բ����ʼ��

	LoopBorttomFlag=0;//�ҵ�Բ���м�ĺ�Բ�ĵײ��ı�־λ
	LoopBorttomRow=0;//Բ���м�ĺ�Բ�ĵײ���������
	LoopMilldleRow=0;//Բ���м�ĺ�Բ�Ķ�����������
	LoopMilldleFlag=0;//�ҵ�Բ���м�ĺ�Բ�Ķ����ı�־λ
	LoopTopRow=0;//�ҵ��⻷�Ķ���
	MilldleBlack=0;//Բ���м�ĺ�Բ�Ķ�����Բ���м�ĺ�Բ�ĵײ����������ƽ��ֵ
	LoopFlag=0;//�ҵ������������͵ı�־

	//��յ�
	for(i=3;i<=32;i++) 
	{
		if((left_pos[i]!=40)&&(left_pos[i]!=79)) 
		{     
			if((left_pos[i]-left_pos[i+1]<0))//�ҵ��յ�
			{
				LeftInflectionPointRow=i;//��¼�ùյ����           
				LeftInflectionPointCol=left_pos[i];//��¼�ùյ����           
				LeftInflectionPointFlag=1;//����ҵ���յ�              
				break;//�ҵ��˳�                                  
			}
		}                                                                                                                                                                                                                                            
	} 

	//�ҹյ� 
	for(i=3;i<=32;i++)//����ɨ��̫Զ�����������
	{
		if((right_pos[i]!=40)&&(right_pos[i]!=1)) //�������в�����
		{     
			if((right_pos[i]-right_pos[i+1]>0))//�ҵ��ұ����йյ�
			{         
				RightInflectionPointRow=i;//��¼�յ����
				RightInflectionPointCol=right_pos[i];//��¼�յ����
				RightInflectionPointFlag=1;//����ҵ���յ�              
				break;//�ҵ��˳�
			}      
		} 
	}
	//��ʮ���п������гɻ�· ���е�ʱ�����ҹյ������ �����2���� ��69����
	//
	//����ͬʱ�ҵ������յ㣬��ʼʶ��·(������Կ���һ�£����ֻ�ҵ�һ���յ��������������ܸ�����ʶ�𵽻�·)

	if(LeftInflectionPointFlag&&RightInflectionPointFlag)//ͬʱ�ҵ������յ�
	{
		StartCol=(unsigned int)((LeftInflectionPointCol+RightInflectionPointCol)/2);// ȡ���ҹյ��������ƽ��ֵ    
		StartRow=(unsigned int)((LeftInflectionPointRow+RightInflectionPointRow)/2);//ȡ���ҹյ���������ƽ��ֵ    
        if(abs(StartCol-40)<4)
        {
            g_point_flag=1;
        }
        else
        {
            g_point_flag=0;
        }
		for(i=StartRow;i<60&&g_point_flag;i++)//�̶�һ�У�StartCol���ӿ�ʼ��������ɨ�裬Ѱ�һ�·�м��Բ���������ҵ�һ���׵��ڵ����䣬Ȼ����кڵ�Ȼ�󵽰ף�
		{
			if(!get_img_point(i,StartCol)&&get_img_point(i+1,StartCol))//������λ��
			{
				LoopBorttomRow=i;//��¼��һ��������� ����ʱ�ҵ��˰ױ�ڵ�λ�ò���¼��
				distance=LoopBorttomRow-StartRow;
				if(distance>15&&distance<35&&LoopBorttomRow<50)
				{
					LoopBorttomFlag=1;//��λ��־   ;           
					break; //�����Ѿ��ҵ������յ��һ�������
				}
				else if(LoopBorttomRow>52&&StartRow<6)
				{
				//           Shi_zi_flag=1;
				//           break; 
				}

			}     
		} 
		//�����ҵ��� �յ㣨�����������ͻ���λ�ã� �׺� �ڰ�����������λ�� ���Ҽ�¼��
		/********************************************************************/    
		if(LoopBorttomFlag)//����֮ǰ�Ѿ��ҵ�Բ�������������--�ж�������ȵ�ͻ��
		{//
            shizi_loop_black_point=beleved_shizi_loop();
			if(ABS(left_pos[StartRow+2]-right_pos[StartRow+2])-35>=4//4,8������������Ҫ����
			&&ABS(left_pos[StartRow+4]-right_pos[StartRow+4])-40>=8)//�������ͻ��
			{
				LoopFlag=1;//����������־λ��
				LoopRightControlFlag=1;//�õ���·��־λ�͸�����
				enter_loop=1;
			}                           
		}
	}
}




       

