#ifndef MY_CFG
#define MY_CFG
#include "common.h"
/**************************************************/
/*
    ����˵�����ѽ������õĲ����������ŵ�my_cfg.c��my_cfg.h�У�
              �Ȱ���������ļ���ע�Ϳ�һ��
    ɽ��С����ʹ��˵������������������ѡ��ͼӼ������������Ƿ��أ�
                        ������ȷ�����κ�״̬�°��м䶼�ǿ�ʼ��
    ��һ���ȵ�ͼ������ν������ı��±ߵ�DOWN_EDGE��UP_EDGE
    �ڶ��������ڶ����ֵ���Լ���ֵ����ֵ�Ĳ�ֵ����ֵ������350��500֮�䣬
            ����ͨ������ FTM3_PRECISON ���ı��ֵ����MK60_FTM.h��
    �����������Զ�����Ѷ���̶�����ֵ�ϣ������Ƴ�����һ�γ�����ֱ������
            ���Զ�����ҵĲ�ֵ�Ƿ����������ô�����
    ���Ĳ����򿪺� OPEN_CIRCLE_CONTROL ���п������ԣ�������OPEN_CIRCLE_SPEED��ֵ
            �����ٶ��ܵ����٣������ȿ������ԣ��ı�ת��P���ı�·��
    ���岽���ص��� OPEN_CIRCLE_CONTROL ���бջ����ԣ�����PID���Լ�����ٶȺ���С�ٶ�
*/

//#define OPEN_CIRCLE_CONTROL //��������
#define OPEN_CIRCLE_SPEED 4300
//****�±����������ν����Ĺ�ʽ�������ĺ�
//***�ѳ�����ֱ���ϣ��������±ߵİ�ɫ������ DOWN_EDGE ��ֵ
//**�ı� UP_EDGE ��ֵ���ǲ�������ʾ�Ļ�ԭ���߽ӽ�ƽ�У�˵�������ɹ�
#define DOWN_EDGE 64   //ͼ���±��ص����ص����
#define UP_EDGE   16   //����ͼ���±�������ʾ�ĵȳ��ľ�����ͼ���ϱ�����ռ�����ص����  16
/*ͼ���ʹ�þ��룬��������õ�*/
#define MIX_distance  15//����ľ���  15
extern uint8 diff_valid_line;//ǰհ������ͨ��С��������


/*�������  ����--PTE5*/
#define DUO_JI FTM3 //����õĶ�ʱ��
#define DUO_JI_CH FTM_CH0//�����ʱ����ͨ��
#define dj_center 5120 //�������ֵ   3745   3670
#define DJ_DIFF 700   //�����������ֵ�Ĳ�ֵ
extern int direct_kp_array[];//���ת���Pֵ����my_cfg.c�ļ��и�
extern uint8 direct_Kd;//ת��D��ֵ����my_cfg.c�ļ��и�
extern float kp_val;//���е�p������������


/*������� ���ſ���PORT_CFG.h�и���*/
#define MOTOR_FTM   FTM0//��ʱ��0����
#define MOTOR1_PWM  FTM_CH5//ͨ��5 PTD5
#define MOTOR2_PWM  FTM_CH6//ͨ��6 PTD6
#define MOTOR3_PWM  FTM_CH2//ͨ��4 PTA5
#define MOTOR4_PWM  FTM_CH3//ͨ��3 PTA6
extern uint8 zhidao_speed;//ֱ���ٶȣ�����С��������
extern uint8 CD_speed;	  //ȫ���ٶȣ�����С��������
extern float speed_P,speed_D;//�����PID��������my_cfg.c�ļ��и�
extern float speed_I;

extern uint8 speed_table[][2];//С��������ʱ���Ĭ�������ٶȣ���my_cfg.c�ļ��и�
extern float diff_speed;//��������



/*��������Ӧ���ţ��õ��������룬������PORT_cfg.h�и�*/

//      ģ��ͨ��    �˿�          ��ѡ��Χ              ����
//#define FTM1_CH0_PIN    PTA8       //PTA8��PTA12��PTB0
//#define FTM1_CH1_PIN    PTA9       //PTA9��PTA13��PTB1
//#define FTM2_CH0_PIN    PTA10       //PTA10��PTB18
//#define FTM2_CH1_PIN    PTA11       //PTA11��PTB19

#endif


