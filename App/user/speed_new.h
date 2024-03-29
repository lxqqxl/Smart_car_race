#ifndef _CONTROL_H
#define _CONTROL_H

#define MOTOR_HZ   10*1000//10KHZ 10*1000

#define M1(duty)   ftm_pwm_duty(MOTOR_FTM, MOTOR1_PWM,duty) //PTD5//方向
#define M2(duty)   ftm_pwm_duty(MOTOR_FTM, MOTOR2_PWM,duty) //PTD6
#define M3(duty)   ftm_pwm_duty(MOTOR_FTM, MOTOR3_PWM,duty) //PTA7//方向
#define M4(duty)   ftm_pwm_duty(MOTOR_FTM, MOTOR4_PWM,duty) //PTA5

//换了电机驱动
//#define M1(duty)   ftm_pwm_duty(MOTOR_FTM, MOTOR1_PWM,duty)
//#define M2(duty)   ftm_pwm_duty(MOTOR_FTM, MOTOR2_PWM,duty)
//#define M3(duty)   ftm_pwm_duty(MOTOR_FTM, MOTOR3_PWM,duty)
//#define M4(duty)   ftm_pwm_duty(MOTOR_FTM, MOTOR4_PWM,duty)

struct motor
{
  int16 pluse;
  int16 setPoint;
  float P;
  float I;
  float D;
  int16 lastError;
  int16 preError;
  int16 out;
  int16 PWM;
};
extern struct motor pidl;
extern struct motor pidr;
extern float diff_speed;
extern void PID_init(void);
extern void MOTOR_init(void);
extern void Getpluse(void);
extern void speed_Contorl(void);
extern void motorPID();      //正常PID
extern void motorPID111();  //带反转ＰＩＤ
void control_speed();
#endif