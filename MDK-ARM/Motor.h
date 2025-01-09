#ifndef _MOTOR_H_
#define _MOTOR_H_

#include "main.h"

#include "stm32f1xx.h"
struct Motor
{
  int16_t rawEncode;      //bian ma qi fan hui de yuan shi 
  int16_t lastRawEncode;  //shang yi ci de bian ma qi yuan shi zhi
  int32_t round;          //quan shu
  int32_t conEncode;      //chulihoudelianxudebianmaqizhi
  int32_t lastConEncode;  //shang ci de chu li hou de lian xu bian ma zhi
  int16_t rawSpeed;       //fan hui de zhuan su 
  int16_t lastRawSpeed;   //shang yi ci fan hui de zhuan su 
  int16_t Current;        //zhuan ju dian liu 
  int8_t temp;            //wen du 
};

typedef struct pid_struct_t
{
  float Kp;
  float Ki;
  float Kd;
	float i_max;
	float out_max;
	float ref;   //mu biao jiao du 
	float fdb;   //she ding jiao du 
	float err[2];
	float p_out;
	float i_out;
	float d_out;
	float output;
}pid_struct_t;

typedef struct
{
  struct Motor data;
	  struct pid_struct_t PID_S;
    struct pid_struct_t PID_A;
}Motor_typedef;
void pid_init(pid_struct_t*pid,
	               float Kp,
                 float Ki,
                 float Kd,
	               float i_max,
	               float out_max);
void gimbal_PID_init(void);
float pid_calc(pid_struct_t*pid,float ref,float fdb);
void MotorResolve(Motor_typedef *motor, uint8_t *RxMessage);
void MotorRoundResolve(Motor_typedef *motor);
#endif