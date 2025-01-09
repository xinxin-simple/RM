#include "Motor.h"
void MotorResolve(Motor_typedef *motor, uint8_t *RxMessage)
{
	 motor->data.lastRawEncode = motor->data.rawEncode;
   motor->data.rawEncode     = ((uint16_t)RxMessage[0] << 8 | (uint16_t)RxMessage[1]);
   motor->data.lastRawSpeed = motor->data.rawSpeed;
   motor->data.rawSpeed     = ((uint16_t)RxMessage[2] << 8 | (uint16_t)RxMessage[3]);
	 motor->data.Current = ((uint16_t)RxMessage[4] << 8 | (uint16_t)RxMessage[5]);
	 motor->data.temp = RxMessage[6];
}
extern pid_struct_t gimbal_yaw_speed_pid;
extern pid_struct_t gimbal_yaw_angle_pid;
void pid_init(pid_struct_t*pid,
	               float Kp,
                 float Ki,
                 float Kd,
	               float i_max,
	               float out_max)
{
	pid->Kp      =Kp;
	pid->Ki      =Ki;
	pid->Kd      =Kd;
	pid->i_max   =i_max;
	pid->out_max =out_max;
}

float pid_calc(pid_struct_t *pid, float ref, float fdb)
{
  pid->ref = ref;
  pid->fdb = fdb;
  pid->err[1] = pid->err[0];
  pid->err[0] = pid->ref - pid->fdb;
  pid->p_out  = pid->Kp * pid->err[0];
  pid->i_out += pid->Ki * pid->err[0];
  pid->d_out  = pid->Kd * (pid->err[0] - pid->err[1]);
  pid->output = pid->p_out + pid->i_out + pid->d_out;
  return pid->output;
}
void gimbal_PID_init()// jiao du huan he su du huan de chu shi hua 
{
	pid_init(&gimbal_yaw_speed_pid, 30, 0, 0, 30000, 30000);//P=30,I=0,D=0
	pid_init(&gimbal_yaw_angle_pid, 400, 0, 0, 0, 320);//P=400,I=0,D=0
}