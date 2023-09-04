#ifndef _PID_H_
#define _PID_H_
#endif
#include "main.h"
typedef struct
	{
		float kp;
		float ki;
		float kd;
		float err_speed_p;
		float err_speed_new;
		float err_speed_old;
		float err_speed_integral;
		float err_speed_differential;
		float pid_out;		
	} pidinfo;	
typedef struct
	{
		float ksp;
		float ksi;
		float ksd;
		float err_s_new;
		float err_s_old;
		float err_s_integral;
		float err_s_differential;
		float pid_out;
		
	} pids_info;
typedef enum 
	{	
		forward,
		back
	}fangxiang;

typedef struct
	{
		 fangxiang direction;
		 int16_t value;
		 float speed;	
	}speed_info;
typedef struct
	{
		 fangxiang direction;
		 float s_measure;	
	}s_info;
void pidclean(pidinfo* pid);
void pidkernel(pidinfo* pid,const speed_info *speed,int8_t expect_speed);
void pidskernel(pids_info* pid,const s_info *s,int8_t s_expect);
int pid_trans_pwm(float pid_out);
void pidset(pidinfo * pid,float kp,float ki,float kd);
void motor_control(float pidout);
float limit(float pid_out,float least,float largest);
float fABS(float a);
void pidsclean(pids_info * pid);
void pidsset(pids_info* pid,float kp,float ki,float kd);
