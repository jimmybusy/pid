#include "pid.h"
#include "math.h"

extern TIM_HandleTypeDef htim3;
void pidkernel(pidinfo* pid,const speed_info* speed_struct,int8_t expect_speed)//增量式
{		
		
		static float err_speed_old =0; 
		static float err_speed_older =0;
		pid->err_speed_new=expect_speed-speed_struct->speed;
		pid->err_speed_integral=pid->err_speed_new;
		pid->err_speed_differential=pid->err_speed_new-2.0f*err_speed_old+err_speed_older;
		pid->err_speed_p=pid->err_speed_new-err_speed_old;
		pid->err_speed_differential=pid->err_speed_new-err_speed_old;
		err_speed_older =err_speed_old;
		err_speed_old=pid->err_speed_new;
		pid->pid_out+=(pid->kp)*(pid->err_speed_p)+(pid->ki)*(pid->err_speed_integral)+(pid->kd)*(pid->err_speed_differential);//ki相当于位置式的kp控制纠正力度	
		pid->pid_out=limit(pid->pid_out,-100,100);
		
};
void pidskernel(pids_info* pid,const s_info *s,int8_t s_expect)//位置式pid kd控制回归速度
{		
		
		static float err_s_old =0; 
		pid->err_s_new=s_expect-s->s_measure;
		pid->err_s_integral+=pid->err_s_new;
		pid->err_s_differential=pid->err_s_new-err_s_old;
		err_s_old=pid->err_s_new;
		pid->pid_out=(pid->ksp)*(pid->err_s_new)+(pid->ksi)*(pid->err_s_integral)+(pid->ksd)*(pid->err_s_differential);
		pid->pid_out=limit(pid->pid_out,-100,100);		
};
int pid_trans_pwm(float pid_out)
{	
	return((limit(pid_out,-100,100)/100)*33);
};
void pidclean(pidinfo * pid)
{
	pid->err_speed_differential=0;
	pid->err_speed_p=0;
	pid->err_speed_integral=0;
	pid->err_speed_new=0;
	pid->err_speed_old=0;
	pid->pid_out=0;
};
void pidset(pidinfo* pid,float kp,float ki,float kd)
{
	pid->kp=kp;
    pid->ki=ki;
	pid->kd=kd;
};
void pidsclean(pids_info * pid)
{
	pid->err_s_differential=0;
	pid->err_s_integral=0;
	pid->err_s_new=0;
	pid->err_s_old=0;
	pid->pid_out=0;
};
void pidsset(pids_info* pid,float kp,float ki,float kd)
{
	pid->ksp=kp;
    pid->ksi=ki;
	pid->ksd=kd;
};
void motor_control(float pidout)
{	int pwm;
	pwm=pid_trans_pwm(pidout);
	if(pid_trans_pwm(pidout)>0)
		{
			htim3.Instance->CCR1=pwm;
			htim3.Instance->CCR2=0;
		}
	else
		{
			htim3.Instance->CCR1=0;
			htim3.Instance->CCR2=(uint16_t)-pwm;
		}
}
float limit(float pid_out,float least,float largest)
{
	if(pid_out>=least&&pid_out<=largest)
		{
			return(pid_out);
		}
	else if(pid_out<least)
		{
			return(least);
		}
	else 
		{
			return(largest);
		}
}
float fABS(float a)
{
	if(a>0)
	{
		return(a);
	}
	else
	{
		return(-a);
	}
}
