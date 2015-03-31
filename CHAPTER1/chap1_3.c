/****************************************************************************
*	The increment PID control algorithm
*	积分分离PID控制算法
*
*	积分分离的基本控制思路：
*		当被控量与设定值偏差较大时，取消积分作用，以免由于积分作用
*	使系统稳定性降低，超调量增大；当被控量接近给定值时，引入积分控制，以便
*	消除静差，提高控制精度。
*
*	input:just for Step Signal
*	目前只针对阶跃信号输入情况
*****************************************************************************/

#include <stdio.h>
#include <stdlib.h>

#define	 Kp		0.2
#define  Ki		0.05
#define	 Kd		0.2

//basic data of pid control 
struct pid_data
{
	float SetPoint;		//Desired Value
	float FeedBack;		//feedback value
	float err;			
	float err_last;
	float integral;
	float u_sum;
};

typedef struct pid_data		pid_t;

//pid struct data init
struct pid_data* pid_init(float SetPoint, float FeedBack, float err, float err_last,float integral, float u_sum)
{
	struct pid_data* tset = malloc(sizeof(struct pid_data));

	tset->SetPoint 	= SetPoint;
	tset->FeedBack 	= FeedBack; 				
	tset->err 		= err;		
	tset->err_last 	= err_last;
	tset->integral 	= integral;
	tset->u_sum		= u_sum;

	return tset;
}

//The Increment PID Control Algorithm
float pid_calc(pid_t* pid)
{
	pid->err 		= pid->SetPoint - pid->FeedBack;
	pid->integral  += pid->err;

	//judgment of Integral separation
	float err  = pid->err;
	float beta = 0;
	
	int M = 2;
	if(M == 1)
	{
		if((abs(err) >= 30) && (abs(err) <= 40))
			beta = 0.3;
		else if((abs(err) >= 20) && (abs(err) <=30))
			beta = 0.6;
		else if((abs(err) >= 10) && (abs(err) <=20))
			beta = 0.9;
		else
			beta = 1.0;
	}
	else if(M == 2)
	{
		beta = 1.0;
	}

	pid->u_sum = Kp*pid->err + Kd*(pid->err - pid->err_last) + beta*Ki*pid->integral;
	pid->FeedBack = pid->u_sum*1.0;
	pid->err_last = pid->err;

	return pid->FeedBack;
}

int main()
{
	printf("System test begin \n");

	pid_t* tset;
	int count = 0;
	float real = 0;

	tset = pid_init(35,0,0,0,0,0);

	while(count < 100)
	{
		real = pid_calc(tset);
		printf("%f\n",real);
		count++;
	}

	free(tset);
	return 0;
}

