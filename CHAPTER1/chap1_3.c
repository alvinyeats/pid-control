/****************************************************************************
*	The Increment PID Control Algorithm
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

#define	 Kp		0.8
#define  Ki		0
#define	 Kd		0

//basic data of pid control 
struct pid_data
{
	float SetPoint;		//Desired Value
	float FeedBack;		//feedback value
	float LastError;	//Error[-1]
	float PreError;		//Error[-2]
	float u_sum;
};

typedef struct pid_data		pid_t;

//pid struct data init
struct pid_data* pid_init(float SetPoint, float FeedBack, float LastError, float PreError, float u_sum)
{
	struct pid_data* tset = malloc(sizeof(struct pid_data));

	tset->SetPoint 	= SetPoint;
	tset->FeedBack 	= FeedBack; 				
	tset->LastError = LastError;		
	tset->PreError 	= PreError;
	tset->u_sum 	= u_sum;

	return tset;
}

//The Increment PID Control Algorithm
float pid_calc(pid_t* pid)
{
	float Err;
	pErr = pid->SetPoint - pid->FeedBack;

	float threshold = 20.0;//set the threshold value
	
	//judgment of Integral separation
	if(abs(Err) <= threshold)
	{
		u_sum = Kp*pErr + Ki*iErr + Kd*dErr;


	}
	else
	{
		u_sum = Kp*pErr + Kd*dErr;

	}

}