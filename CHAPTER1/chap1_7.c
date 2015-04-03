/************************************************************************************
*	The incomplete differential PID control algorithm
*	不完全微分PID控制算法
*
*		在PID控制中，微分信号的引入可改善系统的动态性能，但也容易引进高频干扰，
*	在误差扰动突变时尤其显出微分项的不足。若在控制算法中加入低通滤波器，则可使
*	系统性能得到改善。
*		克服上述缺点的方法之一是在PID算法中加入一个一阶惯性环节（低通滤波器）
*	G_t(s)=1/(1+T_f_s)
*		当M=1时，采用具有不完全微分PID方法；
*		当M=2时，采用普通PID方法；
*
*		引入不完全微分，能有效的克服普通PID的不足。尽管不完全微分PID控制算法比普
*	通PID控制算法要复杂些，但由于其良好的控制特性，近年来越来越得到广泛的应用。
*	
*	input:just for Step Signal
*	目前只针对阶跃信号输入情况
************************************************************************************/

#include <stdio.h>
#include <stdlib.h>

#define  ts 	20
#define  Tf 	180
#define  TD 	140

#define	 Kc		0.035
#define  Ki		0.265
#define	 Kd		(float)(Kc*TD)/ts

//basic data of pid control 
struct pid_data
{
	float SetPoint;		//Desired Value
	float FeedBack;		//feedback value
	float err;			
	float err_last;
	float integral;
	float u_k;
};

typedef struct pid_data		pid_t;

//pid struct data init
struct pid_data* pid_init(float SetPoint, float FeedBack, float err, float err_last, float u_k)
{
	struct pid_data* tset = malloc(sizeof(struct pid_data));

	tset->SetPoint 	= SetPoint;
	tset->FeedBack 	= FeedBack; 				
	tset->err 		= err;		
	tset->err_last 	= err_last;
	tset->u_k		= u_k;

	return tset;
}

//The Increment PID Control Algorithm
float pid_calc(pid_t* pid)
{
	float alfa,ud_k;
	float ud_1=0;
	int M = 1;
	
	pid->err = pid->SetPoint - pid->FeedBack;
	pid->integral += pid->err;
	if(M == 1)
	{
		alfa = Tf/(ts+Tf);
		ud_k = Kd*(1 - alfa)*(pid->err - pid->err_last) + alfa*ud_1;
		ud_1 = ud_k;
		pid->u_k = Kc*pid->err + ud_k + Ki*pid->integral;
	}
	else if(M == 2)
	{
		pid->u_k = Kc*pid->err + Kd*(pid->err - pid->err_last) + Ki*pid->integral;
	}

	pid->FeedBack = pid->u_k*1.0;
	pid->err_last = pid->err;

	return pid->FeedBack;
}

int main()
{
	printf("System test begin \n");

	pid_t* tset;
	int count = 0;
	float real = 0;

	tset = pid_init(35,0,0,0,0);

	while(count < 100)
	{
		real = pid_calc(tset);
		printf("%f\n",real);
		count++;
	}

	free(tset);
	return 0;
}
