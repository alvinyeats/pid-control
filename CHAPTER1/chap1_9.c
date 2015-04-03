/************************************************************************************
*	The PID control algorithm with dead zone
*	带死区的PID控制算法
*
*		在计算机控制系统中，某些系统为了避免控制作用过于频繁，消除由于频繁动作所
*	引起的振荡，可采用带死区的PID控制算法，控制算式为
*		当abs(e_k) <= abs(e0)时，e_k=0;
*		当abs(e_k) >  abs(e0)时，e_k=e_k.
*	式中，e_k为位置跟踪偏差，e_0是一个可调参数，其具体数值可根据实际控制对象由实验确定。
*	若e0值太小，会使控制动作过于频繁，达不到稳定被控对象的目的；若e0太大，则系统将产生
*	较大的滞后。
*		带死区的控制系统实际上是一个非线性系统，当abs(e_k)<=abs(e0)时，数字调节器输出
*	为零；当abs(e_k)>abs(e0)时，数字输出调节器有PID输出。
*
*	input:just for Step Signal
*	目前只针对阶跃信号输入情况
************************************************************************************/

#include <stdio.h>
#include <stdlib.h>

#define	 Kp		1.209
#define  Ki		0
#define	 Kd		0.001

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
	pid->err = pid->SetPoint - pid->FeedBack;
	if(abs(pid->err) <= 0.20)
		pid->integral += pid->err;
	else
		pid->integral = 0;
	
	pid->u_k = Kp*pid->err + Kd*(pid->err - pid->err_last) + Ki*pid->integral;

	int M = 2;
	if(M == 1)
	{}
	else if(M == 2)
	{
		if(abs(pid->err) <= 0.10)
			pid->u_k=0;
	}

	pid->FeedBack += pid->u_k*1.0;
	pid->err_last = pid->err;

	return pid->FeedBack;
}

int main()
{
	printf("System test begin \n");

	pid_t* tset;
	int count = 0;
	float real = 0;

	tset = pid_init(23,0,0,0,0);

	while(count < 100)
	{
		real = pid_calc(tset);
		printf("%f\n",real);
		count++;
	}

	free(tset);
	return 0;
}
