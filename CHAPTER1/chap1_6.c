/****************************************************************************
*	The PID control algorithm with filter
*	带滤波器的PID控制算法
*
*	本例旨在验证低通滤波器的滤波性能，
*	低通滤波器对高频信号具有很好的滤波作用
*	
*	采样时间：1ms
*	干扰信号加在对象的输出端。分三种情况：
*		M=1时，为未加干扰信号；
*		M=2时，为加干扰信号未加滤波；
*		M=3时，为加干扰信号加滤波。
*
*	input:just for Step Signal
*	目前只针对阶跃信号输入情况
*****************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#define	 Kp		1.00
#define  Ki		0.08

//basic data of pid control 
struct pid_data
{
	float SetPoint;		//Desired Value
	float FeedBack;		//feedback value
	float err;			
	float integral;
	float u_sum;
};

typedef struct pid_data		pid_t;

//pid struct data init
struct pid_data* pid_init(float SetPoint, float FeedBack, float err, float integral, float u_sum)
{
	struct pid_data* tset = malloc(sizeof(struct pid_data));

	tset->SetPoint 	= SetPoint;
	tset->FeedBack 	= FeedBack; 				
	tset->err 		= err;		
	tset->integral 	= integral;
	tset->u_sum		= u_sum;

	return tset;
}

//The Increment PID Control Algorithm
float pid_calc(pid_t* pid)
{
	float filty,yyout,err;
	int M = 2;
	
	if(M == 1)
	{
		err = pid->SetPoint - pid->FeedBack;
		filty = pid->FeedBack;
	}

	float random = (float)(rand()%100)/100.0;
	float D = 5.0*random;
	yyout = pid->FeedBack + D;

	if(M == 2)
	{
		filty = yyout;
		err =  pid->SetPoint - filty;
	}

	//I separation
	if(abs(err) <= 0.8)
		pid->integral += err;
	else
		pid->integral = 0;

	pid->u_sum = Kp*err + Ki*pid->integral;
	pid->FeedBack += pid->u_sum*1.0;

	return pid->FeedBack;
}

int main()
{
	printf("System test begin \n");

	pid_t* tset;
	int count = 0;
	float real = 0;

	tset = pid_init(35,0,0,0,0);
	srand(time(NULL));
	while(count < 100)
	{
		real = pid_calc(tset);
		printf("%f\n",real);
		count++;
	}

	free(tset);
	return 0;
}
