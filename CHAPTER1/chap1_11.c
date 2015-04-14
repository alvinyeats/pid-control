/************************************************************************************
*	The step PID control algorithm
*	步进式PID控制算法
*
*		在阶跃响应较大时，很容易产生超调。采用不仅是积分分离PID控制，该方法不直接
*	对阶跃信号进行响应，而是使输入指令信号一步一步地逼近所要求的阶跃信号，可使对象
*	运行平稳，适用于高精度伺服系统的位置跟踪。
*	
*	input:just for Step Signal
*	目前只针对阶跃信号输入情况
*
*	注：因算法本身的复杂度问题，该程序未整定完全，具体参数和计算方式请根据实际情况定义！
************************************************************************************/

#include <stdio.h>
#include <stdlib.h>

#define  ts 	0.001

#define	 Kp		80
#define  Ki		20
#define	 Kd		2.0

int k = 0;//循环次数，全局变量

//basic data of pid control 
struct pid_data
{
	float SetPoint;		//Desired Value
	float FeedBack;		//feedback value
	float err;			
	float integral;
	float u_k;
};

typedef struct pid_data		pid_t;

//pid struct data init
struct pid_data* pid_init(float SetPoint, float FeedBack, float err, float u_k)
{
	struct pid_data* tset = malloc(sizeof(struct pid_data));

	tset->SetPoint 	= SetPoint;
	tset->FeedBack 	= FeedBack; 				
	tset->err 		= err;		
	tset->u_k		= u_k;

	return tset;
}

//The Increment PID Control Algorithm
float pid_calc(pid_t* pid)
{
	float rate = 0.25,
		  rini = 0.0,
		  rd   = 20;

	int M=2;
	if(M==1)
	{
		pid->SetPoint = rd;
		pid->err = pid->SetPoint - pid->FeedBack;
	}

	if(M==2)
	{
		if(rini < (rd - 0.25))
			rini = rini + k*ts*rate;
		else if(rini > (rd + 0.25))
			rini = rini - k*ts*rate;
		else
			rini = rd;

		pid->SetPoint = rini;
		pid->err = pid->SetPoint - pid->FeedBack;
	}

	//PID with I separation
	if(abs(pid->err) <= 0.8)
		pid->integral += ts*pid->err;
	else
		pid->integral = 0;

	pid->u_k = Kp*pid->err + Ki*pid->integral;

	return pid->u_k;
}

int main()
{
	printf("System test begin \n");
	pid_t* tset;
	float real = 0;

	tset = pid_init(35,0,0,0);
	while(k < 100)
	{
		real = pid_calc(tset);
		printf("%f\n",real);
		k++;
	}

	free(tset);
	return 0;
}
