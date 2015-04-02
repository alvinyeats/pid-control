/************************************************************************************
*	The derivative ahead PID control algorithm
*	微分先行PID控制算法
*
*		取M=1，采用微分先行PID控制方法；
*		取M=2，采用普通PID方法。
*
*		对于给定值rin(k)频繁升降的场合，引入微分先行后，可以避免给定值升降时
*	所引起的系统振荡，明显地改善了系统的动态特性。
*
*	input:just for Step Signal
*	目前只针对阶跃信号输入情况
************************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#define  gama 	0.50
#define  Td 	Kd/Kp
#define  Ti 	0.50
#define	 ts 	20

#define	 Kp		0.63
#define  Ki		0.252
#define	 Kd		1.06

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
	float c1,c2,c3;
	float ud_k,ud_1 = 0;
	c1 = gama*Td/(gama*Td + ts);
	c2 = (Td + ts)/(gama*Td + ts);
	c3 = Td/(gama*Td + ts); 

	pid->err = pid->SetPoint - pid->FeedBack;
	pid->integral += pid->err;
	
	float y_1 = 0;
	int M = 2;

	if(M == 1)
	{
		ud_k = c1*ud_1 + c2*pid->FeedBack - c3*y_1;
		pid->u_k = Kp*pid->err + ud_k + Ki*pid->integral;
	}
	else if(M == 2)
	{
		pid->u_k = Kp*pid->err + Kd*(pid->err - pid->err_last)/ts + Ki*pid->integral;
	}

	pid->FeedBack = pid->u_k*1.0;
	

	y_1 = pid->FeedBack;
	pid->err_last = pid->err;

	return pid->FeedBack;
}

int main()
{
	printf("System test begin \n");

	pid_t* tset;
	int count = 0;
	float real = 0;

	tset = pid_init(80,0,0,0,0);

	while(count < 1000)
	{
		real = pid_calc(tset);
		printf("%f\n",real);
		count++;
	}

	free(tset);
	return 0;
}
