/****************************************************************************
*	The integral saturation PID control algorithm
*	抗积分饱和PID控制算法
*
*	基本思想：
*		作为防止积分饱和的方法之一就是抗积分饱和法，该方法的思路是在
*	计算u(k)时，首先判断上一时刻的控制量u(k-1)是否已超出限制范围：
*		若u(k-1)>u_max,则只累加负偏差；若u(k-1)<u_max,则只累加正偏差。
*	这种算法可以避免控制量长时间停留在饱和区。
*
*	input:just for Step Signal
*	目前只针对阶跃信号输入情况
*****************************************************************************/

#include <stdio.h>
#include <stdlib.h>

#define	 Kp		0.2
#define  Ki		0.8
#define	 Kd		0.0

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
	pid->err = pid->SetPoint - pid->FeedBack;
	//pid->integral  += pid->err;

	float err   = pid->err;
	float u_sum = pid->u_sum; 
	int alpha = 0;
	
	int M = 2;
	int um = 6;
	if(M == 1)
	{	
		if(u_sum > um)
		{
			if(err >= 0)
				alpha = 0;
			else
				alpha = 1;
		}
		else if(u_sum <= -um)
		{
			if(err > 0)
				alpha = 1;
			else
				alpha = 0;
		}
		else
		{
			alpha = 1;
		}
	}
	else if(M==2)
		alpha = 1;

	pid->integral  += alpha*pid->err;
	pid->u_sum = Kp*pid->err + Kd*(pid->err - pid->err_last) + Ki*pid->integral;
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

	tset = pid_init(100,0,0,0,0,0);

	while(count < 100)
	{
		real = pid_calc(tset);
		printf("%f\n",real);
		count++;
	}

	free(tset);
	return 0;
}

