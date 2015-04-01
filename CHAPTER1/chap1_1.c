/***************************************
*	The positional PID control algorithm
*	位置式PID控制
*
*	input:just for Step Signal
*	目前只针对阶跃信号输入情况
***************************************/
#include <stdio.h>
#include <stdlib.h>

#define	 Kp		0.32
#define  Ki		0.64
#define	 Kd		0.04

struct pid_data
{
	float SetPoint;
	float FeedBack;
	float err;
	float err_last;
	float integral;
	float u_sum;
};

typedef struct pid_data		pid_t;


//pid struct data init
struct pid_data* pid_init(float SetPoint,float FeedBack,
						  float err, float err_last, float u_sum)
{
	struct pid_data* tset = malloc(sizeof(struct pid_data));

	tset->SetPoint 	= SetPoint;
	tset->FeedBack 	= FeedBack; 				
	tset->err 		= err;		
	tset->err_last 	= err_last;
	tset->u_sum 	= u_sum;

	return tset;
}


float pid_calc(pid_t* pid)
{
	pid->err     	=  pid->SetPoint - pid->FeedBack;
	pid->integral  +=  pid->err;
	pid->u_sum	 	=  Kp*pid->err + Ki*pid->integral + Kd*(pid->err - pid->err_last);
	pid->err_last   =  pid->err;
	pid->FeedBack	=  pid->u_sum*1.0;

	return pid->FeedBack;
}

int main()
{
	printf("System test begin \n");

	pid_t* tset;
	int count = 0;
	float real = 0;

	tset = pid_init(89,0,0,0,0);

	while(count < 100)
	{
		real = pid_calc(tset);
		printf("%f\n",real);
		count++;
	}

	free(tset);
	return 0;
}
