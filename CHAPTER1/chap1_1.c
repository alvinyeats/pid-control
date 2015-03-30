/***************************************
*	The Positional PID Control Algorithm
*	位置式PID控制
*
*	input:just for Step Signal
*	目前只针对阶跃信号输入情况
***************************************/
#include <stdio.h>

#define	 Kp		0.2
#define  Ki		0.05
#define	 Kd		0.2

struct pid_data
{
	float SetData;
	float ActualData;
	float err;
	float err_last;
	float integral;
	float u_sum;
}pid;

void PID_init()
{
	printf("PID_init begin\n");
	
	pid.SetData 	= 0.0;				
	pid.ActualData 	= 0.0;		
	pid.err 		= 0.0;
	pid.err_last 	= 0.0;
	pid.integral 	= 0.0;
	pid.u_sum		= 0.0;

	printf("PID_init end\n");
}

float PID_realize(float desired)
{
	pid.SetData     =  desired;
	pid.err 	    =  pid.SetData - pid.ActualData;
	pid.integral   +=  pid.err;
	pid.u_sum      	=  Kp*pid.err + Ki*pid.integral + Kd*(pid.err - pid.err_last);
	pid.err_last    =  pid.err;
	pid.ActualData  =  pid.u_sum*1.0;

	return pid.ActualData;
}

int main()
{
	printf("System test begin \n");

	PID_init();
	int count = 0;
	float real = 0;
	while(count < 1000)
	{
		real= PID_realize(23);
		printf("%f\n",real);
		count++;
	}

	return 0;
}
