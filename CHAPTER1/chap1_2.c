#include <stdio.h>
#include <stdlib.h>
#include <math.h>


#define	 Kp		0.2
//#define  Ki		0.015
#define	 Kd		0.2
#define  ts		0.001  //sampling time
#define	 Max	10.0
#define	 Min 	8.0
struct pid_data
{
	float SetData;
	float ActualData;
	float err;
	float err_last;
	float sum;
}pid;


void PID_init()
{
	printf("PID_init begin\n");
	pid.SetData = 0.0;				
	pid.ActualData = 0.0;		
	pid.err = 0.0;
	pid.err_last = 0.0;
	pid.sum = 0.0;
	printf("PID_init end\n");
}

float PID_realize(float desired)
{
	if(desired > Max)
		desired = Max;
	else if(desired < Min)
		desired = Min;

	pid.SetData     =  desired;
	pid.err 	    =  pid.SetData - pid.ActualData;
	pid.sum         =  Kp*pid.err + Kd*(pid.err - pid.err_last)/ts;
	pid.err_last    =  pid.err;
	pid.ActualData  =  pid.sum*1.0;

	return pid.ActualData;
}

int main()
{
	printf("System begin \n");
	PID_init();
	int count = 0;
	while(count < 100)
	{
		float real= PID_realize(11.0);
		printf("%f\n",real);
		count++;
	}

	return 0;
}
