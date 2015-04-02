/***************************************
*	The integral separate PID control arithmetic
*	增量式PID控制算法
*
*	input:just for Step Signal
*	目前只针对阶跃信号输入情况
***************************************/

#include <stdio.h>
#include <stdlib.h>

#define	 Kp		0.3
#define  Ki		0.60
#define	 Kd		0.0375

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


//Integral Separate PID Control Arithmetic
float pid_calc(pid_t* pid)
{
	float iErr,pErr,dErr;
	iErr = pid->SetPoint - pid->FeedBack;			//The current error 
	pErr = iErr - pid->PreError;					//Proportion of incremental error
	dErr = iErr - 2*pid->LastError + pid->PreError;	//Differential incremental error
	
	pid->u_sum = Kp*pErr + Ki*iErr + Kd*dErr;		//Control increase quantity
	
	pid->PreError  = pid->LastError;
	pid->LastError = iErr;
	
	pid->FeedBack += pid->u_sum*1.0;
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