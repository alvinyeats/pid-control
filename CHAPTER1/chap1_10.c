/************************************************************************************
*	The PID control algorithm based on feedforward compensation
*	基于前馈补偿的PID控制算法
*
*		在高精度伺服控制中，前馈控制可用来提高系统的跟踪性能。经典控制理论中的
*	前馈控制设计是基于复合控制思想，当闭环系统为连续系统时，使前馈环节与闭环系
*	统的传递函数之积为1，从而实现输出完全复现输入。作者利用前馈控制的思想，针对
*	PID控制设计了前馈补偿，以提高系统的跟踪性能
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
	pid->integral += pid->err;


	//需引入前馈补偿量drin_k和ddrin_k
	float up_k,uf_k;
	float drin_k  = 0.65,
		  ddrin_k = 0.45;
		  	 
	up_k = Kp*pid->err + Ki*pid->integral + Kd*(pid->err - pid->err_last)/ts;
	uf_k = (25/133)*drin_k + (1/133)*ddrin_k;

	int M=2;
	if(M==1)
		pid->u_k = up_k;
	else if(M==2)
		pid->u_k = up_k + uf_k;

	pid->err_last = pid->err;

	return pid->u_k;
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
