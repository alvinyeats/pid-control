/****************************************************************************
*	The gearshift integral PID control algorithm
*	变速积分PID控制算法
*
*	基本思想：
*		在普通的PID控制算法中，由于积分系数Ki是常数，所以在整个控制过程中，
*	积分增量不变。而系统对积分项的要求是，系统偏差大时积分作用应减弱甚至全
*	无，而在偏差小时则应加强。积分系数取大了会产生超调，甚至积分饱和，取小
*	了又迟迟不能消除静差。因此，如何根据系统偏差大小改变积分的速度，对于提
*	高系统品质是很重要的。变速积分PID可较好的解决这一问题。
*		变速积分PID的基本思想是设法改变积分项的累加速度，使其与偏差大小相对
*	应：偏差越大，积分越慢，反之则越快。
*
*	input:just for Step Signal
*	目前只针对阶跃信号输入情况
*****************************************************************************/

#include <stdio.h>
#include <stdlib.h>

#define	 Kp		0.8
#define  Ki		0.7
#define	 Kd		0.05

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
	pid->err 		= pid->SetPoint - pid->FeedBack;
	pid->integral  += (pid->err + pid->err_last)/2;

	
	float err  = pid->err;
	
	int M = 2;
	int A = 0.4,B = 0.6;
	float flag;

	if(M == 1)
	{
		if(abs(err) <= B)
			flag = 1.0;
		else if((abs(err) > B) && (abs(err) <= (A+B)))
			flag = (A - abs(err) + B)/A;
		else
			flag = 0.0;
	}
	else if(M == 2)
	{
		flag = 1.0;
	}

	pid->u_sum = Kp*pid->err + Kd*(pid->err - pid->err_last) + flag*Ki*pid->integral;
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

	tset = pid_init(35,0,0,0,0,0);

	while(count < 100)
	{
		real = pid_calc(tset);
		printf("%f\n",real);
		count++;
	}

	free(tset);
	return 0;
}

