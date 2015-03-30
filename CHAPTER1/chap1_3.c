/***************************************
*	The Increment PID Control Algorithm
*	增量式PID控制算法
*
*	input:just for Step Signal
*	目前只针对阶跃信号输入情况
***************************************/

#include <stdio.h>
#include <stdlib.h>

#define	 Kp		0.8
#define  Ki		0
#define	 Kd		0

#define	 Max	10.0
#define	 Min 	8.0


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