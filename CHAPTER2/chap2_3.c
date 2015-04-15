/****************************************************************************************
*	
*	纯滞后系统的Smith控制算法
*
*		在工业过程控制中，许多被控对象具有纯滞后的性质。Smith（史密斯）提出了一种
*	纯滞后补偿模型，其原理为：与PID控制器并接一补偿环节，该补偿环节称为Smith预估器。
*
*	注：因算法本身的复杂度问题，该程序未整定完全，具体参数和计算方式请根据实际情况定义！
****************************************************************************************/

#include <stdio.h>
#include <stdlib.h>

#define	 Kp		1.0
#define  Ki		0.50
#define	 Kd		0.10

#define  Ts 	20


//构造结构体
struct pid_data
{
	float rin;			
	float yout;	
	float err;			
	float err_last;		
	float integral;		
	float u;
};

//定义结构体类型
typedef struct pid_data		pid_t;

//机构体初始化
struct pid_data* pid_init(float rin, float yout,
float err, float err_last, float u)
{
	struct pid_data* tset = malloc(sizeof(struct pid_data));

	tset->rin 		= rin;
	tset->yout		= yout;
	tset->err 		= err;		
	tset->err_last 	= err_last;
	tset->u			= u;

	return tset;
}

//The Increment PID Control Algorithm
float pid_calc(pid_t* pid)
{
	//wait
	float err_1,err_2,err_3;
	pid->err = pid->rin - pid->yout;

	int M=1;
	if M==1
	{
		wkp = wkp + xiteP*u*x(1);		//P	
		wkd = whi + xiteI*u*x(2);		//I
		wkd = whd + xiteD*u*x(3);		//D
	}
	
	else if (M == 2)
	{
		pid->integral += pid->err;
		pid->u = Kp*pid->err + Kd*(pid->err - pid->err_last)/ts + Ki*pid->integral;
	}

	pid->err_last = pid->err;
	pid->yout = pid->u;

	return pid->yout;
}

int main()
{
	printf("System test begin \n");

	pid_t* tset;
	int count = 0;
	float real = 0;

	tset = pid_init(23,0,0,0,0,0,0,0);

	while(count < 100)
	{
		real = pid_calc(tset);
		printf("%f\n",real);
		count++;
	}

	free(tset);
	return 0;
}
