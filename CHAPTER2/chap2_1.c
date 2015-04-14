/*******************************************************************************************
*	The Cascade PID control
*	串级PID控制
*
*		主外副内，在一般的串级PID控制中，外环被称为主回路，内环被称为副回路。
*		主调节器的输出控制量u1作为副回路的给定量R2(s)。
*		串级控制系统的计算顺序是先主回路（PID1），后副回路（PID2）。控制方式有两种：一
*	种是异步采样控制，即主回路的采样控制周期T1是副回路采样控制周期T2的整数倍。这是因为
*	一般串级控制系统中主控对象的响应速度慢、副控对象的响应速度快的缘故。另一种是同步采
*	样控制，即主、副回路的采样周期相同。这时，应根据副回路选择采样周期，因为副回路的受
*	控对象的响应速度较快。
*
*	注：因算法本身的复杂度问题，该程序未整定完全，具体参数和计算方式请根据实际情况定义！
*******************************************************************************************/

#include <stdio.h>
#include <stdlib.h>

#define	 Kp		1.20
#define  Ki		0.02
#define	 Kd		0.00

//构造结构体
struct pid_data
{
	float rin;			//主回路输入
	float yout1;		//主回路输出
	float yout2;		//副回路输出
	float err;			//主回路反馈误差
	float err_last;		//上一次的主回路反馈误差
	float integral;		//主回路反馈误差积分
	float u1;			//主回路控制量
	float u2;			//副回路控制量
	float err2;			//副回路反馈误差
};

//定义结构体类型
typedef struct pid_data		pid_t;

//机构体初始化
struct pid_data* pid_init(float rin, float yout1,float yout2,
float err, float err_last, float u1, float u2, float err2)
{
	struct pid_data* tset = malloc(sizeof(struct pid_data));

	tset->rin 		= rin;
	tset->yout1		= yout1;
	tset->yout2 	= yout2; 				
	tset->err 		= err;		
	tset->err_last 	= err_last;
	tset->u1		= u1;
	tset->u2		= u2;
	tset->err2		= err2;

	return tset;
}

//The Increment PID Control Algorithm
float pid_calc(pid_t* pid)
{
	//副回路反馈误差权重
	
	float alpha  = 0.85,
		  beta   = 0.15,
		  mju	 = 0.66,
		  nju	 = 0.88;
	
	pid->err 	   = pid->rin - pid->yout1;
	pid->integral += pid->err;
	pid->u1 	   = Kp*pid->err + Ki*pid->integral;

	//具体计算方式根据实际情况而定,切勿生搬硬套！
	pid->err2 = pid->u1 - pid->yout2;
	pid->u2   = 0.05*pid->err2;
	
	pid->yout2 = mju*pid->yout2 + nju*pid->u2;
	pid->yout1 = alpha*pid->yout1 + beta*pid->yout2;
	pid->err_last = pid->err;
	
	return pid->yout1;
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
