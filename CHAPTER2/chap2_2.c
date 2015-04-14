/****************************************************************************************
*	
*	纯滞后系统的大林控制算法
*
*		早在1968年，美国IBM公司的大林（Dahlin）就提出了一种不同于常规PID控制规律
*	的新型算法，即大林算法。该算法的最大特点是将期望的闭环响应设计成一阶惯性加纯
*	延迟，然后反过来得到满足这种闭环响应的控制器。
*
*	注：因算法本身的复杂度问题，该程序未整定完全，具体参数和计算方式请根据实际情况定义！
****************************************************************************************/

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
	//wait
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
