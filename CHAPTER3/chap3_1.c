/****************************************************************************************
*	
*	专家PID控制
*
*		专家控制的实质是基于受控对象和控制规律的各种知识，并以只能的方式利用这些知识来
*	设计控制器。利用专家经验来设计PID参数便构成专家PID控制。
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
	//第一种情况：开环控制阶段
	//误差绝对值已经很大。不论误差变化趋势如何，都应考虑控制器的输出应按最大（或最小）
	//输出，以达到迅速调整误差，使误差绝对值以最大速度减小。
	if(abs(x_1) > 0.8)
		pid->u = 0.45;
	else if(abs(x_1) > 0.40)
		pid->u = 0.40;
	else if(abs(x_1) > 0.20)
		pid->u = 0.12;
	else if(abs(x_1) > 0.01)
		pid->u = 0.10;

	//第二种情况：
	//误差仍较大，考虑由控制器实施较强的控制作用，以达到扭转误差绝对值朝减小方向变化，
	//并迅速减小误差的绝对值。
	if((x_1*x_2 > 0)||(x_2 == 0))
	{
		if(abs(x_1) >= 0.05)
			pid->u = u_1 + 2*Kp*x_1;
		else
			pid->u = u_1 + 0.4*Kp*x_1;
	}

	//第三种情况：
	//误差的绝对值朝较小的方向变化，已经达到平衡状态。此时，可考虑采取保持控制器输出不变。
	if((x_1*x_2 < 0)&&(x_2*x2_1 > 0)||(x_1 == 0))
		pid->u = pid->u;

	//第四种情况：
	//误差的绝对值较大，考虑实施较强的控制作用
	if((x_1*x_2 < 0)&&(x_2*x2_1 < 0))
	{
		if(abs(x_1) >= 0.05)
			pid->u = u_1 + 2*Kp*pid->err_last;
		else
			pid->u = u_1 + 0.6*Kp*pid->err_last;
	}

	//第五种情况：
	//误差的绝对值很小，此时加入积分，减少稳态误差。
	if(abs(x_1) <= 0.001)
		pid->u = 0.5*x_1 + 0.010*x_3;

	x_1 = pid->err;
	x2_1 = x_2;
	x_2 = (pid->err - pid->err_last)/Ts;
	x_3 = x_3 + pid->err*Ts;

	pid->err_last = pid->err;

	return pid->u;
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
