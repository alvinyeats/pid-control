/****************************************************************************************
*	
*	模糊自适应整定PID控制
*
*		自适应模糊PID控制器以误差e和误差ec作为输入，可以满足不同时刻的e和ec对PID参数自整
*	定的要求。利用模糊控制规则在线对PID参数进行修改，便构成了自适应模糊PID控制器。
*		PID参数模糊自整定是找出PID三个参数与e和ec之间的模糊关系，在运行中通过不断检测e和ec，
*	根据模糊控制原理来对3个参数进行在线修改，以满足不同e和ec时对控制参数的不同要求，而使被
*	控对象有良好的动、静态性能。
*		
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
	//pass
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
