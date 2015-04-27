/****************************************************************************************
*	
*	模糊免疫PID控制算法
*
*		免疫PID控制器是借鉴生物系统的免疫机理而设计出的一种非线性控制器。
*		免疫系统虽然十分复杂，但其抗御抗原的自适应能力却是十分明显的。
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
