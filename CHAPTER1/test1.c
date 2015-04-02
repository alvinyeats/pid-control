#include <stdio.h>

#define a 2
#define b 3
#define c (float)(a*b)/5

int main()
{
	if(a==4)
		printf("c=%f\n",c);
	else if(a==3)
	{}	
	else if(b==3)
		printf("c=%f\n",c);
	return 0;
}