#include <stdio.h>
#include <time.h>
#include <stdlib.h>

int main()
{
	srand(time(NULL));
	float random,D;
	
	int i=20;
	while(i--)
	{
		random = (float)(rand()%1000)/1000;
		D = 5.0*random;
		printf("D=%f\n",D);
	}
	
	return 0;
}