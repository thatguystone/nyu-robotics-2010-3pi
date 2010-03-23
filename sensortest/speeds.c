#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#define pi 3.14159265

int main() {
	long homeX = 143;
	long dist = 544; 
	int asd = (int)(acos((double)homeX / dist) * ((double)180 / pi)); 

	printf("%d - %f\n", asd);
}
