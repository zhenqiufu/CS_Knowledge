/*************************************************************************
    > File Name: exercise_1-4.c
    > Author: FU Zhenqiu
    > Mail: fuzhenqiu0810@gmail.com
    > Created Time: Sat 27 Feb 2021 03:44:58 AM PST
 ************************************************************************/

#include<stdio.h>
#define LOWER -20
#define UPPER 100
#define STEP  20

int main ()

{
	int	celcius, fahr;
	int	lower, upper, step;


	lower = LOWER;
	upper = UPPER;
	step = STEP;	

	printf("celcius  fahr\n");
	celcius = lower;
	while (celcius <= upper) {

		fahr = 9 * celcius / 5 + 32;
		printf("%3d %3d\n", celcius, fahr);
		celcius = celcius + step;

	}
}