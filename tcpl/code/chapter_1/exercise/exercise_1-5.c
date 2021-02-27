/*************************************************************************
    > File Name: exercise_1-5.c
    > Author: FU Zhenqiu
    > Mail: fuzhenqiu0810@gmail.com
    > Created Time: Sat 27 Feb 2021 03:51:35 AM PST
 ************************************************************************/

#include<stdio.h>
int main(){
	int fahr;
	for(fahr =300; fahr >= 0; fahr = fahr -20){
		printf("%3d %6.1f\n", fahr, (5.0 / 9.0)*(fahr - 32));
	}
}
