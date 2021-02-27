/*************************************************************************
    > File Name: exercise_1-6.c
    > Author: FU Zhenqiu
    > Mail: fuzhenqiu0810@gmail.com
    > Created Time: Sat 27 Feb 2021 04:01:34 AM PST
 ************************************************************************/

#include<stdio.h>
int main(){
	int c;
	c = getchar() != EOF;
	printf("c = getchar() != is %d\n", c);
}
/*
note:
Ctrl-D is EOF input for Linux system 
Put Ctrl-D can get 0
other input and then Enter can get 1
*/
