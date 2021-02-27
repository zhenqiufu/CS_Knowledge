/*************************************************************************
    > File Name: exercise_1-8.c
    > Author: FU Zhenqiu
    > Mail: fuzhenqiu0810@gmail.com
    > Created Time: Sat 27 Feb 2021 04:16:40 AM PST
 ************************************************************************/

#include<stdio.h>
int main(){

	int num_space, num_tab, num_n;
	num_space = num_tab = num_n = 0;
	int c;

	while((c=getchar()) != EOF){
		if(c == ' ')
			num_space++;
		if(c == '\t')
			num_tab++;
		if(c == '\n')
			num_n++;
	}

	printf("Space = %d, tab = %d, newline = %d.\n", num_space, num_tab, num_n);
}
