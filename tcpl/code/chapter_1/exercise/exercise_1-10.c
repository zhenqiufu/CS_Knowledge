/*************************************************************************
    > File Name: exercise_1-10.c
    > Author: FU Zhenqiu
    > Mail: fuzhenqiu0810@gmail.com
    > Created Time: Sat 27 Feb 2021 04:36:20 AM PST
 ************************************************************************/

#include<stdio.h>
int main(){
	int c;
	while((c=getchar()) != EOF){
		if(c == '\t'){
			printf("\\t");
		}else if(c == '\b'){
			printf("\\b");
		}else if(c == '\\'){
			printf("\\\\");
		}else{
			putchar(c);
		}
	}

}
