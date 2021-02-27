/*************************************************************************
    > File Name: exercise_1-9.c
    > Author: FU Zhenqiu
    > Mail: fuzhenqiu0810@gmail.com
    > Created Time: Sat 27 Feb 2021 04:24:32 AM PST
 ************************************************************************/

#include<stdio.h>
int main(){
	int c;
	while((c=getchar()) != EOF){
		if(c == ' '){
			putchar(c);
			while((c=getchar()) != EOF && c == ' ')
				;
		}
		if(c != EOF){
			putchar(c);
		}
		
			
	}
}
