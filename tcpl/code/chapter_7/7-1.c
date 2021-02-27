/*************************************************************************
    > File Name: 7-1.c
    > Author: FU Zhenqiu
    > Mail: fuzhenqiu0810@gmail.com
    > Created Time: Fri 26 Feb 2021 02:11:13 AM PST
 ************************************************************************/

#include<stdio.h>
#include<ctype.h>
int main ()
{
	int c;
	while((c=getchar())!=EOF){
		putchar(tolower(c));
	}
	return 0;
}