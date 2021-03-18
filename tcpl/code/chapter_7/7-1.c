/*************************************************************************
    > File Name: 7-1.c
    > Author: FU Zhenqiu
    > Mail: fuzhenqiu0810@gmail.com
    > Created Time: Fri 26 Feb 2021 02:11:13 AM PST
 ************************************************************************/

#include <ctype.h>
#include <stdio.h>
int main(int argc, char *argv[]) {
  int c;
  while ((c = getchar()) != EOF) {
    putchar(tolower(c));
  }
  // printf("%d\n", argc);
  // for (int i = 0; i < argc; i++) {
  //   printf("argument[%d]is: %s\n", i, argv[i]);
  // }
  return 0;
}