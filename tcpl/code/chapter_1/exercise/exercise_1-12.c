/*************************************************************************
    > File Name: exercise_1-12.c
    > Author: FU Zhenqiu
    > Mail: fuzhenqiu0810@gmail.com
    > Created Time: Sat 27 Feb 2021 05:32:40 AM PST
 ************************************************************************/

#include <stdio.h>

#define IN 1
#define OUT 0

int main() {
  int c;
  int state = 0;
  while ((c = getchar()) != EOF) {
    if (c >= 'a' && c <= 'z' || c >= 'A' && c <= 'Z') {
      state = IN;
      putchar(c);
    } else {
      if (state == IN) {
        putchar('\n');
      }
      state = OUT;
    }
  }
}