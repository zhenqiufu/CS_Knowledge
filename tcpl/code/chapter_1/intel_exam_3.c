/*************************************************************************
    > File Name: intel_exam.c
    > Author: FU Zhenqiu
    > Mail: fuzhenqiu0810@gmail.com
    > Created Time: Tue 09 Mar 2021 05:04:25 PM CST
 ************************************************************************/

#include <stdio.h>

int main() {
  int c;
  while ((c = getchar()) != EOF) {
    if (c == ' ') {
      putchar('\t');
    } else {
      putchar(c);
    }
  }
}
