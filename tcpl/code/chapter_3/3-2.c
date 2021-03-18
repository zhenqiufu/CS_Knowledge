/*************************************************************************
    > File Name: 3-2.c
    > Author: FU Zhenqiu
    > Mail: fuzhenqiu0810@gmail.com
    > Created Time: Thu 18 Mar 2021 04:50:58 PM CST
 ************************************************************************/

#include <stdio.h>

int main() {
  int i, c, nwhite, nother, ndight[10];
  nwhite = nother = 0;

  for (int i = 0; i < 10; ++i) {
    ndight[i] = 0;
  }

  while ((c = getchar()) != EOF) {
    switch (c) {
      case '0':
      case '1':
      case '2':
      case '3':
      case '4':
      case '5':
      case '6':
      case '7':
      case '8':
      case '9':
        ndight[c - '0']++;
        break;
      case ' ':
      case '\n':
      case '\t':
        nwhite++;
        break;
      default:
        nother++;
        break;
    }
  }
  printf("digits = ");
  for (int i = 0; i < 10; i++) {
    printf("%d", ndight[i]);
  }
  printf(", white space = %d, other = %d\n", nwhite, nother);

  return 0;
}