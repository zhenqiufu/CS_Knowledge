/*************************************************************************
    > File Name: exercise_1-14.c
    > Author: FU Zhenqiu
    > Mail: fuzhenqiu0810@gmail.com
    > Created Time: Mon 08 Mar 2021 10:25:48 PM PST
 ************************************************************************/

#include <stdio.h>

#define ALPHABETA_SIZE 26

int main() {
  int c, all, num[ALPHABETA_SIZE];
  for (int i = 0; i < ALPHABETA_SIZE; i++) {
    num[i] = 0;
  }

  while ((c = getchar()) != EOF) {
    if (c >= 'a' && c <= 'z') {
      num[c - 'a']++;
    } else if (c >= 'A' && c <= 'Z') {
      num[c - 'A']++;
    }
  }
  // output
  printf("***************************************\n");
  for (int i = 0; i < ALPHABETA_SIZE; i++) {
    printf("%c:  ", 'A' + i);
    for (int j = 0; j < num[i]; j++) {
      printf("=");
    }
    printf("\n");
  }

  return 0;
}