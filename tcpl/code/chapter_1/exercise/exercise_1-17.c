/*************************************************************************
    > File Name: exercise_1-17.c
    > Author: FU Zhenqiu
    > Mail: fuzhenqiu0810@gmail.com
    > Created Time: Mon 08 Mar 2021 10:26:07 PM PST
 ************************************************************************/

#include <stdio.h>
#define MAXLINE 100
#define OUTSTANDARD 8

int getline2(char line[], int maxline);

int main() {
  int len;
  char line[MAXLINE];
  while ((len = getline2(line, MAXLINE)) > 0) {
    if (len > OUTSTANDARD) {
      printf("%s\n", line);
    }
  }

  return 0;
}

int getline2(char line[], int maxline) {
  int c, i;
  for (i = 0; i < maxline - 1 && (c = getchar()) != EOF && c != '\n'; i++) {
    line[i] = c;
  }
  if (c == '\n') {
    line[i] = c;
    i++;
  }
  line[i] = '\0';
  return i;
}