/*************************************************************************
    > File Name: exercise_1-19.c
    > Author: FU Zhenqiu
    > Mail: fuzhenqiu0810@gmail.com
    > Created Time: Mon 08 Mar 2021 10:26:19 PM PST
 ************************************************************************/

#include <stdio.h>
#define MAXLINE 100

int mygetline(char line[], int maxline);
void copy(char to[], char from[]);

int main() {
  int len;
  int max;
  char line[MAXLINE];
  char longest[MAXLINE];

  max = 0;
  while ((len = mygetline(line, MAXLINE)) > 0) {
    if (len > max) {
      max = len;
      copy(longest, line);
    }
  }

  if (max > 0) {
    printf("longest string line is: \n");
    printf("%s\n", longest);
  }
  return 0;
}

int mygetline(char s[], int lim) {
  int i, c;
  for (i = 0; i < lim && (c = getchar()) != EOF && c != '\n'; ++i) {
    s[i] = c;
  }
  if (c == '\n') {
    s[i] = c;
    ++i;
  }
  s[i] = '\0';
  return i;
}

void copy(char to[], char from[]) {
  int i;
  i = 0;

  while ((to[i] = from[i]) != '\0') {
    i++;
  }
}
