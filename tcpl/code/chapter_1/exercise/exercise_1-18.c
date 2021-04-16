/*************************************************************************
    > File Name: exercise_1-18.c
    > Author: FU Zhenqiu
    > Mail: fuzhenqiu0810@gmail.com
    > Created Time: Mon 08 Mar 2021 10:26:14 PM PST
 ************************************************************************/

#include <stdio.h>
#define MAXLINE 100

int getline2(char line[], int maxline);

int main() {
  int len;
  char line[MAXLINE];
  while ((len = getline2(line, MAXLINE)) > 0) {
    printf("%d\n", len);
    printf("Before deleta space:  BEGIN**%s", line);
    printf("**END*\n");
    //
    int i = 1;
    int new_len = len;
    while (line[len - i] == ' ' || line[len - i] == '\t') {
      i++;
      new_len--;
    }

    line[new_len] = '\0';
    printf("%d\n", new_len);
    printf("After deleta space:   BEGIN**%s", line);
    printf("**END*\n");
  }
  return 0;
}

int getline2(char line[], int maxline) {
  int i, c;
  for (i = 0; i < maxline - 1 && (c = getchar()) != EOF && c != '\n'; i++) {
    line[i] = c;
  }
  if (c == '\n') {
    line[i] = '\0';  //去除换行符，更清楚表示是否删掉了空格
    // i++;
  }
  line[i] = '\0';
  return i;
}