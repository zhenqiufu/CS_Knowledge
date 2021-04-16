/*************************************************************************
    > File Name: exercise_1-19.c
    > Author: FU Zhenqiu
    > Mail: fuzhenqiu0810@gmail.com
    > Created Time: Mon 08 Mar 2021 10:26:19 PM PST
 ************************************************************************/

#include <stdio.h>
#define MAXLINE 100

int getline2(char line[], int maxline);
void myreverse(char to[], char from[], int len);
void reverse(char line[]);

int main() {
  int len;
  char line[MAXLINE];
  char reline[MAXLINE];

  while ((len = getline2(line, MAXLINE)) > 0) {
    printf("raw line:     %s\n", line);
    reverse(line);

    printf("reverse line: %s\n", line);
  }

  return 0;
}

int getline2(char line[], int limit)

{
  int i, c;

  for (i = 0; i < limit - 1 && (c = getchar()) != EOF && c != '\n'; i++) {
    line[i] = c;
  }

  if (c == '\n') {
    line[i] = c;
    i++;
  }

  line[i] = '\0';

  return i;
}

void reverse(char s[])

{
  int i, j, c;

  i = 0;

  /* This step can be avoided if the length of the string is given as an
   * argument.
   */

  for (j = 0; s[j] != '\n'; j++)
    ;

  if (j > 1)

    while (i < j) {
      c = s[i];
      s[i] = s[j];
      s[j] = c;

      i++;
      j--;
    }
}