/*************************************************************************
    > File Name: exercise_1-3.c
    > Author: FU Zhenqiu
    > Mail: fuzhenqiu0810@gmail.com
    > Created Time: 2021年02月03日 星期三 10时26分02秒
 ************************************************************************/

#include <stdio.h>
int main() {
  float fahr, celcius;
  int lower, upper, step;

  lower = 0;
  upper = 300;
  step = 20;

  printf("fahr celcius\n");
  fahr = lower;
  while (fahr <= upper) {
    celcius = (5.0 / 9.0) * (fahr - 32);
    printf("%3.0f  %6.1f\n", fahr, celcius);
    fahr = fahr + step;
  }
  return 0;
}
