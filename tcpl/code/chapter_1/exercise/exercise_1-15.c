/*************************************************************************
    > File Name: exercise_1-15.c
    > Author: FU Zhenqiu
    > Mail: fuzhenqiu0810@gmail.com
    > Created Time: Mon 08 Mar 2021 10:25:55 PM PST
 ************************************************************************/

#include <stdio.h>

#define UPPER 300
#define LOWER 0
#define STEP 10

float celcius(float);

int main() {
  float fahr = LOWER;

  while (fahr <= UPPER) {
    printf("%3.0f  %6.1f\n", fahr, celcius(fahr));
    fahr += STEP;
  }
  return 0;
}

float celcius(float fahr) { return (5.0 / 9.0) * (fahr - 32); }
