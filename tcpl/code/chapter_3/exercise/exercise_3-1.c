/*************************************************************************
    > File Name: exercise_3-1.c
    > Author: FU Zhenqiu
    > Mail: fuzhenqiu0810@gmail.com
    > Created Time: Thu 18 Mar 2021 04:16:20 PM CST
 ************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <time.h>

int main() {
  clock_t begin, end;
  double cost;
  //开始记录
  begin = clock();
  /*待测试程序段*/
  printf("hello world!\n");
  //结束记录
  end = clock();
  cost = (double)(end - begin) / CLOCKS_PER_SEC;
  printf("constant CLOCKS_PER_SEC is: %ld, time cost is: %lf secs\n",
         CLOCKS_PER_SEC, cost);
}

