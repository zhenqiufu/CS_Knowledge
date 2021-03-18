/*************************************************************************
    > File Name: 3-1.c
    > Author: FU Zhenqiu
    > Mail: fuzhenqiu0810@gmail.com
    > Created Time: Thu 18 Mar 2021 04:15:58 PM CST
 ************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#define N 11
int V[N] = {1, 2, 3, 5, 7, 11, 13, 17, 19, 23, 31};

int main() {
  clock_t begin, end;
  double cost;
  //开始记录
  begin = clock();
  /*待测试程序段*/

  int x = 4;
  int ans = binsearch(x, V, N);
  printf("ans is %d\n", ans);
  //结束记录
  end = clock();
  cost = (double)(end - begin) / CLOCKS_PER_SEC;
  printf("constant CLOCKS_PER_SEC is: %ld, time cost is: %lf secs\n",
         CLOCKS_PER_SEC, cost);
}

int binsearch(int x, int v[], int n) {
  int low, high, mid;

  low = 0;
  high = n - 1;
  while (low <= high) {
    mid = (low + high) / 2;

    if (x < v[mid]) {
      high = mid - 1;
    } else if (x > v[mid]) {
      low = mid + 1;
    } else {
      return mid;
    }
  }
  return -1;
}

