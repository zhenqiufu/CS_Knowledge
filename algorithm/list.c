/*************************************************************************
    > File Name: list.c
    > Author: FU Zhenqiu
    > Mail: fuzhenqiu0810@gmail.com
    > Created Time: 2021年05月06日 星期四 19时16分35秒
 ************************************************************************/

#include <stdio.h>

typedef struct Node {
  int x;
  struct Node* pNext;
} Node, *PNode;

// 创建链表
PNode creat_list();

// 遍历链表
void traverse_list(PNode head);

// 判断是否为空
bool is_empty(PNode head);

// 返回长度
int long_list(PNode head);

// 删除指定位置元素

// 指定位置插入元素

// 排序链表

int main(){
	PNode head=NULL;
	head = creat_list();
	return 0;
}

PNode creat_list(){
	
}
