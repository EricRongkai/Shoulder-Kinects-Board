/***************************************************

*文件描述:
*         循环队列数据结构
*Author:
*         LRK
*Time:
*					2018.6.28
*version:
*         v1.0
***************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <Queue.h>

//创建循环队列
SqQueue *initQueue()
{
	SqQueue *sq=(SqQueue *)malloc(sizeof(SqQueue));
	if(sq ==NULL)
	{
		while(1);
	}
	sq->rear=sq->front=0;
	return sq;
}

//判断循环队列是否为空

int isEmpty(SqQueue qu)
{
	return (qu.front ==qu.rear?1:0);
}

//元素进循环队列

int enQueue(SqQueue *qu,unsigned short x)
{

	if((qu->rear+1)%maxsize ==qu->front){
		return 0;
	}
	qu->rear=(qu->rear+1)%maxsize;
	qu->data[qu->rear]=x;
	return 1;
}

//元素出循环队列

int deQueue(SqQueue *qu,unsigned short *y)
{
	if(qu->rear ==qu->front)
	{
		return 0;
  }
	*y=qu->data[qu->front];
	qu->front=(qu->front+1)%maxsize;
	return 1;
}

//打印循环队列

int printQueue(SqQueue qu)
{
	if(qu.rear ==qu.front)
	{
		return 0;
	}
	while(qu.rear !=qu.front)
	{
		qu.front=(qu.front+1)%maxsize;
		printf("当前队列值=%d\n",qu.data[qu.front]);
	}
	return 1;
}



//创建链队列
LiQueue *initLiQueue()
{
	LiQueue *lq=(LiQueue *)malloc(sizeof(LiQueue));
	if(lq ==NULL)
	{
		while(1);
	}
	lq->front=lq->rear=NULL;
	return lq;
}

//判断链队列是否为空
int ISEmpty(LiQueue *lq)
{
	if(lq->front==NULL || lq->rear==NULL)
	{
		return 0;
	}
	else
	{
		return 1;
	}
}
//元素进链队列
void ENQueue(LiQueue *lq,unsigned short x)
{
	QNode *p;
	p=(QNode *)malloc(sizeof(QNode));
	p->data=x;
	p->next=NULL;
	if(lq->rear==NULL)
	{
		lq->front=lq->rear=p;//如果第一次插入则设置头指针和尾指针为p
  }
	else
	{
		lq->rear->next=p;//链队列的尾部插入p
		lq->rear=p;//设置链队列的尾指针指向p
	}
}

//元素出链队列
int DEQueue(LiQueue *lq,unsigned short *y)
{
	QNode *p=lq->front;
	if(lq->rear==NULL || lq->front==NULL)
	{
		return 0;
	}

	if(lq->front==lq->rear)
	{
		lq->front=lq->rear=NULL;
	}
	else
	{
		lq->front=lq->front->next;
	}
	*y=p->data;
	free(p);
	return 1;
}

//打印链队列
void PRINTQueue(LiQueue lq)
{
	if(lq.front==NULL || lq.rear==NULL)
	{
		return;
	}

	while(lq.front!=NULL)
	{
		printf("%d\n",lq.front->data);
		lq.front=lq.front->next;
	}
}
