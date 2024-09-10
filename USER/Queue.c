/***************************************************

*�ļ�����:
*         ѭ���������ݽṹ
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

//����ѭ������
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

//�ж�ѭ�������Ƿ�Ϊ��

int isEmpty(SqQueue qu)
{
	return (qu.front ==qu.rear?1:0);
}

//Ԫ�ؽ�ѭ������

int enQueue(SqQueue *qu,unsigned short x)
{

	if((qu->rear+1)%maxsize ==qu->front){
		return 0;
	}
	qu->rear=(qu->rear+1)%maxsize;
	qu->data[qu->rear]=x;
	return 1;
}

//Ԫ�س�ѭ������

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

//��ӡѭ������

int printQueue(SqQueue qu)
{
	if(qu.rear ==qu.front)
	{
		return 0;
	}
	while(qu.rear !=qu.front)
	{
		qu.front=(qu.front+1)%maxsize;
		printf("��ǰ����ֵ=%d\n",qu.data[qu.front]);
	}
	return 1;
}



//����������
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

//�ж��������Ƿ�Ϊ��
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
//Ԫ�ؽ�������
void ENQueue(LiQueue *lq,unsigned short x)
{
	QNode *p;
	p=(QNode *)malloc(sizeof(QNode));
	p->data=x;
	p->next=NULL;
	if(lq->rear==NULL)
	{
		lq->front=lq->rear=p;//�����һ�β���������ͷָ���βָ��Ϊp
  }
	else
	{
		lq->rear->next=p;//�����е�β������p
		lq->rear=p;//���������е�βָ��ָ��p
	}
}

//Ԫ�س�������
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

//��ӡ������
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
