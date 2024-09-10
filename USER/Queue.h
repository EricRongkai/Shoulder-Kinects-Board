#define maxsize 85
typedef struct SqQueue
{
	unsigned short data[maxsize];
	int front;//队首指针
	int rear;//队尾指针
}SqQueue;


//链队列结点结构
typedef struct QNode
{
	unsigned short data;
	struct QNode *next;
}QNode;

//链队列结构
typedef struct LiQueue
{
	QNode *front;
	QNode *rear;
}LiQueue;

SqQueue *initQueue(void);
int isEmpty(SqQueue qu);
int enQueue(SqQueue *qu,unsigned short x);
int deQueue(SqQueue *qu,unsigned short *y);
int printQueue(SqQueue qu);

LiQueue *initLiQueue(void);
int ISEmpty(LiQueue *lq);
void ENQueue(LiQueue *lq,unsigned short x);
int DEQueue(LiQueue *lq,unsigned short *y);
void PRINTQueue(LiQueue lq);

