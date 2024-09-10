#define maxsize 85
typedef struct SqQueue
{
	unsigned short data[maxsize];
	int front;//����ָ��
	int rear;//��βָ��
}SqQueue;


//�����н��ṹ
typedef struct QNode
{
	unsigned short data;
	struct QNode *next;
}QNode;

//�����нṹ
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

