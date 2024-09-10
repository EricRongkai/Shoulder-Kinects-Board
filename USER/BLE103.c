#include "stm32f10x.h"
#include "BLE103.h"
#include "delay.h"
#include "string.h"
#include "includes.h"
#include "misc.h"
#include "stm32f10x_exti.h"

#define  BLE1_WAKE_L     GPIO_ResetBits(GPIOB, GPIO_Pin_3)
#define  BLE1_WAKE_H     GPIO_SetBits(GPIOB, GPIO_Pin_3)
#define  BLE2_WAKE_L     GPIO_ResetBits(GPIOB, GPIO_Pin_4)
#define  BLE2_WAKE_H     GPIO_SetBits(GPIOB, GPIO_Pin_4)

#define  BLE1_RESET_L    GPIO_ResetBits(GPIOA, GPIO_Pin_11)
#define  BLE1_RESET_H    GPIO_SetBits(GPIOA, GPIO_Pin_11)
#define  BLE2_RESET_L    GPIO_ResetBits(GPIOA, GPIO_Pin_12)
#define  BLE2_RESET_H    GPIO_SetBits(GPIOA, GPIO_Pin_12)

#define  BLE1_LINK    	 GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_8)
#define  BLE2_LINK    	 GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_9)

#define BYTE0(dwTemp)       (*(char *)(&dwTemp))
#define BYTE1(dwTemp)       (*((char *)(&dwTemp) + 1))
#define BYTE2(dwTemp)       (*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp)       (*((char *)(&dwTemp) + 3))

struct BLE103 BLE1;
struct BLE103 BLE2;
int CMC = 0;
int CMCOF = 500000;
char qq=0;
OS_SEM MY_SEM; 

void BLE103_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;
	OS_ERR err;
	//ʱ�ӳ�ʼ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
	//USART1��ʼ��
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	USART_InitStructure.USART_BaudRate = 57600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	
	USART_Init(USART1, &USART_InitStructure);
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
	USART_ITConfig(USART1, USART_IT_IDLE, ENABLE); //��������1���߿����ж�
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//ѡ����鷽ʽ2 
	/* ʹ�� USART1 �ж� */
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	USART_Cmd(USART1, ENABLE);
	USART_ClearFlag(USART1, USART_FLAG_TC);     /* �巢����Ǳ�־��Transmission Complete flag */
	
	//USART3��ʼ��
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	USART_Init(USART3, &USART_InitStructure); 
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
	USART_ITConfig(USART3, USART_IT_IDLE, ENABLE); //��������1���߿����ж�
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//ѡ����鷽ʽ2 
	/* ʹ�� USART3 �ж� */
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	USART_Cmd(USART3, ENABLE);
	USART_ClearFlag(USART3, USART_FLAG_TC);     /* �巢����Ǳ�־��Transmission Complete flag */
	
	//����IO��ʼ��
	//PB8 BLE1 LINK
	//PB9 BLE2 LINK
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource8); 
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource9); 
	EXTI_InitStructure.EXTI_Line =  EXTI_Line8|EXTI_Line9;
	EXTI_InitStructure.EXTI_Mode =  EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
	EXTI_InitStructure.EXTI_LineCmd = DISABLE;
	EXTI_Init(&EXTI_InitStructure);
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	//PB3 BLE1 WAKE
	//PB4 BLE2 WAKE
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3|GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	//PA11 BLE1 RESET
	//PA12 BLE2 RESET
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11|GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_SetBits(GPIOA, GPIO_Pin_11|GPIO_Pin_12);
	GPIO_SetBits(GPIOB, GPIO_Pin_3|GPIO_Pin_4);
}

void EnableLINK(void)
{
	EXTI_InitTypeDef EXTI_InitStructure;
	EXTI_InitStructure.EXTI_Line =  EXTI_Line8|EXTI_Line9;
	EXTI_InitStructure.EXTI_Mode =  EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);
}
/**************************************************************
* BLE1���� 
* sta=0����
* sta=1����
***************************************************************/
void BLE1_WAKE(u8 sta)
{
	if(sta==0)
	{
		BLE1_WAKE_H;
	}
	else if(sta==1)
	{
		BLE1_WAKE_L;
		Delay_ms(1500);
	}
}
/**************************************************************
* BLE2���� 
* sta=0����
* sta=1��
***************************************************************/
void BLE2_WAKE(u8 sta)
{
	if(sta==0)
	{
		BLE2_WAKE_H;
	}
	else if(sta==1)
	{
		BLE2_WAKE_L;
		Delay_ms(1500);
	}
}

/**************************************************************
* BLE1��λ 
***************************************************************/
void BLE1_RESET(void)
{
	OS_ERR err;
	BLE1_RESET_L;
	Delay_ms(30);
	BLE1_RESET_H;
	while(!BLE1.NewATReturn);
	BLE1.NewATReturn = 0;
	strcpy(BLE1.ModelHELLO,BLE1.DataReceive);
	memset(BLE1.DataReceive, 0, BLE1.DataAmount);  //��ջ���
	Delay_ms(1000);
}
/**************************************************************
* BLE2��λ 
***************************************************************/
void BLE2_RESET(void)
{
	BLE2_RESET_L;
	Delay_ms(30);
	BLE2_RESET_H;
	while(!BLE2.NewATReturn);
	BLE2.NewATReturn = 0;
	strcpy(BLE2.ModelHELLO,BLE2.DataReceive);
	memset(BLE2.DataReceive, 0, BLE2.DataAmount);  //��ջ���
	Delay_ms(1000);
}

/**************************************************************
* BLE1l����״̬
***************************************************************/
u8 Get_BLE1_LINK(void)
{
	u8 sta = BLE1_LINK;
	return sta;
}
/**************************************************************
* BLE2����״̬
***************************************************************/
u8 Get_BLE2_LINK(void)
{
	u8 sta = BLE2_LINK;
	return sta;
}
/**************************************************************
* BLE1����ATָ��
***************************************************************/
void BLE1_Send_AT_Command(char *Command)
{
	int i = 0;
	for(i=0;i<strlen(Command);i++)
	{
		USART_SendData(USART1, Command[i]);
		while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
	}
}

/**************************************************************
* BLE2����ATָ��
***************************************************************/
void BLE2_Send_AT_Command(char *Command)
{
	int i = 0;
	for(i=0;i<strlen(Command);i++)
	{
		USART_SendData(USART3, Command[i]);
		while (USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET);
	}
}

/**************************************************************
* BLE1�м̴���
***************************************************************/
void BLE2_Reload_Data(unsigned char *Data,char LEN)
{
	int i = 0;
	for(i=0;i<LEN;i++)
	{
		USART_SendData(USART3, Data[i]);
		while (USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET);
	}
}

/**************************************************************
* BLE1����ԭʼ����
***************************************************************/
void BLE1_Send_RawData(unsigned short TS, short *Adt, short *Gdt)
{
	unsigned char DataToSend[20];
	unsigned char verify = 0x00;
	int i = 0;
	DataToSend[0] = 0xAA;
	DataToSend[1] = 0xAA;
	DataToSend[2] = BYTE0(TS);
	DataToSend[3] = BYTE1(TS);
	DataToSend[4] = BYTE0(Adt[0]);
	DataToSend[5] = BYTE1(Adt[0]);
	DataToSend[6] = BYTE0(Adt[1]);
	DataToSend[7] = BYTE1(Adt[1]);
	DataToSend[8] = BYTE0(Adt[2]);
	DataToSend[9] = BYTE1(Adt[2]);
	DataToSend[10] = BYTE0(Gdt[0]);
	DataToSend[11] = BYTE1(Gdt[0]);
	DataToSend[12] = BYTE0(Gdt[1]);
	DataToSend[13] = BYTE1(Gdt[1]);
	DataToSend[14] = BYTE0(Gdt[2]);
	DataToSend[15] = BYTE1(Gdt[2]);
	for(i=0;i<16;i++)
	{
		verify |= DataToSend[i];
	}
	DataToSend[16] = verify;
	DataToSend[17] = 0xBB;
	for(i=0;i<18;i++)
	{
		USART_SendData(USART1, DataToSend[i]);
		while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
	}
}

/**************************************************************
* BLE2����ԭʼ����
***************************************************************/
void BLE2_Send_RawData(unsigned short TS, short *Adt, short *Gdt)
{	
	unsigned char DataToSend[20];
	unsigned char verify = 0x00;
	int i = 0;
	DataToSend[0] = 0xCC;
	DataToSend[1] = 0xCC;
	DataToSend[2] = BYTE0(TS);
	DataToSend[3] = BYTE1(TS);
	DataToSend[4] = BYTE0(Adt[0]);
	DataToSend[5] = BYTE1(Adt[0]);
	DataToSend[6] = BYTE0(Adt[1]);
	DataToSend[7] = BYTE1(Adt[1]);
	DataToSend[8] = BYTE0(Adt[2]);
	DataToSend[9] = BYTE1(Adt[2]);
	DataToSend[10] = BYTE0(Gdt[0]);
	DataToSend[11] = BYTE1(Gdt[0]);
	DataToSend[12] = BYTE0(Gdt[1]);
	DataToSend[13] = BYTE1(Gdt[1]);
	DataToSend[14] = BYTE0(Gdt[2]);
	DataToSend[15] = BYTE1(Gdt[2]);
	for(i=0;i<16;i++)
	{
		verify |= DataToSend[i];
	}
	DataToSend[16] = verify;
	DataToSend[17] = 0x00;
	DataToSend[18] = 0x00;
	DataToSend[19] = 0xBB;
	for(i=0;i<20;i++)
	{
		USART_SendData(USART3, DataToSend[i]);
		while (USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET);
	}
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/**************************************************************
* BLE1����ATָ��ģʽ
* 0��δ�ɹ�
* 1���ɹ�
***************************************************************/
u8 BLE1_ENCM()
{
	unsigned int TOC=0;       //��ʱ����
	char AT[] = "+++a";
	BLE1_Send_AT_Command(AT);
	while(!BLE1.NewATReturn)
	{
		TOC++;
		if(TOC>=300000)
		{
			return 0;
		}
	}
	char *ReturnValue = "a+ok\r\n";
	BLE1.NewATReturn = 0;
	if(!strcmp(BLE1.DataReceive,ReturnValue))
	{
		BLE1.CM_Sta = 1;
		memset(BLE1.DataReceive, 0, BLE1.DataAmount);  //��ջ���
		return 1;
	}
	else
	{
		BLE1.CM_Sta = 0;
		BLE1.NewATReturn = 0;
		memset(BLE1.DataReceive, 0, BLE1.DataAmount);  //��ջ���
		return 0;
	}
}
/**************************************************************
* BLE2����ATָ��ģʽ
* 0��δ�ɹ�
* 1���ɹ�
***************************************************************/
u8 BLE2_ENCM()
{
	unsigned int TOC=0;       //��ʱ����
	char AT[] = "+++a";
	BLE2_Send_AT_Command(AT);
	while(!BLE2.NewATReturn)
	{
		TOC++;
		if(TOC>=300000)
		{
			return 0;
		}
	}
	char *ReturnValue = "a+ok\r\n";
	BLE2.NewATReturn = 0;
	if(!strcmp(BLE2.DataReceive,ReturnValue))
	{
		BLE2.CM_Sta = 1;
		memset(BLE2.DataReceive, 0, BLE2.DataAmount);  //��ջ���
		return 1;
	}
	else
	{
		BLE2.CM_Sta = 0;
		BLE2.NewATReturn = 0;
		memset(BLE2.DataReceive, 0, BLE2.DataAmount);  //��ջ���
		return 0;
	}
}
/**************************************************************
* BLE1����͸��ģʽ
* 0��δ�ɹ�
* 1���ɹ�
***************************************************************/
u8 BLE1_ENTM()
{
	unsigned int TOC=0;       //��ʱ����
	char AT[] = "AT+ENTM\r\n";
	BLE1_Send_AT_Command(AT);
	while(!BLE1.NewATReturn)
	{
		TOC++;
		if(TOC>=300000)
		{
			return 0;
		}
	}
	char *ReturnValue = "\r\n+ENTM:OK\r\nOK\r\n";
	BLE1.NewATReturn = 0;
	if(!strcmp(BLE1.DataReceive,ReturnValue))
	{
		BLE1.CM_Sta = 0;
		memset(BLE1.DataReceive, 0, BLE1.DataAmount);  //��ջ���
		return 1;
	}
	else
	{
		BLE1.CM_Sta = 1;
		memset(BLE1.DataReceive, 0, BLE1.DataAmount);  //��ջ���
		return 0;
	}
}
/**************************************************************
* BLE2����͸��ģʽ
* 0��δ�ɹ�
* 1���ɹ�
***************************************************************/
u8 BLE2_ENTM()
{
	unsigned int TOC=0;       //��ʱ����
	char AT[] = "AT+ENTM\r\n";
	BLE2_Send_AT_Command(AT);
	while(!BLE2.NewATReturn)
	{
		TOC++;
		if(TOC>=300000)
		{
			return 0;
		}
	}
	char *ReturnValue = "\r\n+ENTM:OK\r\nOK\r\n";
	BLE2.NewATReturn = 0;
	if(!strcmp(BLE2.DataReceive,ReturnValue))
	{
		BLE2.CM_Sta = 0;
		memset(BLE2.DataReceive, 0, BLE2.DataAmount);  //��ջ���
		return 1;
	}
	else
	{
		BLE2.CM_Sta = 1;
		memset(BLE2.DataReceive, 0, BLE2.DataAmount);  //��ջ���
		return 0;
	}
}

/**************************************************************
* BLE1��ѯģ������
* 0��δ�ɹ�
* 1���ɹ�
* Return��"\r\n+NAME:name\r\nOK\r\n"
***************************************************************/
u8 BLE1_NAME()
{
	unsigned int TOC=0;       //��ʱ����
	char AT[] = "AT+NAME?\r\n";
	BLE1_Send_AT_Command(AT);
	while(!BLE1.NewATReturn)
	{
		TOC++;
		if(TOC>=300000)
		{
			return 0;
		}
	}
	BLE1.NewATReturn = 0;
	strcpy(BLE1.ModelNAME,BLE1.DataReceive);
	memset(BLE1.DataReceive, 0, BLE1.DataAmount);  //��ջ���
	return 1;
}

/**************************************************************
* BLE2��ѯģ������
* 0��δ�ɹ�
* 1���ɹ�
* Return��"\r\n+NAME:name\r\nOK\r\n"
***************************************************************/
u8 BLE2_NAME()
{
	unsigned int TOC=0;       //��ʱ����
	char AT[] = "AT+NAME?\r\n";
	BLE2_Send_AT_Command(AT);
	while(!BLE2.NewATReturn)
	{
		TOC++;
		if(TOC>=300000)
		{
			return 0;
		}
	}
	BLE2.NewATReturn = 0;
	strcpy(BLE2.ModelNAME,BLE2.DataReceive);
	memset(BLE2.DataReceive, 0, BLE2.DataAmount);  //��ջ���
	return 1;
}


/**************************************************************
* BLE1��ѯģ��ģʽ
* 0��δ�ɹ�
* 1���ɹ�
* Return��"\r\n+MODE:mode\r\nOK\r\n"
***************************************************************/
u8 BLE1_MODE()
{
	unsigned int TOC=0;       //��ʱ����
	char AT[] = "AT+MODE?\r\n";
	BLE1_Send_AT_Command(AT);
	while(!BLE1.NewATReturn)
	{
		TOC++;
		if(TOC>=300000)
		{
			return 0;
		}
	}
	BLE1.NewATReturn = 0;
	strcpy(BLE1.ModelMODE,BLE1.DataReceive);
	memset(BLE1.DataReceive, 0, BLE1.DataAmount);  //��ջ���
	return 1;
}
/**************************************************************
* BLE2��ѯģ��ģʽ
* 0��δ�ɹ�
* 1���ɹ�
* Return��"\r\n+MODE:mode\r\nOK\r\n"
***************************************************************/
u8 BLE2_MODE()
{
	unsigned int TOC=0;       //��ʱ����
	char AT[] = "AT+MODE?\r\n";
	BLE2_Send_AT_Command(AT);
	while(!BLE2.NewATReturn)
	{
		TOC++;
		if(TOC>=300000)
		{
			return 0;
		}
	}
	BLE2.NewATReturn = 0;
	strcpy(BLE2.ModelMODE,BLE2.DataReceive);
	memset(BLE2.DataReceive, 0, BLE2.DataAmount);  //��ջ���
	return 1;
}

/**************************************************************
* BLE1��ѯģ��MAC��ַ
* 0��δ�ɹ�
* 1���ɹ�
* Return��"\r\n+MODE:mode\r\nOK\r\n"
***************************************************************/
u8 BLE1_MAC()
{
	unsigned int TOC=0;       //��ʱ����
	char AT[] = "AT+MAC?\r\n";
	BLE1_Send_AT_Command(AT);
	while(!BLE1.NewATReturn)
	{
		TOC++;
		if(TOC>=300000)
		{
			return 0;
		}
	}
	BLE1.NewATReturn = 0;
	strcpy(BLE1.ModelMAC,BLE1.DataReceive);
	memset(BLE1.DataReceive, 0, BLE1.DataAmount);  //��ջ���
	return 1;
}
/**************************************************************
* BLE2��ѯģ��MAC��ַ
* 0��δ�ɹ�
* 1���ɹ�
* Return��"\r\n+MAC:mac\r\nOK\r\n"
***************************************************************/
u8 BLE2_MAC()
{
	unsigned int TOC=0;       //��ʱ����
	char AT[] = "AT+MAC?\r\n";
	BLE2_Send_AT_Command(AT);
	while(!BLE2.NewATReturn)
	{
		TOC++;
		if(TOC>=300000)
		{
			return 0;
		}
	}
	BLE2.NewATReturn = 0;
	strcpy(BLE2.ModelMAC,BLE2.DataReceive);
	memset(BLE2.DataReceive, 0, BLE2.DataAmount);  //��ջ���
	return 1;
}

/**************************************************************
* BLE1��ѯģ��汾��
* 0��δ�ɹ�
* 1���ɹ�
* Return��"\r\n+VER:�汾��\r\nOK\r\n"
***************************************************************/
u8 BLE1_CIVER()
{
	unsigned int TOC=0;       //��ʱ����
	char AT[] = "AT+CIVER?\r\n";
	BLE1_Send_AT_Command(AT);
	while(!BLE1.NewATReturn)
	{
		TOC++;
		if(TOC>=300000)
		{
			return 0;
		}
	}
	BLE1.NewATReturn = 0;
	strcpy(BLE1.ModelCIVER,BLE1.DataReceive);
	memset(BLE1.DataReceive, 0, BLE1.DataAmount);  //��ջ���
	return 1;
}
/**************************************************************
* BLE2��ѯģ��汾��
* 0��δ�ɹ�
* 1���ɹ�
* Return��"\r\n+VER:�汾��\r\nOK\r\n"
***************************************************************/
u8 BLE2_CIVER()
{
	unsigned int TOC=0;       //��ʱ����
	char AT[] = "AT+CIVER?\r\n";
	BLE2_Send_AT_Command(AT);
	while(!BLE2.NewATReturn)
	{
		TOC++;
		if(TOC>=300000)
		{
			return 0;
		}
	}
	BLE2.NewATReturn = 0;
	strcpy(BLE2.ModelCIVER,BLE2.DataReceive);
	memset(BLE2.DataReceive, 0, BLE2.DataAmount);  //��ջ���
	return 1;
}

/**************************************************************
* BLE1ʹ��/������������0��off�� 1��on��
* 0��δ�ɹ�
* 1���ɹ�
* Return��"\r\n+VER:�汾��\r\nOK\r\n"
***************************************************************/
u8 BLE1_MAXPUT(char EN)
{
	unsigned int TOC=0;       //��ʱ����
	char *AT;
	if(EN==0)
	{
		AT = "AT+MAXPUT=OFF\r\n";
	}
	else if(EN==1)
	{
		AT = "AT+MAXPUT=ON\r\n";
	}
	else{return 0;}
	BLE1_Send_AT_Command(AT);
	while(!BLE1.NewATReturn)
	{
		TOC++;
		if(TOC>=300000)
		{
			return 0;
		}
	}
	BLE1.NewATReturn = 0;
	strcpy(BLE1.ModelMAXPUT,BLE1.DataReceive);
	memset(BLE1.DataReceive, 0, BLE1.DataAmount);  //��ջ���
	return 1;
}
/**************************************************************
* BLE2ʹ��/������������0��off�� 1��on��
* 0��δ�ɹ�
* 1���ɹ�
* Return��"\r\n+VER:�汾��\r\nOK\r\n"
***************************************************************/
u8 BLE2_MAXPUT(char EN)
{
	unsigned int TOC=0;       //��ʱ����
	char *AT;
	if(EN==0)
	{
		AT = "AT+MAXPUT=OFF\r\n";
	}
	else if(EN==1)
	{
		AT = "AT+MAXPUT=ON\r\n";
	}
	else{return 0;}
	BLE2_Send_AT_Command(AT);
	while(!BLE2.NewATReturn)
	{
		TOC++;
		if(TOC>=300000)
		{
			return 0;
		}
	}
	BLE2.NewATReturn = 0;
	strcpy(BLE2.ModelMAXPUT,BLE2.DataReceive);
	memset(BLE2.DataReceive, 0, BLE2.DataAmount);  //��ջ���
	return 1;
}

/***********************************************************************
* BLE1���ù���ģʽ��M����ģʽ��S����ģʽ��B���㲥/iBeacon��F��Mesh����ģʽ��
* 0��δ�ɹ�
* 1���ɹ�
* Return��"\r\n+VER:�汾��\r\nOK\r\n"
************************************************************************/
u8 BLE1_SET_MODE(char Mode)
{
	unsigned int TOC=0;       //��ʱ����
	char *AT;
	if(Mode=='M')
	{
		AT = "AT+MODE=M\r\n";
	}
	else if(Mode=='S')
	{
		AT = "AT+MODE=S\r\n";
	}
	else if(Mode=='B')
	{
		AT = "AT+MODE=B\r\n";
	}
	else if(Mode=='F')
	{
		AT = "AT+MODE=F\r\n";
	}
	else{return 0;}
	BLE1_Send_AT_Command(AT);
	while(!BLE1.NewATReturn)
	{
		TOC++;
		if(TOC>=300000)
		{
			return 0;
		}
	}
	BLE1.NewATReturn = 0;
	CMC=0;
	strcpy(BLE1.ModelMODE,BLE1.DataReceive);
	memset(BLE1.DataReceive, 0, BLE1.DataAmount);  //��ջ���
	BLE1_Z();
	return 1;
}

/***********************************************************************
* BLE2���ù���ģʽ��M����ģʽ��S����ģʽ��B���㲥/iBeacon��F��Mesh����ģʽ��
* 0��δ�ɹ�
* 1���ɹ�
* Return��"\r\n+VER:�汾��\r\nOK\r\n"
************************************************************************/
u8 BLE2_SET_MODE(char Mode)
{
	unsigned int TOC=0;       //��ʱ����
	char *AT;
	if(Mode=='M')
	{
		AT = "AT+MODE=M\r\n";
	}
	else if(Mode=='S')
	{
		AT = "AT+MODE=S\r\n";
	}
	else if(Mode=='B')
	{
		AT = "AT+MODE=B\r\n";
	}
	else if(Mode=='F')
	{
		AT = "AT+MODE=F\r\n";
	}
	else{ return 0;}
	BLE2_Send_AT_Command(AT);
	while(!BLE2.NewATReturn)
	{
		TOC++;
		if(TOC>=300000)
		{
			return 0;
		}
	}
	BLE2.NewATReturn = 0;
	CMC=0;
	strcpy(BLE2.ModelMODE,BLE2.DataReceive);
	memset(BLE2.DataReceive, 0, BLE2.DataAmount);  //��ջ���
	BLE2_Z();
	return 1;
}

///**************************************************************
//* BLE1�����ӻ�
//* 0��δ�ɹ�
//* 1���ɹ�
//* Return��"\r\n+VER:�汾��\r\nOK\r\n"
//***************************************************************/
//u8 BLE1_SCAN()
//{
//	unsigned int TOC=0;       //��ʱ����
//	char AT[] = "AT+SCAN\r\n";
//	BLE1_Send_AT_Command(AT);
//	while(!BLE1.NewATReturn)
//	{
//		TOC++;
//		if(TOC>=300000)
//		{
//			return 0;
//		}
//	}
//	BLE1.NewATReturn = 0;
//	strcpy(BLE1.ModelSLIST,BLE1.DataReceive);
//	memset(BLE1.DataReceive, 0, BLE1.DataAmount);  //��ջ���
//	return 1;
//}
///**************************************************************
//* BLE2�����ӻ�
//* 0��δ�ɹ�
//* 1���ɹ�
//* Return��"\r\n+VER:�汾��\r\nOK\r\n"
//***************************************************************/
//u8 BLE2_SCAN()
//{
//	unsigned int TOC=0;       //��ʱ����
//	char AT[] = "AT+SCAN\r\n";
//	BLE2_Send_AT_Command(AT);
//	while(!BLE2.NewATReturn)
//	{
//		TOC++;
//		if(TOC>=300000)
//		{
//			return 0;
//		}
//	}
//	BLE2.NewATReturn = 0;
//	strcpy(BLE2.ModelSLIST,BLE2.DataReceive);
//	memset(BLE2.DataReceive, 0, BLE2.DataAmount);  //��ջ���
//	return 1;
//}

/**************************************************************
* BLE1����/��ѯ�豸�ϵ�Ĭ������ģ��� MAC ��ַ
* 0��δ�ɹ�
* 1���ɹ�
* Return��"\r\n+CONNADD:mac\r\nOK\r\n"
***************************************************************/
u8 BLE1_CONNADD()
{
	unsigned int TOC=0;       //��ʱ����
	char AT[] = "AT+CONNADD=9CA52512DD19\r\n";
	BLE1_Send_AT_Command(AT);
	while(!BLE1.NewATReturn)
	{
		TOC++;
		if(TOC>=300000)
		{
			return 0;
		}
	}
	BLE1.NewATReturn = 0;
	strcpy(BLE1.ModelCONNADD,BLE1.DataReceive);
	memset(BLE1.DataReceive, 0, BLE1.DataAmount);  //��ջ���
	BLE1_Z();
	return 1;
}
/**************************************************************
* BLE2�����豸�ϵ�Ĭ������ģ��� MAC ��ַ
* 0��δ�ɹ�
* 1���ɹ�
* Return��"\r\n+CONNADD:mac\r\nOK\r\n"
***************************************************************/
u8 BLE2_CONNADD()
{
	unsigned int TOC=0;       //��ʱ����
	char AT[] = "AT+CONNADD=9CA52512DD19\r\n";
	BLE2_Send_AT_Command(AT);
	while(!BLE2.NewATReturn)
	{
		TOC++;
		if(TOC>=300000)
		{
			return 0;
		}
	}
	BLE2.NewATReturn = 0;
	strcpy(BLE2.ModelCONNADD,BLE2.DataReceive);
	memset(BLE2.DataReceive, 0, BLE2.DataAmount);  //��ջ���
	BLE2_Z();
	return 1;
}

/**************************************************************
* BLE1����ģ������
* 0��δ�ɹ�
* 1���ɹ�
***************************************************************/
u8 BLE1_Z()
{
	unsigned int TOC=0;       //��ʱ����
	char AT[] = "AT+Z\r\n";
	BLE1_Send_AT_Command(AT);
	while(!BLE1.NewATReturn)
	{
		TOC++;
		if(TOC>=300000)
		{
			return 0;
		}
	}
	BLE1.NewATReturn = 0;
	char *ReturnValue = "\r\n+RST:OK\r\nOK\r\n";
	if(!strcmp(BLE1.DataReceive,ReturnValue))
	{
		memset(BLE1.DataReceive, 0, BLE1.DataAmount);  //��ջ���
		return 1;
	}
	else
	{
		memset(BLE1.DataReceive, 0, BLE1.DataAmount);  //��ջ���
		return 0;
	}
}
/**************************************************************
* BLE2����ģ������
* 0��δ�ɹ�
* 1���ɹ�
***************************************************************/
u8 BLE2_Z()
{
	unsigned int TOC=0;       //��ʱ����
	char AT[] = "AT+Z\r\n";
	BLE2_Send_AT_Command(AT);
	while(!BLE2.NewATReturn)
	{
		TOC++;
		if(TOC>=300000)
		{
			return 0;
		}
	}
	BLE2.NewATReturn = 0;
	char *ReturnValue = "\r\n+RST:OK\r\nOK\r\n";
	if(!strcmp(BLE2.DataReceive,ReturnValue))
	{
		memset(BLE2.DataReceive, 0, BLE2.DataAmount);  //��ջ���
		return 1;
	}
	else
	{
		memset(BLE2.DataReceive, 0, BLE2.DataAmount);  //��ջ���
		return 0;
	}
}

/**************************************************************
* BLE1��ѯģ������״̬
* 0��δ�ɹ�
* 1���ɹ�
***************************************************************/
u8 BLE1_LINK_STA()
{
	unsigned int TOC=0;       //��ʱ����
	CPU_SR_ALLOC();
	OS_CRITICAL_ENTER();	//�����ٽ���	
	char AT[] = "AT+LINK?\r\n";
	BLE1_Send_AT_Command(AT);
	OS_CRITICAL_EXIT();		//�˳��ٽ���	
	while(!BLE1.NewATReturn)
	{
		TOC++;
		if(TOC>=300000)
		{
			return 0;
		}
	}
	BLE1.NewATReturn = 0;
	if(strstr(BLE1.DataReceive,"OnLine")!=0)
	{
		memset(BLE1.DataReceive, 0, BLE1.DataAmount);  //��ջ���
		return 1;
	}
	else if(strstr(BLE1.DataReceive,"OffLine")!=0)
	{
		memset(BLE1.DataReceive, 0, BLE1.DataAmount);  //��ջ���
		return 0;
	}
}
/**************************************************************
* BLE2��ѯģ������״̬
* 0��δ����
* 1��������
***************************************************************/
u8 BLE2_LINK_STA()
{
	unsigned int TOC=0;       //��ʱ����
	char AT[] = "AT+LINK?\r\n";
	BLE2_Send_AT_Command(AT);
	while(!BLE2.NewATReturn)
	{
		TOC++;
		if(TOC>=300000)
		{
			return 0;
		}
	}
	BLE2.NewATReturn = 0;
	if(strstr(BLE2.DataReceive,"OnLine")!=0)
	{
		memset(BLE2.DataReceive, 0, BLE2.DataAmount);  //��ջ���
		return 1;
	}
	else if(strstr(BLE2.DataReceive,"OffLine")!=0)
	{
		memset(BLE2.DataReceive, 0, BLE2.DataAmount);  //��ջ���
		return 0;
	}
}
/**************************************************************
* BLE1ʹ�ܶ����Զ�����
* 0��δ�ɹ�
* 1���ɹ�
***************************************************************/
u8 BLE1_AUTOCONN()
{
	unsigned int TOC=0;       //��ʱ����
	char AT[] = "AT+AUTOCONN=ON\r\n";
	BLE1_Send_AT_Command(AT);
	while(!BLE1.NewATReturn)
	{
		TOC++;
		if(TOC>=300000)
		{
			return 0;
		}
	}
	BLE1.NewATReturn = 0;
	if(strstr(BLE1.DataReceive,"ON")!=0)
	{
		memset(BLE1.DataReceive, 0, BLE1.DataAmount);  //��ջ���
		return 1;
	}
	else if(strstr(BLE1.DataReceive,"OFF")!=0)
	{
		memset(BLE1.DataReceive, 0, BLE1.DataAmount);  //��ջ���
		return 0;
	}
}
/**************************************************************
* BLE2ʹ�ܶ����Զ�����
* 0��δ����
* 1��������
***************************************************************/
u8 BLE2_AUTOCONN()
{
	unsigned int TOC=0;       //��ʱ����
	char AT[] = "AT+AUTOCONN=ON\r\n";
	BLE2_Send_AT_Command(AT);
	while(!BLE2.NewATReturn)
	{
		TOC++;
		if(TOC>=300000)
		{
			return 0;
		}
	}
	BLE2.NewATReturn = 0;
	if(strstr(BLE2.DataReceive,"ON")!=0)
	{
		memset(BLE2.DataReceive, 0, BLE2.DataAmount);  //��ջ���
		return 1;
	}
	else if(strstr(BLE2.DataReceive,"OFF")!=0)
	{
		memset(BLE2.DataReceive, 0, BLE2.DataAmount);  //��ջ���
		return 0;
	}
}

/**************************************************************
* BLE1��������ģʽ
* 0��δ�ɹ�
* 1���ɹ�
***************************************************************/
u8 BLE1_HIBERNATE()
{
	unsigned int TOC=0;       //��ʱ����
	char AT[] = "AT+HIBERNATE\r\n";
	BLE1_Send_AT_Command(AT);
	while(!BLE1.NewATReturn)
	{
		TOC++;
		if(TOC>=300000)
		{
			return 0;
		}
	}
	BLE1.NewATReturn = 0;
	if(strstr(BLE1.DataReceive,"OK")!=0)
	{
		memset(BLE1.DataReceive, 0, BLE1.DataAmount);  //��ջ���
		return 1;
	}
	else
	{
		memset(BLE1.DataReceive, 0, BLE1.DataAmount);  //��ջ���
		return 0;
	}
}
/**************************************************************
* BLE2��������ģʽ
* 0��δ�ɹ�
* 1���ɹ�
***************************************************************/
u8 BLE2_HIBERNATE()
{
	unsigned int TOC=0;       //��ʱ����
	char AT[] = "AT+HIBERNATE\r\n";
	BLE2_Send_AT_Command(AT);
	while(!BLE2.NewATReturn)
	{
		TOC++;
		if(TOC>=300000)
		{
			return 0;
		}
	}
	BLE2.NewATReturn = 0;
	if(strstr(BLE2.DataReceive,"OK")!=0)
	{
		memset(BLE2.DataReceive, 0, BLE2.DataAmount);  //��ջ���
		return 1;
	}
	else
	{
		memset(BLE2.DataReceive, 0, BLE2.DataAmount);  //��ջ���
		return 0;
	}
}
