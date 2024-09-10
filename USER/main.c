#include "stm32f10x.h"
#include "stm32f10x_exti.h"
#include "stdlib.h"
#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "LED.h"
#include "includes.h"
#include "BLE103.h"
#include "mpu9250.h"
#include "Task.h"
#include "stm32_iic.h"
#include "KEY.h"
#include "Power.h"
#include "ads.h"
#include "ads_hal.h"

//UCOSIII���������ȼ��û�������ʹ��
//����Щ���ȼ��������UCOSIII��5��ϵͳ�ڲ�����
//���ȼ�0���жϷ������������� OS_IntQTask()
//���ȼ�1��ʱ�ӽ������� OS_TickTask()
//���ȼ�2����ʱ���� OS_TmrTask()
//���ȼ�OS_CFG_PRIO_MAX-2��ͳ������ OS_StatTask()
//���ȼ�OS_CFG_PRIO_MAX-1���������� OS_IdleTask()

//�������ȼ�
#define START_TASK_PRIO		3
//�����ջ��С	
#define START_STK_SIZE 		128
//������ƿ�
OS_TCB StartTaskTCB;
//�����ջ	
CPU_STK START_TASK_STK[START_STK_SIZE];
//������
void start_task(void *p_arg);
//ADS�������������ݵ�ַ
uint8_t *ADS_DATA = NULL;

//ADS���������ݽ��ջص�����
void ads_datahandler(float *data, uint8_t *buffer, uint8_t type, uint8_t device)
{
#ifdef DEBUG
  switch(type)
	{
		case 0x00: 
			printf("Device:%d|Type:%d|Value:%f\n",device,type, data[0]);break;
		case 0x03: 
			printf("Device:%d|Type:%d|Value:%f\n",device,type, data[1]);break;
		case 0x04: 
			printf("Device:%d|Type:%d|Axis1:%f Axis2:%f\n",device,type,data[0],data[1]);break;
	}
#elif RELEASE
		switch(device)
		{
		case 0x01: 
			if(buffer[0]==ADS_SAMPLE)
			{
				memcpy(ADS_DATA+5,buffer,3);
			}
			else if(buffer[0]==ADS_STRETCH_SAMPLE)
			{
				memcpy(ADS_DATA+8,buffer,3);
			}break;
		case 0x02: 
			if(buffer[0]==ADS_SAMPLE)
			{
				memcpy(ADS_DATA+11,buffer,3);
			}
			else if(buffer[0]==ADS_STRETCH_SAMPLE)
			{
				memcpy(ADS_DATA+14,buffer,3);
			}break;
		case 0x03: 
			memcpy(ADS_DATA+17,buffer,5);break;
		case 0x04: 
			memcpy(ADS_DATA+22,buffer,5);break;
		case 0x05: 
			if(buffer[0]==ADS_SAMPLE)
			{
				memcpy(ADS_DATA+27,buffer,3);
			}
			else if(buffer[0]==ADS_STRETCH_SAMPLE)
			{
				memcpy(ADS_DATA+30,buffer,3);
			}break;
		case 0x06: 
			if(buffer[0]==ADS_SAMPLE)
			{
				memcpy(ADS_DATA+33,buffer,3);
			}
			else if(buffer[0]==ADS_STRETCH_SAMPLE)
			{
				memcpy(ADS_DATA+36,buffer,3);
			}break;
	 }
#endif
}

#define  SCL_H         GPIOB->BSRR = GPIO_Pin_10  
#define  SCL_L         GPIOB->BRR  = GPIO_Pin_10  
#define  SDA_H         GPIOB->BSRR = GPIO_Pin_11  
#define  SDA_L         GPIOB->BRR  = GPIO_Pin_11 
//������
int main(void)
{
	/***********************************�ϵ�ϵͳ��ʼ��***************************************/
	__set_PRIMASK(1);  															//�����ж�
	OS_ERR err;																			//����OS�������
	RCC_ClocksTypeDef RCC_clock;										//����ϵͳʱ�Ӽ�����
  CPU_SR_ALLOC();																	//OS��ʼ���Ĵ���״̬�������ʼΪ0
	LED_Config(); 																	//LED��ʼ��
	Yled_on;
	KEY_Init();																			//������ʼ��
	delay_init();  																	//ʱ�ӳ�ʼ��	//LED��ʼ��
	USART1_Config();																//���ڳ�ʼ��
	i2cInit();      							  								//MPU9250IIC���ߵĳ�ʼ��
	RCC_GetClocksFreq(&RCC_clock);  								//��ȡ��ǰ��ʱ��Ƶ��
#ifdef DEBUG
	printf("System Initialization Finish !!\n");
#endif
	/***********************************����ģ���ʼ��***************************************/
	if(Mpu9250_Work_Mode_Init()) while(1);					//MPU9250��ʼ��
	
	ads_init_t init_val;	
	int ads_sta = 0;
	ADS_DATA = (uint8_t*)malloc(40);
	memset(ADS_DATA,0,60);
	
	ads_hal_select_device(0x01);													
	init_val.addr = 0x11;														//����ads������I2C��ַ
	init_val.datardy_pin = GPIO_Pin_4;							//����ads�����������ж�����
	init_val.reset_pin = GPIO_Pin_5;								//����ads��������������
	init_val.sps = ADS_50_HZ;											  //����ads������������
	init_val.ads_sample_callback = &ads_datahandler;//����ads�������ص�����
	ads_sta = ads_init(&init_val);
#ifdef DEBUG
	if(ads_sta == ADS_OK){
		printf("Flex_1 Initialization Finish\n");
	}else{
	  printf("Flex_1 Initialization Error\n");
	}
#elif RELEASE
	if(ads_sta == ADS_OK){Yled_off;Delay_ms(100);Yled_on;}else{Yled_on;}
#endif
	ads_hal_select_device(0x02);													
	init_val.addr = 0x12;														//����ads������I2C��ַ
	init_val.datardy_pin = GPIO_Pin_6;							//����ads�����������ж�����
	init_val.reset_pin = GPIO_Pin_7;								//����ads��������������
	init_val.sps = ADS_50_HZ;											  //����ads������������
	init_val.ads_sample_callback = &ads_datahandler;//����ads�������ص�����
	ads_sta = ads_init(&init_val);
#ifdef DEBUG
	if(ads_sta == ADS_OK){
		printf("Flex_2 Initialization Finish\n");
	}else{
	  printf("Flex_2 Initialization Error\n");
	}
#elif RELEASE
	if(ads_sta == ADS_OK){Yled_off;Delay_ms(100);Yled_on;}else{ Yled_on;}
#endif

	ads_hal_select_device(0x03);													
	init_val.addr = 0x13;														//����ads������I2C��ַ
	init_val.datardy_pin = GPIO_Pin_12;							//����ads�����������ж�����
	init_val.reset_pin = GPIO_Pin_13;								//����ads��������������
	init_val.sps = ADS_50_HZ;											  //����ads������������
	init_val.ads_sample_callback = &ads_datahandler;//����ads�������ص�����
	ads_sta = ads_init(&init_val);
#ifdef DEBUG
	if(ads_sta == ADS_OK){
		printf("Flex_3 Initialization Finish\n");
	}else{
	  printf("Flex_3 Initialization Error\n");
	}
#elif RELEASE
	if(ads_sta == ADS_OK){Yled_off;Delay_ms(100);Yled_on;}else{Yled_on;}
#endif
	
	ads_hal_select_device(0x04);													
	init_val.addr = 0x14;														//����ads������I2C��ַ
	init_val.datardy_pin = GPIO_Pin_15;							//����ads�����������ж�����
	init_val.reset_pin = GPIO_Pin_3;								//����ads��������������
	init_val.sps = ADS_50_HZ;											  //����ads������������
	init_val.ads_sample_callback = &ads_datahandler;//����ads�������ص�����
	ads_sta = ads_init(&init_val);
#ifdef DEBUG
	if(ads_sta == ADS_OK){
		printf("Flex_4 Initialization Finish\n");
	}else{
	  printf("Flex_4 Initialization Error\n");
	}
#elif RELEASE
	if(ads_sta == ADS_OK){Yled_off;Delay_ms(100);Yled_on;}else{Yled_on;}
#endif


	ads_hal_select_device(0x05);													
	init_val.addr = 0x15;														//����ads������I2C��ַ
	init_val.datardy_pin = GPIO_Pin_5;							//����ads�����������ж�����
	init_val.reset_pin = GPIO_Pin_4;								//����ads��������������
	init_val.sps = ADS_50_HZ;											  //����ads������������
	init_val.ads_sample_callback = &ads_datahandler;//����ads�������ص�����
	ads_sta = ads_init(&init_val);
#ifdef DEBUG
	if(ads_sta == ADS_OK){
		printf("Flex_5 Initialization Finish\n");
	}else{
	  printf("Flex_5 Initialization Error\n");
	}
#elif RELEASE
	if(ads_sta == ADS_OK){Yled_off;Delay_ms(100);Yled_on;}else{Yled_on;}
#endif
	
	ads_hal_select_device(0x06);													
	init_val.addr = 0x16;														//����ads������I2C��ַ
	init_val.datardy_pin = GPIO_Pin_8;							//����ads�����������ж�����
	init_val.reset_pin = GPIO_Pin_9;								//����ads��������������
	init_val.sps = ADS_50_HZ;											  //����ads������������
	init_val.ads_sample_callback = &ads_datahandler;//����ads�������ص�����
	ads_sta = ads_init(&init_val);
#ifdef DEBUG
	if(ads_sta == ADS_OK){
		printf("Flex_6 Initialization Finish\n");
	}else{
	  printf("Flex_6 Initialization Error\n");
	}
#elif RELEASE
	if(ads_sta == ADS_OK){Yled_off;Delay_ms(100);Yled_on;}else{Yled_on;}
#endif
  Yled_off;
	/***********************************����ģ���ʼ��***************************************/
  
	
	
	
	
	
	
	/***********************************����ϵͳ��ʼ��***************************************/
	OSInit(&err);		    														//��ʼ��UCOSIII
	OS_CRITICAL_ENTER();														//�����ٽ���			 
	//������ʼ����
	OSTaskCreate(  (OS_TCB 	* )&StartTaskTCB,				//������ƿ�
								 (CPU_CHAR	* )"start task", 			//��������
                 (OS_TASK_PTR )start_task, 				//������
                 (void		* )0,										//���ݸ��������Ĳ���
                 (OS_PRIO	  )START_TASK_PRIO,     //�������ȼ�
                 (CPU_STK   * )&START_TASK_STK[0],//�����ջ����ַ
                 (CPU_STK_SIZE)START_STK_SIZE/10,	//�����ջ�����λ
                 (CPU_STK_SIZE)START_STK_SIZE,		//�����ջ��С
                 (OS_MSG_QTY  )0,									//�����ڲ���Ϣ�����ܹ����յ������Ϣ��Ŀ,Ϊ0ʱ��ֹ������Ϣ
                 (OS_TICK	  )0,										//��ʹ��ʱ��Ƭ��תʱ��ʱ��Ƭ���ȣ�Ϊ0ʱΪĬ�ϳ��ȣ�
                 (void   	* )0,										//�û�����Ĵ洢��
                 (OS_OPT    )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR, //����ѡ��
                 (OS_ERR 	* )&err);								//��Ÿú�������ʱ�ķ���ֵ
	OS_CRITICAL_EXIT();															//�˳��ٽ���	 
	OSStart(&err);      														//����UCOSIII
  /***************************************************************************************/
								 
	__set_PRIMASK(0);  						  								//�����ж�
}

//��ʼ����������
void start_task(void *p_arg)
{
	OS_ERR err;
	CPU_SR_ALLOC();
	p_arg = p_arg;

	CPU_Init();
#if OS_CFG_STAT_TASK_EN > 0u
    OSStatTaskCPUUsageInit(&err);  //ͳ������                
#endif
	
#ifdef CPU_CFG_INT_DIS_MEAS_EN	  //���ʹ���˲����жϹر�ʱ��
    CPU_IntDisMeasMaxCurReset();
#endif
	
#if	OS_CFG_SCHED_ROUND_ROBIN_EN   //��ʹ��ʱ��Ƭ��ת��ʱ��
	OSSchedRoundRobinCfg(DEF_ENABLED,1,&err);  //ʹ��ʱ��Ƭ��ת���ȹ���,ʱ��Ƭ����Ϊ1��ϵͳʱ�ӽ��ģ���1*5=5ms
#endif
	
	OS_CRITICAL_ENTER();				//�����ٽ���

	Create_LED_Task();
  Create_ADS_Start_Task();
	Create_Data_Report_Task(ADS_DATA);
	Create_Get_MPU_Data_Task(ADS_DATA);
	Creat_EventFlag();
//	OS_Start_Flag = 1;
	
	OS_CRITICAL_EXIT();					//�˳��ٽ���
	
	OSTaskDel((OS_TCB*)0,&err);	//ɾ��start_task��������
}



