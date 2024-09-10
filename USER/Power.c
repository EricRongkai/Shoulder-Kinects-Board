#include "stm32f10x.h"
#include "Power.h"
#include "stm32f10x_pwr.h"
#include "misc.h"
#include "stm32f10x_exti.h"
#include "LED.h"
#include "Delay.h"
#include "mpu9250.h"

#define WKUP_Sta   			GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_0);

//����ģʽ
void Sys_Standby(void)
{ 
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);	 //ʹ��PWR����ʱ��
	PWR_WakeUpPinCmd(ENABLE); 													 //ʹ�ܻ��ѹܽŹ���,��WkUp�������ؽ���
	PWR_EnterSTANDBYMode();	  													 //���������STANDBY��ģʽ ������Ӧ�Ľ����������Ѿ���װ���������ĺ�����	
}

//ϵͳ�������ģʽ
//1.��λ���е�io
//2.����ģʽ����
void Sys_Enter_Standby(void)
{		
	MPU9250_MPU9250sleep();															 //MPU9250����˯��ģʽ
	MPU9250_gyromagSleep();															 //MPU9250�����Ǵ����ƽ���˯��ģʽ
	RCC_APB2PeriphResetCmd(0x01fc,DISABLE);//��λ
	Sys_Standby();//��������ģʽ
}

//���WKUP�ŵ��ź�
//����ֵ1:��������3s����
//     0:����Ĵ������������ģʽ	
u8 Check_WKUP(void)
{
	u8 t=0;									//��¼���µ�ʱ��
	Yled_on; 								//����LED 
	while(1)
	{
		char WKUP_KD = WKUP_Sta;
		if(WKUP_KD == 1)
		{
			t++;			
			delay_ms(20);
			if(t>=100)					//���³���3����
			{
				Yled_on;	 				//����DS0 
				return 1; 				//����3s������
			}
		}
		else 
		{ 
			Yled_off;
			return 0; 					//���²���3��
		}
	}
}  

//PA0 WKUP���ѳ�ʼ��
void WKUP_Init(void)
{	
  GPIO_InitTypeDef  GPIO_InitStructure;  		  
	NVIC_InitTypeDef NVIC_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);//ʹ��GPIOA�͸��ù���ʱ��

	GPIO_InitStructure.GPIO_Pin =GPIO_Pin_0;	    //PA.0
	GPIO_InitStructure.GPIO_Mode =GPIO_Mode_IPU;  //��������
	GPIO_Init(GPIOA, &GPIO_InitStructure);				//��ʼ��IO

	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource0);		//�ж���0����GPIOA.0
  EXTI_InitStructure.EXTI_Line = EXTI_Line0;									  //���ð������е��ⲿ��·
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;						//�����ⲿ�ж�ģʽ:EXTI��·Ϊ�ж�����
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;  			//�����ش���
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;											//�ж�ʹ��
	EXTI_Init(&EXTI_InitStructure);																//��ʼ���ⲿ�ж�

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn; 							//ʹ�ܰ������ڵ��ⲿ�ж�ͨ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3; 		//��ռ���ȼ�3��
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3; 						//�����ȼ�3��
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 							//ʹ���ⲿ�ж�ͨ��
	NVIC_Init(&NVIC_InitStructure); 															//����NVIC_InitStruct��ָ���Ĳ�����ʼ������NVIC�Ĵ���
}

 

 
