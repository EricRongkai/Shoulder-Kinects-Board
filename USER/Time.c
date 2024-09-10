/***************************************************

*�ļ�����:
*         ��ʱ��2,3,4����         
*Author:
*         ���F��
*Time:
*					2018.5.12
*version:
*         v1.0
***************************************************/
#include "Time.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_rcc.h"
#include "misc.h"

void TIM3_Int_Init(u16 arr,u16 psc)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	/* ��ʼ����ʱ��3                                                     */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); 	
	/* ��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ������5000Ϊ500ms*/
	TIM_TimeBaseStructure.TIM_Period = arr; 	   
	/* ����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ                           */ 
	TIM_TimeBaseStructure.TIM_Prescaler =psc-1;  	
	/* ����ʱ�ӷָ�:TDTS = Tck_tim                                     */
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; 
	/* TIM���ϼ���ģʽ                                                 */
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	/* ����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ   */
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); 	
	TIM_ITConfig(TIM3, TIM_IT_Update ,ENABLE);
	
	/* ���ö�ʱ��3�����ȼ�                                                    */
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn; 
	/* ��ռ���ȼ�2��                                                          */
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	/* �����ȼ�3��                                                            */
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2; 
	/* IRQͨ����ʹ��                                                          */
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	/* ����NVIC_InitStruct��ָ���Ĳ�����ʼ������NVIC�Ĵ���                    */
	NVIC_Init(&NVIC_InitStructure); 
	
	TIM_Cmd(TIM3, ENABLE);  
}
/************************************************************************
*��������SysTick_Init(void)
*��  �ܣ����������Ҫ�����9���ں��㷨�е�ʱ�����
*ʱ  �䣺2015/10/4
*��  ��: TrackVR��logzhan
*************************************************************************/
void SysTick_Init(void)
{
	
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	/* ��ʼ����ʱ��2                                                    */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE); 
	/* ��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ             */
	TIM_TimeBaseStructure.TIM_Period = 20000; 
	/* ����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ  10kHz�ļ���Ƶ�ʣ�1us       */
	TIM_TimeBaseStructure.TIM_Prescaler =7200-1; 
	/* ����ʱ�ӷָ�:TDTS = Tck_tim                                     */	
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; 
	/* TIM���ϼ���ģʽ                                                 */
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  	
	/* ����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ   */
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure); 	
	TIM_ITConfig(TIM2, TIM_IT_Update ,ENABLE);
	TIM_Cmd(TIM2, ENABLE);
	/* ���ö�ʱ��2�����ȼ�                                                    */
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn; 
	/* ��ռ���ȼ�2��                                                          */
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	/* �����ȼ�3��                                                            */
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3; 
	/* IRQͨ����ʹ��                                                          */
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	/* ����NVIC_InitStruct��ָ���Ĳ�����ʼ������NVIC�Ĵ���                    */
	NVIC_Init(&NVIC_InitStructure); 
	
	/* ** ��ʼ����ʱ��4��������������̬����  ***/
	
	/*ʱ��ʹ��*/
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE); 
	
	/*��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ������5000Ϊ500ms*/
	TIM_TimeBaseStructure.TIM_Period = 0xffff; 
	
	/*����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ  10Khz�ļ���Ƶ��*/
	TIM_TimeBaseStructure.TIM_Prescaler =7200-1; 
	
	/*����ʱ�ӷָ�:TDTS = Tck_tim*/	
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; 
	
	/*TIM���ϼ���ģʽ*/
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  
	
	/*����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ*/
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure); 
	
	TIM_Cmd(TIM4, ENABLE);
	
}

/************************************************************************
* ������ ��  GET_NOWTIME(void)
* ˵��   ��  ����AHRS�㷨����������ʱ��(������0),���ص�ǰsystick����
             ��ֵ,32λ
* ʱ��   ��  2017/1/15
* ����   ��  TrackVR��logzhan
*************************************************************************/
float GET_NOWTIME(void)  
{
	float Temp=0 ;
	static uint32_t now=0;                                         /* �������ڼ��� ��λ us				*/
 	now = TIM4->CNT;                                               /* ����16λʱ��	                    */
  TIM4->CNT=0;		                                               /* ���¼�ʱ							*/
	Temp = (float)now / 2000000.0f;                                /* �����s���ٳ���2��(1/2 *T)       */
	return Temp;
}

/************************************************************************
* ������ ��  GET_NOWTIME1(void)
* ˵��   ��  ����AHRS�㷨����������ʱ��(������0),���ص�ǰsystick����
             ��ֵ,32λ
* ʱ��   ��  2017/1/15
* ����   ��  TrackVR��logzhan
*************************************************************************/
float GET_NOWTIME1(void)  
{
	float temp2=0;
	static uint32_t now2=0;  			                           /* �������ڼ��� ��λ us	            */
 	now2 = TIM2->CNT;                                        /* ����16λʱ��                      */
  TIM2->CNT=0;												                     /* ���¼�ʱ                          */
	temp2 = (float)now2 / 20000.0f;                        /* �����ms���ٳ���2��(1/2 *T)       */
	return temp2;
}
/************************************************************************
* ������ ��  get_ms(unsigned long *time)
* ˵��   ��  ����MPLlib�㷨����������ʱ��(������0),���ص�ǰsystick����
             ��ֵ,32λ
* ʱ��   ��  2017/1/15
* ����   ��  LRK
*************************************************************************/
void get_ms(unsigned long *time)
{
	static uint32_t now2=0;  			                           /* �������ڼ��� ��λ ms            */
 	now2 = TIM4->CNT;                                        /* ����16λʱ��                      */
  TIM2->CNT=0 ;												                     /* ���¼�ʱ                          */
	*time = now2;                                            /* �õ�msֵ       */
}
