/***************************************************

*文件描述:
*         定时器2,3,4驱动         
*Author:
*         刘镕恺
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
	
	/* 初始化定时器3                                                     */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); 	
	/* 设置在下一个更新事件装入活动的自动重装载寄存器周期的值计数到5000为500ms*/
	TIM_TimeBaseStructure.TIM_Period = arr; 	   
	/* 设置用来作为TIMx时钟频率除数的预分频值                           */ 
	TIM_TimeBaseStructure.TIM_Prescaler =psc-1;  	
	/* 设置时钟分割:TDTS = Tck_tim                                     */
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; 
	/* TIM向上计数模式                                                 */
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	/* 根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位   */
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); 	
	TIM_ITConfig(TIM3, TIM_IT_Update ,ENABLE);
	
	/* 配置定时器3的优先级                                                    */
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn; 
	/* 先占优先级2级                                                          */
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	/* 从优先级3级                                                            */
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2; 
	/* IRQ通道被使能                                                          */
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	/* 根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器                    */
	NVIC_Init(&NVIC_InitStructure); 
	
	TIM_Cmd(TIM3, ENABLE);  
}
/************************************************************************
*函数名：SysTick_Init(void)
*功  能：这个函数主要是针对9轴融合算法中的时间采样
*时  间：2015/10/4
*作  者: TrackVR：logzhan
*************************************************************************/
void SysTick_Init(void)
{
	
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	/* 初始化定时器2                                                    */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE); 
	/* 设置在下一个更新事件装入活动的自动重装载寄存器周期的值             */
	TIM_TimeBaseStructure.TIM_Period = 20000; 
	/* 设置用来作为TIMx时钟频率除数的预分频值  10kHz的计数频率，1us       */
	TIM_TimeBaseStructure.TIM_Prescaler =7200-1; 
	/* 设置时钟分割:TDTS = Tck_tim                                     */	
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; 
	/* TIM向上计数模式                                                 */
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  	
	/* 根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位   */
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure); 	
	TIM_ITConfig(TIM2, TIM_IT_Update ,ENABLE);
	TIM_Cmd(TIM2, ENABLE);
	/* 配置定时器2的优先级                                                    */
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn; 
	/* 先占优先级2级                                                          */
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	/* 从优先级3级                                                            */
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3; 
	/* IRQ通道被使能                                                          */
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	/* 根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器                    */
	NVIC_Init(&NVIC_InitStructure); 
	
	/* ** 初始化定时器4，用于陀螺仪姿态解算  ***/
	
	/*时钟使能*/
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE); 
	
	/*设置在下一个更新事件装入活动的自动重装载寄存器周期的值计数到5000为500ms*/
	TIM_TimeBaseStructure.TIM_Period = 0xffff; 
	
	/*设置用来作为TIMx时钟频率除数的预分频值  10Khz的计数频率*/
	TIM_TimeBaseStructure.TIM_Prescaler =7200-1; 
	
	/*设置时钟分割:TDTS = Tck_tim*/	
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; 
	
	/*TIM向上计数模式*/
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  
	
	/*根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位*/
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure); 
	
	TIM_Cmd(TIM4, ENABLE);
	
}

/************************************************************************
* 函数名 ：  GET_NOWTIME(void)
* 说明   ：  用于AHRS算法解算计算采样时间(传感器0),返回当前systick计数
             器值,32位
* 时间   ：  2017/1/15
* 作者   ：  TrackVR：logzhan
*************************************************************************/
float GET_NOWTIME(void)  
{
	float Temp=0 ;
	static uint32_t now=0;                                         /* 采样周期计数 单位 us				*/
 	now = TIM4->CNT;                                               /* 读高16位时间	                    */
  TIM4->CNT=0;		                                               /* 重新计时							*/
	Temp = (float)now / 2000000.0f;                                /* 换算成s，再除以2得(1/2 *T)       */
	return Temp;
}

/************************************************************************
* 函数名 ：  GET_NOWTIME1(void)
* 说明   ：  用于AHRS算法解算计算采样时间(传感器0),返回当前systick计数
             器值,32位
* 时间   ：  2017/1/15
* 作者   ：  TrackVR：logzhan
*************************************************************************/
float GET_NOWTIME1(void)  
{
	float temp2=0;
	static uint32_t now2=0;  			                           /* 采样周期计数 单位 us	            */
 	now2 = TIM2->CNT;                                        /* 读高16位时间                      */
  TIM2->CNT=0;												                     /* 重新计时                          */
	temp2 = (float)now2 / 20000.0f;                        /* 换算成ms，再除以2得(1/2 *T)       */
	return temp2;
}
/************************************************************************
* 函数名 ：  get_ms(unsigned long *time)
* 说明   ：  用于MPLlib算法解算计算采样时间(传感器0),返回当前systick计数
             器值,32位
* 时间   ：  2017/1/15
* 作者   ：  LRK
*************************************************************************/
void get_ms(unsigned long *time)
{
	static uint32_t now2=0;  			                           /* 采样周期计数 单位 ms            */
 	now2 = TIM4->CNT;                                        /* 读高16位时间                      */
  TIM2->CNT=0 ;												                     /* 重新计时                          */
	*time = now2;                                            /* 得到ms值       */
}
