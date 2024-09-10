#include "stm32f10x.h"
#include "Power.h"
#include "stm32f10x_pwr.h"
#include "misc.h"
#include "stm32f10x_exti.h"
#include "LED.h"
#include "Delay.h"
#include "mpu9250.h"

#define WKUP_Sta   			GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_0);

//待机模式
void Sys_Standby(void)
{ 
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);	 //使能PWR外设时钟
	PWR_WakeUpPinCmd(ENABLE); 													 //使能唤醒管脚功能,在WkUp的上升沿进行
	PWR_EnterSTANDBYMode();	  													 //进入待机（STANDBY）模式 设置相应的进入条件（已经封装成了完整的函数）	
}

//系统进入待机模式
//1.复位所有的io
//2.待机模式设置
void Sys_Enter_Standby(void)
{		
	MPU9250_MPU9250sleep();															 //MPU9250进入睡眠模式
	MPU9250_gyromagSleep();															 //MPU9250陀螺仪磁力计进入睡眠模式
	RCC_APB2PeriphResetCmd(0x01fc,DISABLE);//复位
	Sys_Standby();//启动待机模式
}

//检测WKUP脚的信号
//返回值1:连续按下3s以上
//     0:错误的触发，进入待机模式	
u8 Check_WKUP(void)
{
	u8 t=0;									//记录按下的时间
	Yled_on; 								//亮灯LED 
	while(1)
	{
		char WKUP_KD = WKUP_Sta;
		if(WKUP_KD == 1)
		{
			t++;			
			delay_ms(20);
			if(t>=100)					//按下超过3秒钟
			{
				Yled_on;	 				//点亮DS0 
				return 1; 				//按下3s以上了
			}
		}
		else 
		{ 
			Yled_off;
			return 0; 					//按下不足3秒
		}
	}
}  

//PA0 WKUP唤醒初始化
void WKUP_Init(void)
{	
  GPIO_InitTypeDef  GPIO_InitStructure;  		  
	NVIC_InitTypeDef NVIC_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);//使能GPIOA和复用功能时钟

	GPIO_InitStructure.GPIO_Pin =GPIO_Pin_0;	    //PA.0
	GPIO_InitStructure.GPIO_Mode =GPIO_Mode_IPU;  //上拉输入
	GPIO_Init(GPIOA, &GPIO_InitStructure);				//初始化IO

	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource0);		//中断线0连接GPIOA.0
  EXTI_InitStructure.EXTI_Line = EXTI_Line0;									  //设置按键所有的外部线路
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;						//设外外部中断模式:EXTI线路为中断请求
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;  			//上升沿触发
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;											//中断使能
	EXTI_Init(&EXTI_InitStructure);																//初始化外部中断

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn; 							//使能按键所在的外部中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3; 		//先占优先级3级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3; 						//从优先级3级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 							//使能外部中断通道
	NVIC_Init(&NVIC_InitStructure); 															//根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器
}

 

 
