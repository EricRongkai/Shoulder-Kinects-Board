#include "LED.h"
#include "stm32f10x.h"

//指示灯配置
void LED_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
  
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);   //初始化PB15蓝色SYS_STA1状态指示
    
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);    
  
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);		//初始化PA8黄色SYS_STA2状态指示
    
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);    
    
	Yled_off;
	Bled_off;
}

