#include "sys.h"

//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK Mini STM32开发板
//系统中断分组设置化		   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//修改日期:2012/9/10
//版本：V1.4
//版权所有，盗版必究。
//Copyright(C) 正点原子 2009-2019
//All rights reserved
//********************************************************************************  

//THUMB指令不支持汇编内联
//采用如下方法实现执行汇编指令WFI  
void WFI_SET(void)
{
	__ASM volatile("wfi");		  
}
//关闭所有中断
void INTX_DISABLE(void)
{		  
	__ASM volatile("cpsid i");
}
//开启所有中断
void INTX_ENABLE(void)
{
	__ASM volatile("cpsie i");		  
}
//设置栈顶地址
//addr:栈顶地址
__asm void MSR_MSP(u32 addr) 
{
    MSR MSP, r0 			//set Main Stack value
    BX r14
}

void RCC_Configuration(void)
{

		RCC_DeInit();//将外设RCC寄存器重设为缺省值

		RCC_HSICmd(ENABLE);//使能HSI

		while(RCC_GetFlagStatus(RCC_FLAG_HSIRDY)==RESET);//等待HSI使能成功

		//FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);

		//FLASH_SetLatency(FLASH_Latency_2);

		RCC_HCLKConfig(RCC_SYSCLK_Div1);

		RCC_PCLK1Config(RCC_HCLK_Div2);

		RCC_PCLK2Config(RCC_HCLK_Div1);

		//设置PLL时钟源及倍频系数

		RCC_PLLConfig(RCC_PLLSource_HSI_Div2,RCC_PLLMul_2);//使能或者失能PLL,这个参数可以取：ENABLE或者DISABLE

		RCC_PLLCmd(ENABLE);//如果PLL被用于系统时钟,那么它不能被失能

		//等待指定的RCC标志位设置成功等待PLL初始化成功

		while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY)==RESET);

		//设置系统时钟（SYSCLK）设置PLL为系统时钟源

		RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);//选择想要的系统时钟

		//等待PLL成功用作于系统时钟的时钟源

		//0x00：HSI作为系统时钟

		//0x04：HSE作为系统时钟

		//0x08：PLL作为系统时钟

		while(RCC_GetSYSCLKSource()!=0x08);//需与被选择的系统时钟对应起来，RCC_SYSCLKSource_PLL

}
