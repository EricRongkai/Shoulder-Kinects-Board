/**
  ******************************************************************************
  * @file    GPIO/IOToggle/stm32f10x_it.c 
  * @author  
  * @version V1.0
  * @date    3/30/2015
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and peripherals
  *          interrupt service routine.
  ******************************************************************************
  * @copy
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2010 STMicroelectronics</center></h2>
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_it.h"
#include "stm32f10x_exti.h"
#include "EMG_Signal.h"
#include "mpu9250.h"
#include "delay.h"
#include "string.h"
#include "includes.h"
#include "LED.h"
#include "Power.h"
#include "Task.h"
#include "ads_hal.h"

#define RawData
//#define Mahony

extern void gyro_data_ready_cb(void);
int Magn_Flag = 0;
unsigned char DataReady; 
extern float Pitch,Roll,Yaw;
signed short mpu_acce[3] = {0};
signed short mpu_gyro[3] = {0};
signed short mpu_magn[3] = {0};

extern MPU9250_Data Raw_Data; 
extern MPU9250_Data Cali_Data;
extern float q[4];
unsigned short count_id; 
extern volatile uint16_t g_AdcFilteredValue;
OS_ERR err;

#define  led_on    GPIO_ResetBits(GPIOB, GPIO_Pin_13)
#define  led_off   GPIO_SetBits(GPIOB, GPIO_Pin_13)

/** @addtogroup STM32F10x_StdPeriph_Examples
  * @{
  */

/** @addtogroup GPIO_IOToggle
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
		printf("ERROR HARDFAULT!!\n");
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSV_Handler exception.
  * @param  None
  * @retval None
  */
//void PendSV_Handler(void)
//{
//}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
//void SysTick_Handler(void)
//{
//}

/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/
void TIM3_IRQHandler(void)
{
	 OSIntEnter();
	
   if( TIM_GetITStatus(TIM3 , TIM_IT_Update) != RESET)
   {
		 
		   DataReady=1;
			 if(count_id < 50000)
			 {
				 count_id++;
			 }
		 	 else count_id=0;
			 
			 READ_MPU9250_ACCEL();		 
  		 READ_MPU9250_GYRO();	
	  }
	 TIM_ClearITPendingBit(TIM3 , TIM_FLAG_Update);
	OSIntExit();
}

void TIM2_IRQHandler(void)
{
	 OSIntEnter();
   if( TIM_GetITStatus(TIM2 , TIM_IT_Update) != RESET)
   { 
#ifdef Mahony
	   AHRSupdate(Cali_Data.Gyro_Data[0], Cali_Data.Gyro_Data[1], Cali_Data.Gyro_Data[2],
							  Cali_Data.Acce_Data[0], Cali_Data.Acce_Data[1], Cali_Data.Acce_Data[2],
							  Cali_Data.Magn_data[0], Cali_Data.Magn_data[1], Cali_Data.Magn_data[2],q,0,1);
#endif
	 }
	 TIM_ClearITPendingBit(TIM2 , TIM_FLAG_Update);

	 OSIntExit();
}


char Uart1_Buffer[128];       //接收缓冲区
char Uart1_Rx = 0;            //Uart1_Buffer下标
char Uart1_Sta = 0;           //接受状态
char Clear;

void USART1_IRQHandler() 
{ 
	 OSIntEnter();

  if(USART_GetITStatus(USART1,USART_IT_RXNE) != RESET) 
  { 
		USART_ClearITPendingBit(USART1,USART_IT_RXNE);
		Uart1_Buffer[Uart1_Rx] = USART_ReceiveData(USART1);
		Uart1_Rx++;
	}
	
	else if(USART_GetITStatus(USART1,USART_IT_IDLE) == SET)//传输完一条完整的数据就会进入这个
	{
		Uart1_Sta = 1;																	//接收到一条完整的数据
		Clear = USART1->SR; 														//清USART_IT_IDLE标志
		Clear = USART1->DR;
		Uart1_Rx = 0;																		//清零接收的个数
  } 
  
	if(USART_GetFlagStatus(USART1,USART_FLAG_ORE) == SET) 
	{ 
		USART_ClearFlag(USART1,USART_FLAG_ORE); 
		USART_ReceiveData(USART1); 
	}
	
		OSIntExit();
}

//char Uart3_Buffer[128];       //接收缓冲区
//char Uart3_Rx = 0;           //Uart1_Buffer下标
//char Uart3_Sta = 0;          //接受状态

//void USART3_IRQHandler() 
//{ 
//	if(OS_Start_Flag==1)
//	{
//	 OSIntEnter();
//	}
//  if(USART_GetITStatus(USART3,USART_IT_RXNE) != RESET) 
//  { 
//		USART_ClearITPendingBit(USART3,USART_IT_RXNE);
//		BLE2.TempData = USART_ReceiveData(USART3);
//		Uart3_Buffer[Uart3_Rx] = BLE2.TempData;
//		Uart3_Rx++;
//	}
//	
//	else if(USART_GetITStatus(USART3,USART_IT_IDLE) == SET)//传输完一条完整的数据就会进入这个
//	{
//		Uart3_Sta = 1;																	//接收到一条完整的数据
//		BLE2.NewATReturn = 1;
//		if(Uart3_Buffer[0] == 'C'&& Uart3_Buffer[1] == 'M'&& Uart3_Buffer[2]=='D')
//		{
//			if(Uart3_Buffer[3]=='1')
//			{
//				OSFlagPost((OS_FLAG_GRP*) &MeasureEvent,
//									 (OS_FLAGS) MeasureFlag,
//									 (OS_OPT) OS_OPT_POST_FLAG_SET,
//									 (OS_ERR*) &err);
//			  BLE1_Send_AT_Command("CMD1");
//			}
//			if(Uart3_Buffer[3]=='0')
//			{
//				OSFlagPost((OS_FLAG_GRP*) &MeasureEvent,
//									 (OS_FLAGS) MeasureFlag,
//									 (OS_OPT) OS_OPT_POST_FLAG_CLR,
//									 (OS_ERR*) &err);
//				BLE1_Send_AT_Command("CMD0");
//				Bled_off;
//				Yled_off;
//			}
//		}
//		BLE2.DataReceive = Uart3_Buffer;								//更新数据
//		Clear = USART3->SR; 														//清USART_IT_IDLE标志
//		Clear = USART3->DR;
//		BLE2.DataAmount = Uart3_Rx;				   						//复制接收到的数据个数
//		Uart3_Rx = 0;																		//清零接收的个数
//  } 
//  
//	if(USART_GetFlagStatus(USART3,USART_FLAG_ORE) == SET) 
//	{ 
//		USART_ClearFlag(USART3,USART_FLAG_ORE); 
//		USART_ReceiveData(USART3); 
//	}
//	if(OS_Start_Flag==1)
//	{
//	 OSIntExit();
//	}
//}

bool KEY_flag = true;
void EXTI0_IRQHandler()
{
	OS_ERR err;
	
	OSIntEnter();
	
	if(EXTI_GetITStatus(EXTI_Line0)!=RESET)
	{
		EXTI_ClearITPendingBit(EXTI_Line0);
		if(KEY_flag==true)
		{
			OS_FlagPost(&MeasureEvent,MeasureFlag,OS_OPT_POST_FLAG_SET,0,&err);
			KEY_flag = false;
		}
		else if(KEY_flag==false)
		{
			OS_FlagPost(&MeasureEvent,MeasureFlag,OS_OPT_POST_FLAG_CLR,0,&err);
			KEY_flag = true;
		}
	}
	
	OSIntExit();
}

void EXTI4_IRQHandler()
{
	OSIntEnter();
	
	if(EXTI_GetITStatus(EXTI_Line4)!=RESET)
	{
		EXTI_ClearITPendingBit(EXTI_Line4);
		ads_hal_select_device(0x01);
		ads_hal_interrupt(0x01);
	}
	
	OSIntExit();
}
void EXTI9_5_IRQHandler()
{
	OSIntEnter();
	
	if(EXTI_GetITStatus(EXTI_Line5)!=RESET)
	{
		EXTI_ClearITPendingBit(EXTI_Line5);
		ads_hal_select_device(0x05);
		ads_hal_interrupt(0x05);
	}
	if(EXTI_GetITStatus(EXTI_Line6)!=RESET)
	{
		EXTI_ClearITPendingBit(EXTI_Line6);
		ads_hal_select_device(0x02);
		ads_hal_interrupt(0x02);
	}
	if(EXTI_GetITStatus(EXTI_Line8)!=RESET)
	{
		EXTI_ClearITPendingBit(EXTI_Line8);
		ads_hal_select_device(0x06);
		ads_hal_interrupt(0x06);
	}
	
	OSIntExit();
}

void EXTI15_10_IRQHandler()
{
	OSIntEnter();
	
	if(EXTI_GetITStatus(EXTI_Line15)!=RESET)
	{
		EXTI_ClearITPendingBit(EXTI_Line15);
		ads_hal_select_device(0x04);
		ads_hal_interrupt(0x04);
	}
	
	if(EXTI_GetITStatus(EXTI_Line12)!=RESET)
	{
		EXTI_ClearITPendingBit(EXTI_Line12);
		ads_hal_select_device(0x03);
		ads_hal_interrupt(0x03);
	}
	
	OSIntExit();
}
/*
*********************************************************************************************************
*	函 数 名: DMA1_Channel1_IRQHandler
*	功能说明: ADC1用的DMA传输完成中断服务函数
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void DMA1_Channel1_IRQHandler(void)
{
//	AdcDmaIrq();
}

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @}
  */

/**
  * @}
  */

/******************* (C) COPYRIGHT 2010 STMicroelectronics *****END OF FILE****/
