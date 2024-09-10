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

//UCOSIII中以下优先级用户程序不能使用
//将这些优先级分配给了UCOSIII的5个系统内部任务
//优先级0：中断服务服务管理任务 OS_IntQTask()
//优先级1：时钟节拍任务 OS_TickTask()
//优先级2：定时任务 OS_TmrTask()
//优先级OS_CFG_PRIO_MAX-2：统计任务 OS_StatTask()
//优先级OS_CFG_PRIO_MAX-1：空闲任务 OS_IdleTask()

//任务优先级
#define START_TASK_PRIO		3
//任务堆栈大小	
#define START_STK_SIZE 		128
//任务控制块
OS_TCB StartTaskTCB;
//任务堆栈	
CPU_STK START_TASK_STK[START_STK_SIZE];
//任务函数
void start_task(void *p_arg);
//ADS传感器阵列数据地址
uint8_t *ADS_DATA = NULL;

//ADS传感器数据接收回调函数
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
//主函数
int main(void)
{
	/***********************************上电系统初始化***************************************/
	__set_PRIMASK(1);  															//关总中断
	OS_ERR err;																			//定义OS错误变量
	RCC_ClocksTypeDef RCC_clock;										//定义系统时钟检查变量
  CPU_SR_ALLOC();																	//OS初始化寄存器状态，定义初始为0
	LED_Config(); 																	//LED初始化
	Yled_on;
	KEY_Init();																			//按键初始化
	delay_init();  																	//时钟初始化	//LED初始化
	USART1_Config();																//串口初始化
	i2cInit();      							  								//MPU9250IIC总线的初始化
	RCC_GetClocksFreq(&RCC_clock);  								//获取当前各时钟频率
#ifdef DEBUG
	printf("System Initialization Finish !!\n");
#endif
	/***********************************传感模块初始化***************************************/
	if(Mpu9250_Work_Mode_Init()) while(1);					//MPU9250初始化
	
	ads_init_t init_val;	
	int ads_sta = 0;
	ADS_DATA = (uint8_t*)malloc(40);
	memset(ADS_DATA,0,60);
	
	ads_hal_select_device(0x01);													
	init_val.addr = 0x11;														//设置ads传感器I2C地址
	init_val.datardy_pin = GPIO_Pin_4;							//设置ads传感器数据中断引脚
	init_val.reset_pin = GPIO_Pin_5;								//设置ads传感器重置引脚
	init_val.sps = ADS_50_HZ;											  //设置ads传感器采样率
	init_val.ads_sample_callback = &ads_datahandler;//设置ads传感器回调函数
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
	init_val.addr = 0x12;														//设置ads传感器I2C地址
	init_val.datardy_pin = GPIO_Pin_6;							//设置ads传感器数据中断引脚
	init_val.reset_pin = GPIO_Pin_7;								//设置ads传感器重置引脚
	init_val.sps = ADS_50_HZ;											  //设置ads传感器采样率
	init_val.ads_sample_callback = &ads_datahandler;//设置ads传感器回调函数
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
	init_val.addr = 0x13;														//设置ads传感器I2C地址
	init_val.datardy_pin = GPIO_Pin_12;							//设置ads传感器数据中断引脚
	init_val.reset_pin = GPIO_Pin_13;								//设置ads传感器重置引脚
	init_val.sps = ADS_50_HZ;											  //设置ads传感器采样率
	init_val.ads_sample_callback = &ads_datahandler;//设置ads传感器回调函数
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
	init_val.addr = 0x14;														//设置ads传感器I2C地址
	init_val.datardy_pin = GPIO_Pin_15;							//设置ads传感器数据中断引脚
	init_val.reset_pin = GPIO_Pin_3;								//设置ads传感器重置引脚
	init_val.sps = ADS_50_HZ;											  //设置ads传感器采样率
	init_val.ads_sample_callback = &ads_datahandler;//设置ads传感器回调函数
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
	init_val.addr = 0x15;														//设置ads传感器I2C地址
	init_val.datardy_pin = GPIO_Pin_5;							//设置ads传感器数据中断引脚
	init_val.reset_pin = GPIO_Pin_4;								//设置ads传感器重置引脚
	init_val.sps = ADS_50_HZ;											  //设置ads传感器采样率
	init_val.ads_sample_callback = &ads_datahandler;//设置ads传感器回调函数
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
	init_val.addr = 0x16;														//设置ads传感器I2C地址
	init_val.datardy_pin = GPIO_Pin_8;							//设置ads传感器数据中断引脚
	init_val.reset_pin = GPIO_Pin_9;								//设置ads传感器重置引脚
	init_val.sps = ADS_50_HZ;											  //设置ads传感器采样率
	init_val.ads_sample_callback = &ads_datahandler;//设置ads传感器回调函数
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
	/***********************************蓝牙模块初始化***************************************/
  
	
	
	
	
	
	
	/***********************************操作系统初始化***************************************/
	OSInit(&err);		    														//初始化UCOSIII
	OS_CRITICAL_ENTER();														//进入临界区			 
	//创建开始任务
	OSTaskCreate(  (OS_TCB 	* )&StartTaskTCB,				//任务控制块
								 (CPU_CHAR	* )"start task", 			//任务名字
                 (OS_TASK_PTR )start_task, 				//任务函数
                 (void		* )0,										//传递给任务函数的参数
                 (OS_PRIO	  )START_TASK_PRIO,     //任务优先级
                 (CPU_STK   * )&START_TASK_STK[0],//任务堆栈基地址
                 (CPU_STK_SIZE)START_STK_SIZE/10,	//任务堆栈深度限位
                 (CPU_STK_SIZE)START_STK_SIZE,		//任务堆栈大小
                 (OS_MSG_QTY  )0,									//任务内部消息队列能够接收的最大消息数目,为0时禁止接收消息
                 (OS_TICK	  )0,										//当使能时间片轮转时的时间片长度，为0时为默认长度，
                 (void   	* )0,										//用户补充的存储区
                 (OS_OPT    )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR, //任务选项
                 (OS_ERR 	* )&err);								//存放该函数错误时的返回值
	OS_CRITICAL_EXIT();															//退出临界区	 
	OSStart(&err);      														//开启UCOSIII
  /***************************************************************************************/
								 
	__set_PRIMASK(0);  						  								//开总中断
}

//开始任务任务函数
void start_task(void *p_arg)
{
	OS_ERR err;
	CPU_SR_ALLOC();
	p_arg = p_arg;

	CPU_Init();
#if OS_CFG_STAT_TASK_EN > 0u
    OSStatTaskCPUUsageInit(&err);  //统计任务                
#endif
	
#ifdef CPU_CFG_INT_DIS_MEAS_EN	  //如果使能了测量中断关闭时间
    CPU_IntDisMeasMaxCurReset();
#endif
	
#if	OS_CFG_SCHED_ROUND_ROBIN_EN   //当使用时间片轮转的时候
	OSSchedRoundRobinCfg(DEF_ENABLED,1,&err);  //使能时间片轮转调度功能,时间片长度为1个系统时钟节拍，既1*5=5ms
#endif
	
	OS_CRITICAL_ENTER();				//进入临界区

	Create_LED_Task();
  Create_ADS_Start_Task();
	Create_Data_Report_Task(ADS_DATA);
	Create_Get_MPU_Data_Task(ADS_DATA);
	Creat_EventFlag();
//	OS_Start_Flag = 1;
	
	OS_CRITICAL_EXIT();					//退出临界区
	
	OSTaskDel((OS_TCB*)0,&err);	//删除start_task任务自身
}



