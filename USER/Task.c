
/***************************************************

*文件描述:
*         通用任务
*Author:
*         刘镕恺
*Time:
*					2022.3.12
*version:
*         v1.0
***************************************************/

#include "LED.h"
#include "Task.h"
#include "delay.h"
#include "mpu9250.h"
#include "includes.h"
#include "usart.h"
#include "ads.h"
#include "ads_hal.h"
#include "CRC.h"


#define true 1
#define false 0 
#define bool  uint8_t

#define BYTE0(dwTemp)       (*(char *)(&dwTemp))
#define BYTE1(dwTemp)       (*((char *)(&dwTemp) + 1))
#define BYTE2(dwTemp)       (*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp)       (*((char *)(&dwTemp) + 3))

OS_TCB BLED_Blink3_TaskTCB;
CPU_STK BLED_Blink3_TASK_STK[BLED_Blink3_STK_SIZE];

OS_TCB YLED_Blink3_TaskTCB;
CPU_STK YLED_Blink3_TASK_STK[YLED_Blink3_STK_SIZE];

OS_TCB Get_MPU_Data_TaskTCB;
CPU_STK Get_MPU_Data_TASK_STK[Get_MPU_Data_STK_SIZE];

OS_TCB ADS_Run_TaskTCB;
CPU_STK ADS_Run_TASK_STK[ADS_Run_STK_SIZE];

OS_TCB Data_Report_TaskTCB;
CPU_STK Data_Report_TASK_STK[Data_Report_STK_SIZE];

OS_TCB Status_Check_TaskTCB;
CPU_STK Status_Check_TASK_STK[Status_Check_STK_SIZE];
/*********************************************************************************************/
void Creat_EventFlag(void)
{
	OS_ERR err;
	OSFlagCreate((OS_FLAG_GRP*) &MeasureEvent,
							 (CPU_CHAR*) "MeasureEvent",
							 (OS_FLAGS) 0x00,
							 (OS_ERR*) err);
}
/*********************************************************************************************/
void Create_LED_Task(void)
{
	OS_ERR err;
	CPU_SR_ALLOC();
	//创建BLED闪烁任务
	OSTaskCreate((OS_TCB 	* )&BLED_Blink3_TaskTCB,		
				       (CPU_CHAR	* )"BLED Blink task", 		
               (OS_TASK_PTR ) BLED_Blink3_task, 			
               (void		* )0,					
               (OS_PRIO	  )BLED_Blink3_TASK_PRIO,     
               (CPU_STK   * )&BLED_Blink3_TASK_STK[0],	
               (CPU_STK_SIZE)BLED_Blink3_STK_SIZE/10,	
               (CPU_STK_SIZE)BLED_Blink3_STK_SIZE,		
               (OS_MSG_QTY  )0,					
               (OS_TICK	  )0,					
               (void   	* )0,					
               (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR,
               (OS_ERR 	* )&err);				
							 
	//创建YLED闪烁任务
	OSTaskCreate((OS_TCB 	* )&YLED_Blink3_TaskTCB,		
							 (CPU_CHAR	* )"YLED Blink task", 		
               (OS_TASK_PTR )YLED_Blink3_task, 			
               (void		* )0,					
               (OS_PRIO	  )YLED_Blink3_TASK_PRIO,     	
               (CPU_STK   * )&YLED_Blink3_TASK_STK[0],	
               (CPU_STK_SIZE)YLED_Blink3_STK_SIZE/10,	
               (CPU_STK_SIZE)YLED_Blink3_STK_SIZE,		
               (OS_MSG_QTY  )0,					
               (OS_TICK	  )0,					
               (void   	* )0,				
               (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR, 
               (OS_ERR 	* )&err);
	OSTaskSuspend(&YLED_Blink3_TaskTCB,&err);
	//OSTaskSuspend(&BLED_Blink3_TaskTCB,&err);
}

//BLED闪烁3次 周期3秒
void BLED_Blink3_task(void *p_arg)
{
	OS_ERR err;
	CPU_SR_ALLOC();
	p_arg = p_arg;
	while(1)
	{
		Bled_on;
		OSTimeDlyHMSM(0,0,0,100,OS_OPT_TIME_HMSM_STRICT,&err);
		Bled_off;
		OSTimeDlyHMSM(0,0,0,100,OS_OPT_TIME_HMSM_STRICT,&err);
		Bled_on;
		OSTimeDlyHMSM(0,0,0,100,OS_OPT_TIME_HMSM_STRICT,&err);
		Bled_off;
		OSTimeDlyHMSM(0,0,0,100,OS_OPT_TIME_HMSM_STRICT,&err);
		Bled_on;
		OSTimeDlyHMSM(0,0,0,100,OS_OPT_TIME_HMSM_STRICT,&err);
		Bled_off;
		OSTimeDlyHMSM(0,0,3,0,OS_OPT_TIME_HMSM_STRICT,&err);
	}
}

//YLED闪烁3次 周期3秒
void YLED_Blink3_task(void *p_arg)
{
	OS_ERR err;
	CPU_SR_ALLOC();
	p_arg = p_arg;
	while(1)
	{
		Yled_on;
		OSTimeDlyHMSM(0,0,0,100,OS_OPT_TIME_HMSM_STRICT,&err);
		Yled_off;
		OSTimeDlyHMSM(0,0,0,100,OS_OPT_TIME_HMSM_STRICT,&err);
		Yled_on;
		OSTimeDlyHMSM(0,0,0,100,OS_OPT_TIME_HMSM_STRICT,&err);
		Yled_off;
		OSTimeDlyHMSM(0,0,0,100,OS_OPT_TIME_HMSM_STRICT,&err);
		Yled_on;
		OSTimeDlyHMSM(0,0,0,100,OS_OPT_TIME_HMSM_STRICT,&err);
		Yled_off;
		OSTimeDlyHMSM(0,0,3,0,OS_OPT_TIME_HMSM_STRICT,&err);
	}
}

/*********************************************************************************************/
void Create_Get_MPU_Data_Task(void *buf)
{
	OS_ERR err;
	CPU_SR_ALLOC();
	OSTaskCreate((OS_TCB 	* )&Get_MPU_Data_TaskTCB,		
						 (CPU_CHAR	* )"Get_MPU_Data task", 		
						 (OS_TASK_PTR ) Get_MPU_Data_task, 			
						 (void		* )  buf,					
						 (OS_PRIO	  )  Get_MPU_Data_TASK_PRIO,     
						 (CPU_STK   * )&Get_MPU_Data_TASK_STK[0],	
						 (CPU_STK_SIZE)Get_MPU_Data_STK_SIZE/10,	
						 (CPU_STK_SIZE)Get_MPU_Data_STK_SIZE,		
						 (OS_MSG_QTY  )0,					
						 (OS_TICK	  )0,					
						 (void   	* )0,					
						 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR,
						 (OS_ERR 	* )&err);	
//	OSTaskSuspend(&Get_MPU_Data_TaskTCB,&err);
}

void Get_MPU_Data_task(void *p_arg)
{	
	OS_ERR err;
	uint8_t *buf = (uint8_t *)p_arg;
	uint8_t *data;
	while(1)
	{	
		//读取惯性传感器数据
    data = READ_MPU9250_ACCEL();	
		memcpy(buf+39,data,6);
		data = READ_MPU9250_GYRO();
		memcpy(buf+45,data,6);
		OSTimeDlyHMSM(0,0,0,20,OS_OPT_TIME_HMSM_STRICT,&err);
	}
}
/*********************************************************************************************/
void Create_ADS_Start_Task(void)
{
	OS_ERR err;
	CPU_SR_ALLOC();
	OSTaskCreate((OS_TCB 	* )&ADS_Run_TaskTCB,		
						 (CPU_CHAR	* )"ADS_Run task", 		
						 (OS_TASK_PTR ) ADS_Run_task, 			
						 (void		* )0,					
						 (OS_PRIO	  )  ADS_Run_TASK_PRIO,     
						 (CPU_STK   * )&ADS_Run_TASK_STK[0],	
						 (CPU_STK_SIZE)ADS_Run_STK_SIZE/10,	
						 (CPU_STK_SIZE)ADS_Run_STK_SIZE,		
						 (OS_MSG_QTY  )0,					
						 (OS_TICK	  )0,					
						 (void   	* )0,					
						 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR,
						 (OS_ERR 	* )&err);
}

//ADS传感器使能
void ADS_Run_task(void *p_arg)
{
	OS_ERR err;
	CPU_SR_ALLOC();
	p_arg = p_arg;
	ads_hal_select_device(0x01);
	ads_stretch_en(true);
	ads_run(true);
	
	ads_hal_select_device(0x02);
	ads_stretch_en(true);
	ads_run(true);
	
	ads_hal_select_device(0x03);
	ads_two_axis_enable_axis(ADS_AXIS_0_EN| ADS_AXIS_1_EN);
  ads_run(true);
	
	ads_hal_select_device(0x04);
	ads_two_axis_enable_axis(ADS_AXIS_0_EN| ADS_AXIS_1_EN);
  ads_run(true);
	
	ads_hal_select_device(0x05);
	ads_stretch_en(true);
	ads_run(true);
	
	ads_hal_select_device(0x06);
	ads_stretch_en(true);
	ads_run(true);
}
/*********************************************************************************************/
void Create_Data_Report_Task(void *buf)
{
	OS_ERR err;
	CPU_SR_ALLOC();
	OSTaskCreate((OS_TCB 	* )&Data_Report_TaskTCB,		
						 (CPU_CHAR	* )"Data_Report task", 		
						 (OS_TASK_PTR ) Data_Report_task, 			
						 (void		* )  buf,					
						 (OS_PRIO	  )  Data_Report_TASK_PRIO,     
						 (CPU_STK   * )&Data_Report_TASK_STK[0],	
						 (CPU_STK_SIZE)Data_Report_STK_SIZE/2,	
						 (CPU_STK_SIZE)Data_Report_STK_SIZE,		
						 (OS_MSG_QTY  )0,					
						 (OS_TICK	  )0,					
						 (void   	* )0,					
						 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR,
						 (OS_ERR 	* )&err);
}

//50Hz数据报告频率
/********************************************/
/**************** 数据包格式 ****************/
/* 序号       功能          数据      长度   */
/* 0          帧头          0xAA       1    */
/* 1-4       时间戳         ----       4    */
/* 5-7    ADS_1弯曲数据     ----       3    */
/* 8-10   ADS_1拉伸数据     ----       3    */
/* 11-13  ADS_2弯曲数据     ----       3    */
/* 14-16  ADS_2拉伸数据     ----       3    */
/* 17-21  ADS_3弯曲数据     ----       5    */
/* 22-26  ADS_4弯曲数据     ----       5    */
/* 27-29  ADS_5弯曲数据     ----       3    */
/* 30-32  ADS_5拉伸数据     ----       3    */
/* 33-35  ADS_6弯曲数据     ----       3    */
/* 36-38  ADS_6拉伸数据     ----       3    */
/* 39-44   加速度计XYZ      ----       6    */
/* 45-50    陀螺仪XYZ       ----       6    */
/* 51-57     预留位         0x00       8    */
/* 58       CRC8校验位       ----       1    */
/* 59         帧尾          0xCD       1    */
/********************************************/

OS_FLAG_GRP MeasureEvent;        //测量数据事件组
void Data_Report_task(void *p_arg)
{
	OS_ERR err;
	CPU_SR_ALLOC();
	uint8_t *buf = (uint8_t *)p_arg;
	unsigned int TS = 0;
	uint32_t SUM = 0;
	uint8_t BCC = 0;
	*(buf)   = 0xAA;
	while(1)
	{
//		OS_FLAGS flags_num = OSFlagPend(&MeasureEvent,MeasureFlag,0,OS_OPT_PEND_FLAG_SET_ALL + OS_OPT_PEND_BLOCKING,0,&err);
    if (TS<=4294967295){TS ++;}else TS =0;
		*(buf+1) = BYTE0(TS);
		*(buf+2) = BYTE1(TS);
		*(buf+3) = BYTE2(TS);
		*(buf+4) = BYTE3(TS);
//		BCC = crc8(buf,58);
		for(int i=0;i<58;i++)
		{
			SUM = SUM + buf[i];
		}
		BCC = SUM & 0xFF;
		*(buf+58) = BCC;
		*(buf+59) = 0xCD;
		SUM = 0;
		UsartSendPackage(p_arg,60);
		OSTimeDlyHMSM(0,0,0,20,OS_OPT_TIME_HMSM_STRICT,&err);

	}
}

/*********************************************************************************************/
#define PA0_Sta   			GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_0);
void Create_Status_Check_Task(void)
{
	OS_ERR err;
	CPU_SR_ALLOC();
	OSTaskCreate((OS_TCB 	* )&Status_Check_TaskTCB,		
						 (CPU_CHAR	* )"Status_Check_task", 		
						 (OS_TASK_PTR ) Status_Check_task, 			
						 (void		* )0,					
						 (OS_PRIO	  )  Status_Check_TASK_PRIO,     
						 (CPU_STK   * )&Status_Check_TASK_STK[0],	
						 (CPU_STK_SIZE)Status_Check_STK_SIZE/10,	
						 (CPU_STK_SIZE)Status_Check_STK_SIZE,		
						 (OS_MSG_QTY  )0,					
						 (OS_TICK	  )0,					
						 (void   	* )0,					
						 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR,
						 (OS_ERR 	* )&err);
}

//ADS传感器使能
void Status_Check_task(void *p_arg)
{  
	
}





