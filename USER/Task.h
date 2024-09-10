#include "includes.h"
/********************************任务设置**********************************/
//BLED
#define BLED_Blink3_TASK_PRIO		7
#define BLED_Blink3_STK_SIZE 		128
extern OS_TCB BLED_Blink3_TaskTCB;
extern CPU_STK BLED_Blink3_TASK_STK[BLED_Blink3_STK_SIZE];
void BLED_Blink3_task(void *p_arg);
//YLED
#define YLED_Blink3_TASK_PRIO		7	
#define YLED_Blink3_STK_SIZE 		128
extern OS_TCB YLED_Blink3_TaskTCB;
extern CPU_STK YLED_Blink3_TASK_STK[YLED_Blink3_STK_SIZE];
void YLED_Blink3_task(void *p_arg);
//ADS传感器使能任务
#define ADS_Run_TASK_PRIO		6	
#define ADS_Run_STK_SIZE 		128
extern OS_TCB ADS_Run_TaskTCB;
extern CPU_STK ADS_Run_TASK_STK[ADS_Run_STK_SIZE];
void ADS_Run_task(void *p_arg);
//获取传感器数据任务
#define Get_MPU_Data_TASK_PRIO		5	
#define Get_MPU_Data_STK_SIZE 		256
extern OS_TCB Get_MPU_Data_TaskTCB;
extern CPU_STK Get_MPU_Data_TASK_STK[Get_MPU_Data_STK_SIZE];
void Get_MPU_Data_task(void *p_arg);
//数据上报
#define Data_Report_TASK_PRIO		4	
#define Data_Report_STK_SIZE 		256
extern OS_TCB Data_Report_TaskTCB;
extern CPU_STK Data_Report_TASK_STK[Data_Report_STK_SIZE];
void Data_Report_task(void *p_arg);
//状态检查
#define Status_Check_TASK_PRIO		8	
#define Status_Check_STK_SIZE 		128
extern OS_TCB Status_Check_TaskTCB;
extern CPU_STK Status_Check_TASK_STK[Status_Check_STK_SIZE];
void Status_Check_task(void *p_arg);

void Create_LED_Task(void);
void Create_Get_MPU_Data_Task(void *buf);
void Create_ADS_Start_Task(void);
void Create_Data_Report_Task(void *buf);
void Create_Stasus_Check_Task(void);

/********************************事件标志**********************************/
void Creat_EventFlag(void);

extern OS_FLAG_GRP MeasureEvent;        //测量数据事件
#define MeasureFlag   0x01





