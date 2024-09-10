/***************************************************

*文件描述:
*         EMG信号采样
*Author:
*         LRK
*Time:
*					2018.5.16
*version:
*         v1.0
***************************************************/

#include "EMG_Signal.h"
#include "stm32f10x_adc.h"
#include "stm32f10x_dma.h"
#include "Queue.h"

#define ADC1_DR_Address  ((uint32_t)0x4001244C)	// STM32F1XX  ADC1_DR

#define ADNUM	100								// 样本个数，这些采样数据求平均值，必须>2 
#define ADCHANNEL 1             //多通道扫描开启的通道数
#define size 6000               //数组大小  
#define N 12                    //滑动平均滤波计算平均值时所取的点数  

volatile uint16_t ADCConvertedValue[ADNUM][ADCHANNEL];  // ADC采样数据 
volatile uint8_t g_AdcDmaOk = 0;				                // ADC使用的DMA数据传输完成标志，1-DMA传输完成
volatile uint16_t g_AdcFilteredValue[ADCHANNEL];		    // 简单滤波后的ADC采样数据
float Sum1;
extern unsigned short count_id;
extern int count;
SqQueue *EmgSqAddr;
#define  led_on    GPIO_ResetBits(GPIOB, GPIO_Pin_13)
#define  led_off   GPIO_SetBits(GPIOB, GPIO_Pin_13)

void AdcDmaInit(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	DMA_InitTypeDef  DMA_InitStructure;
	ADC_InitTypeDef ADC_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_GPIOA, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	RCC_ADCCLKConfig(RCC_PCLK2_Div6);											// ADC时钟6分频 72/6=12MHz<14MHz
	
	    //************ 配置GPIO ************//	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2; //PA0,1,2模拟输入,ADC_IN0,1,2,PA0用于电池电压检测
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;	
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

    //************ 配置ADC 使用DMA 软件触发 ************//	
	ADC_DeInit(ADC1);
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;							        // ADC1工作在独立模式
  ADC_InitStructure.ADC_ScanConvMode = ENABLE;								            // 模数转换工作在扫描模式（多通道）
  ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;							        // 模数转换工作在连续转换模式
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;			// 转换由软件而不是外部触发启动
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;			            // ADC数据右对齐
  ADC_InitStructure.ADC_NbrOfChannel = ADCHANNEL;							            // 顺序进行规则转换的ADC通道的数目
	
	ADC_Init(ADC1, &ADC_InitStructure);		// 根据ADC_InitStructure中指定的参数初始化外设ADC1的寄存器
	                                      // 总转换时间=采样时间+12.5周期=71.5+12.5=84*3周期=21us=47.62kHz
																				// 采样率=47.62kHz/68=700Hz
	ADC_RegularChannelConfig(ADC1,ADC_Channel_0, 1, ADC_SampleTime_239Cycles5);	// 设置指定ADC的规则组通道，设置它们的转化顺序和采样时间（取最大值）
		
	ADC_DMACmd(ADC1, ENABLE);													// 使能ADC1的DMA请求	
	ADC_Cmd(ADC1, ENABLE);														// 使能ADC1
	ADC_ClearFlag(ADC1, ADC_FLAG_EOC);                //清除状态位
	
	// ADC自校准，开机后运行一次保证精度
  ADC_ResetCalibration(ADC1);											  // 重置ADC1的校准寄存器
  while(ADC_GetResetCalibrationStatus(ADC1));				// 获取ADC重置校准寄存器的状态并等待其值变为0（校准寄存器已初始化）
 
	ADC_StartCalibration(ADC1);											  // 开始ADC1的校准状态
  while(ADC_GetCalibrationStatus(ADC1));						// 获取ADC校准寄存器的状态并等待其值变为0（已完成校准）
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);						// 使能ADC1的软件转换启动功能
	
		    //************ 配置DMA ************//
	DMA_DeInit(DMA1_Channel1);

	DMA_InitStructure.DMA_PeripheralBaseAddr = ADC1_DR_Address;					  // DMA对应外设的基地址，AD采样数据的存放地址
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)&ADCConvertedValue;	// 接收数据存储区
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;							      // DMA的转换模式是SRC模式，就是从外设向内存中搬运
	DMA_InitStructure.DMA_BufferSize = ADNUM*ADCHANNEL;									  // DMA缓存大小
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;			// 接收一次数据后，设备地址是否后移
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;						    // 接收一次数据后，目标内存地址是否后移
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;	// 转换结果的数据大小
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;		// DMA搬运的数据尺寸,16位
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;								        // 传输模式，循环缓存模式，常用，M2M开启了，这个模式失效
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;							      // DMA优先级，高
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;								          // M2M模式禁止
	DMA_Init(DMA1_Channel1, &DMA_InitStructure);								          // 初始化DMA1_CH1

	DMA_ITConfig(DMA1_Channel1, DMA_IT_TC, ENABLE);								        // 使能DMA1_CH1数据传输完成中断

	DMA_Cmd(DMA1_Channel1, ENABLE);												                // 使能DMA1_CH1
	
  NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
	//创建循环队列
	EmgSqAddr = initQueue();
}

/*
*********************************************************************************************************
*	函 数 名: ADsampleFilter
*	功能说明: ADC采样处理，去掉一个最大值和一个最小值求平均值
*	形    参: 无
*	返 回 值: ADC采样值
*********************************************************************************************************
*/
static uint16_t ADsampleFilter(__IO uint16_t *_ADCConvertedValue)
{
    uint32_t result=0;
    uint16_t i;
    uint16_t min,max;
	
		min = _ADCConvertedValue[0];
		max = min;
		
			//找出数组中最大值和最小值
		for(i=1; i<ADNUM; i++)
		{
			if(_ADCConvertedValue[i] < min)		// 找出最小值
			{
				min = _ADCConvertedValue[i];
			}
			if(_ADCConvertedValue[i] > max)		// 找出最大值
			{
				max = _ADCConvertedValue[i];
			}
		}
		
	// 求平均值
    for(i=0; i<ADNUM; i++)					
    {
        result += _ADCConvertedValue[i];
    }
  	result = result - min - max;
    return (uint16_t)(result/(ADNUM-2));
}

/*
*********************************************************************************************************
*	函 数 名: Smooth
*	功能说明: ADC采样处理，滑动均值滤波
*	形    参: 无
*	返 回 值: ADC采样值
*********************************************************************************************************
*/
void Smooth(float data[])  
{  
    Sum1=0;  
    for(int j=0;j<size;j++)  
    {  
        if(j<N/2)  
        {  
            for(int k=0;k<N;k++)  
            {  
                Sum1+=data[j+k];  
            }  
            data[j]=Sum1/N;  
        }  
        else  
            if(j<size -N/2)  
            {  
                for(int k=0;k<N/2;k++)  
                {  
                    Sum1+=(data[j+k]+data[j-k]);  
                }  
                data[j]=Sum1/N;  
            }  
            else  
            {  
                for(int k=0;k<size-j;k++)  
                {  
                    Sum1+=data[j+k];  
                }  
                for(int k=0;k<(N-size+j);k++)  
                {  
                    Sum1+=data[j-k];  
                }  
                data[j]=Sum1/N;  
            }  
        Sum1=0;  
    }  
}  
/*
*********************************************************************************************************
*	函 数 名: AdcDmaIrq
*	功能说明: ADC1用的DMA传输完成中断服务函数
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void AdcDmaIrq(void)
{
	uint16_t ADC_Channel_0_Value[ADNUM];
//	uint16_t ADC_Channel_1_Value[ADNUM];
//	uint16_t ADC_Channel_2_Value[ADNUM];
	if(DMA_GetITStatus(DMA1_IT_TC1) != RESET)
	{ 
		for(int i=0;i<ADNUM;i++)
		{
			ADC_Channel_0_Value[i]=ADCConvertedValue[i][0];

		}
		g_AdcFilteredValue[0] = ADsampleFilter(ADC_Channel_0_Value);	// 获得电池电压滤波后的采样数据
		DMA_ClearITPendingBit(DMA1_IT_TC1);
		g_AdcDmaOk = 1;											                    // 标志位置位
	}  
}

