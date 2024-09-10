/***************************************************

*�ļ�����:
*         EMG�źŲ���
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

#define ADNUM	100								// ������������Щ����������ƽ��ֵ������>2 
#define ADCHANNEL 1             //��ͨ��ɨ�迪����ͨ����
#define size 6000               //�����С  
#define N 12                    //����ƽ���˲�����ƽ��ֵʱ��ȡ�ĵ���  

volatile uint16_t ADCConvertedValue[ADNUM][ADCHANNEL];  // ADC�������� 
volatile uint8_t g_AdcDmaOk = 0;				                // ADCʹ�õ�DMA���ݴ�����ɱ�־��1-DMA�������
volatile uint16_t g_AdcFilteredValue[ADCHANNEL];		    // ���˲����ADC��������
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
	RCC_ADCCLKConfig(RCC_PCLK2_Div6);											// ADCʱ��6��Ƶ 72/6=12MHz<14MHz
	
	    //************ ����GPIO ************//	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2; //PA0,1,2ģ������,ADC_IN0,1,2,PA0���ڵ�ص�ѹ���
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;	
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

    //************ ����ADC ʹ��DMA ������� ************//	
	ADC_DeInit(ADC1);
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;							        // ADC1�����ڶ���ģʽ
  ADC_InitStructure.ADC_ScanConvMode = ENABLE;								            // ģ��ת��������ɨ��ģʽ����ͨ����
  ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;							        // ģ��ת������������ת��ģʽ
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;			// ת��������������ⲿ��������
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;			            // ADC�����Ҷ���
  ADC_InitStructure.ADC_NbrOfChannel = ADCHANNEL;							            // ˳����й���ת����ADCͨ������Ŀ
	
	ADC_Init(ADC1, &ADC_InitStructure);		// ����ADC_InitStructure��ָ���Ĳ�����ʼ������ADC1�ļĴ���
	                                      // ��ת��ʱ��=����ʱ��+12.5����=71.5+12.5=84*3����=21us=47.62kHz
																				// ������=47.62kHz/68=700Hz
	ADC_RegularChannelConfig(ADC1,ADC_Channel_0, 1, ADC_SampleTime_239Cycles5);	// ����ָ��ADC�Ĺ�����ͨ�����������ǵ�ת��˳��Ͳ���ʱ�䣨ȡ���ֵ��
		
	ADC_DMACmd(ADC1, ENABLE);													// ʹ��ADC1��DMA����	
	ADC_Cmd(ADC1, ENABLE);														// ʹ��ADC1
	ADC_ClearFlag(ADC1, ADC_FLAG_EOC);                //���״̬λ
	
	// ADC��У׼������������һ�α�֤����
  ADC_ResetCalibration(ADC1);											  // ����ADC1��У׼�Ĵ���
  while(ADC_GetResetCalibrationStatus(ADC1));				// ��ȡADC����У׼�Ĵ�����״̬���ȴ���ֵ��Ϊ0��У׼�Ĵ����ѳ�ʼ����
 
	ADC_StartCalibration(ADC1);											  // ��ʼADC1��У׼״̬
  while(ADC_GetCalibrationStatus(ADC1));						// ��ȡADCУ׼�Ĵ�����״̬���ȴ���ֵ��Ϊ0�������У׼��
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);						// ʹ��ADC1�����ת����������
	
		    //************ ����DMA ************//
	DMA_DeInit(DMA1_Channel1);

	DMA_InitStructure.DMA_PeripheralBaseAddr = ADC1_DR_Address;					  // DMA��Ӧ����Ļ���ַ��AD�������ݵĴ�ŵ�ַ
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)&ADCConvertedValue;	// �������ݴ洢��
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;							      // DMA��ת��ģʽ��SRCģʽ�����Ǵ��������ڴ��а���
	DMA_InitStructure.DMA_BufferSize = ADNUM*ADCHANNEL;									  // DMA�����С
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;			// ����һ�����ݺ��豸��ַ�Ƿ����
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;						    // ����һ�����ݺ�Ŀ���ڴ��ַ�Ƿ����
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;	// ת����������ݴ�С
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;		// DMA���˵����ݳߴ�,16λ
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;								        // ����ģʽ��ѭ������ģʽ�����ã�M2M�����ˣ����ģʽʧЧ
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;							      // DMA���ȼ�����
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;								          // M2Mģʽ��ֹ
	DMA_Init(DMA1_Channel1, &DMA_InitStructure);								          // ��ʼ��DMA1_CH1

	DMA_ITConfig(DMA1_Channel1, DMA_IT_TC, ENABLE);								        // ʹ��DMA1_CH1���ݴ�������ж�

	DMA_Cmd(DMA1_Channel1, ENABLE);												                // ʹ��DMA1_CH1
	
  NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
	//����ѭ������
	EmgSqAddr = initQueue();
}

/*
*********************************************************************************************************
*	�� �� ��: ADsampleFilter
*	����˵��: ADC��������ȥ��һ�����ֵ��һ����Сֵ��ƽ��ֵ
*	��    ��: ��
*	�� �� ֵ: ADC����ֵ
*********************************************************************************************************
*/
static uint16_t ADsampleFilter(__IO uint16_t *_ADCConvertedValue)
{
    uint32_t result=0;
    uint16_t i;
    uint16_t min,max;
	
		min = _ADCConvertedValue[0];
		max = min;
		
			//�ҳ����������ֵ����Сֵ
		for(i=1; i<ADNUM; i++)
		{
			if(_ADCConvertedValue[i] < min)		// �ҳ���Сֵ
			{
				min = _ADCConvertedValue[i];
			}
			if(_ADCConvertedValue[i] > max)		// �ҳ����ֵ
			{
				max = _ADCConvertedValue[i];
			}
		}
		
	// ��ƽ��ֵ
    for(i=0; i<ADNUM; i++)					
    {
        result += _ADCConvertedValue[i];
    }
  	result = result - min - max;
    return (uint16_t)(result/(ADNUM-2));
}

/*
*********************************************************************************************************
*	�� �� ��: Smooth
*	����˵��: ADC��������������ֵ�˲�
*	��    ��: ��
*	�� �� ֵ: ADC����ֵ
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
*	�� �� ��: AdcDmaIrq
*	����˵��: ADC1�õ�DMA��������жϷ�����
*	��    ��: ��
*	�� �� ֵ: ��
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
		g_AdcFilteredValue[0] = ADsampleFilter(ADC_Channel_0_Value);	// ��õ�ص�ѹ�˲���Ĳ�������
		DMA_ClearITPendingBit(DMA1_IT_TC1);
		g_AdcDmaOk = 1;											                    // ��־λ��λ
	}  
}

