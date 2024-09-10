
/***************************************************
ads_hal_i2c.c
*文件描述:
*         ADS传感器底层驱动
*Author:
*         刘镕恺
*Time:
*					2022.3.26
*version:
*         v1.0
***************************************************/

/****************************************************************/
/*                     Board V2.0 Pin Assignment                */
/****************************************************************/
/*FLEX_1: interrupt_pin-->GPIOA_Pin4   reset_pin-->GPIOA_Pin5   */
/*FLEX_2: interrupt_pin-->GPIOA_Pin6   reset_pin-->GPIOA_Pin7   */
/*FLEX_3: interrupt_pin-->GPIOB_Pin12  reset_pin-->GPIOB_Pin13  */
/*FLEX_4: interrupt_pin-->GPIOA_Pin15  reset_pin-->GPIOB_Pin3   */
/*FLEX_5: interrupt_pin-->GPIOB_Pin5   reset_pin-->GPIOB_Pin4   */
/*FLEX_6: interrupt_pin-->GPIOB_Pin8   reset_pin-->GPIOB_Pin9   */
/****************************************************************/

#include "ads_hal.h"
#include "ads.h"

/* Hardware Specific Includes */
#include "stm32f10x.h"
#include "stm32f10x_exti.h"
#include "stm32f10x_i2c.h"
#include "LED.h"
#include "delay.h"
#include "stdio.h"
#include "string.h"

static void (*ads_read_callback)(uint8_t *,uint8_t);

static uint8_t read_buffer[ADS_TRANSFER_SIZE];

#define ADS_DEFAULT_ADDR		(0x12)							// Default I2C address of the ADS one axis sensor
#define ADS_DEFAULT_DEVICE	(0x01)							// Default ADS sensor
#define ADS_IIC_TIMEOUT     (5000)

static uint32_t ADS_RESET_PIN = 0;
static uint32_t ADS_INTERRUPT_PIN = 0;
static GPIO_TypeDef *ADS_RESET_GPIO = GPIOA;
static GPIO_TypeDef *ADS_INTERRUPT_GPIO = GPIOA;

u8 ads_addrs[6] = {0x11,0x12,0x13,0x14,0x15,0x16};
u8 ads_devices[6] = {0x01,0x02,0x03,0x04,0x05,0x06};

static uint8_t _address = ADS_DEFAULT_ADDR;
static uint8_t _devices = ADS_DEFAULT_DEVICE;

volatile BOOL _ads_int_enabled = FALSE;

u16 timeout=ADS_IIC_TIMEOUT;

/************************************************************************/
/*                        HAL Stub Functions                            */
/************************************************************************/

static void ads_hal_pin_int_init(void);
static void ads_hal_i2c_init(void);

/**
 * @brief ADS data ready interrupt. Reads out packet from ADS and fires 
 *  		callback in ads.c
 */
void ads_hal_interrupt(u8 device)
{
	uint8_t len=0;
	switch(device)
		{
		case 0x01: 
			len=3;break;
		case 0x02: 
			len=3;break;
		case 0x03: 
			len=5;break;
		case 0x04: 
			len=5;break;
		case 0x05: 
			len=3;break;
		case 0x06: 
			len=3;break;
	}
	if(ads_hal_read_buffer(read_buffer, len) == ADS_OK)
	{
		switch(device)
		{
		  case 0x01: 
				ads_read_callback(read_buffer,device);break;
			case 0x02: 
				ads_read_callback(read_buffer,device);break;
			case 0x03: 
				read_buffer[0]=0x04; 
			  ads_read_callback(read_buffer,device);break;
			case 0x04: 
				read_buffer[0]=0x04; 
			  ads_read_callback(read_buffer,device);break;
			case 0x05: 
				ads_read_callback(read_buffer,device);break;
			case 0x06: 
				ads_read_callback(read_buffer,device);break;
		}
	}
}

/**
 * @brief Initializes the pin ADS_INTERRUPT_PIN as a falling edge pin change interrupt.
 *			Assign the interrupt service routine as ads_hal_interrupt. Enable pullup
 *			Enable interrupt	
 */
static void ads_hal_pin_int_init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure; 
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);//使能GPIOA
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4|GPIO_Pin_6|GPIO_Pin_15;	       							
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 								//上拉输入
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);												//初始化IO
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource4);		//中断线连接GPIOA
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource6);		//中断线连接GPIOA
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource15);	//中断线连接GPIOA
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);//使能GPIOB
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5|GPIO_Pin_8|GPIO_Pin_12;	       							
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 								//上拉输入
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);												//初始化IO
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource5);		//中断线连接GPIOB
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource8);		//中断线连接GPIOB
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource12);		//中断线连接GPIOB

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);//使能复用功能时钟
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	NVIC_InitStructure.NVIC_IRQChannel = EXTI4_IRQn; 							//使能中断向量表位置
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; 		//先占优先级3级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1; 						//从优先级3级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 							//使能外部中断通道
	NVIC_Init(&NVIC_InitStructure); 															//根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器
	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn; 						//使能中断向量表位置
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; 		//先占优先级3级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2; 						//从优先级3级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 							//使能外部中断通道
	NVIC_Init(&NVIC_InitStructure); 															//根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器
	NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn; 					//使能中断向量表位置
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; 		//先占优先级3级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3; 						//从优先级3级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 							//使能外部中断通道
	NVIC_Init(&NVIC_InitStructure);
}

/**
 * @brief Initializes the pin RESET_PIN as output pull up mode.	
 */
static void ads_hal_pin_reset_init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;  	
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB, ENABLE);//使能GPIOA和GPIOB时钟
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5|GPIO_Pin_7;	     
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 						  //推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);												//初始化IO
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_9|GPIO_Pin_13;	     
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 						  //推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);												//初始化IO
}


/**
 * @brief Wait for ADS sensor to release bus
 */
void I2C_EE_WaitADSStandbyState(u8 device_addr)      
{
  do
  {
    I2C_GenerateSTART(I2C2, ENABLE);
    while(I2C_GetFlagStatus(I2C2,I2C_FLAG_SB)==ERROR);//检测EV5事件
		I2C_Send7bitAddress(I2C2,device_addr << 1,I2C_Direction_Transmitter);
  }while(I2C_GetFlagStatus(I2C2,I2C_FLAG_ADDR)==ERROR);//
    I2C_ClearFlag(I2C2, I2C_FLAG_AF);//清楚标志位
    I2C_GenerateSTOP(I2C2, ENABLE);  //结束信号
}

/**
 * @brief Send a command to IIC slave 
 * @parm: addr: ADS sensor slave device address
 *        reg: register address
 *        data: byte data pointer 
 *        numToWrite: the number of bytes to write
 */
BOOL IIC2_Write(u8 addr,u8 reg,u8 *data,u8 numToWrite)
{
	while(I2C_GetFlagStatus(I2C2,I2C_FLAG_BUSY)==SET)
	{
		if((timeout--)==0)
		{
			I2C2_CLEAR_BUSY();
		}
	}timeout=ADS_IIC_TIMEOUT;
	
	I2C_GenerateSTART(I2C2,ENABLE);                               //发送起始信号
	while(!I2C_CheckEvent(I2C2,I2C_EVENT_MASTER_MODE_SELECT))     //检测EV5事件
	{
		if((timeout--)==0)
		{
#ifdef DEBUG
			printf("(WRITE_ERROR_01) ADS IIC write EV5 Fail\n");
#elif RELEASE
			
#endif
		}
	}	timeout=ADS_IIC_TIMEOUT;
	
	I2C_Send7bitAddress(I2C2,addr << 1,I2C_Direction_Transmitter);//发送7位EEPROM的硬件地址
	while(!I2C_CheckEvent(I2C2,I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))//检测EV6事件
	{
		if((timeout--)==0)
		{
#ifdef DEBUG
			printf("(WRITE_ERROR_02) ADS IIC write EV6 Fail\n");
#elif RELEASE
			
#endif
		}
	}	timeout=ADS_IIC_TIMEOUT;
	
	I2C_SendData(I2C2,reg);                                       //发送操作的内存地址
	while(!I2C_CheckEvent(I2C2,I2C_EVENT_MASTER_BYTE_TRANSMITTED))//检测EV8事件
	{
		if((timeout--)==0)
		{
#ifdef DEBUG
			printf("(WRITE_ERROR_03) ADS IIC write EV8 Fail\n");
#elif RELEASE
			
#endif
		}
	}	timeout=ADS_IIC_TIMEOUT;	
	
	while(numToWrite)
	{
		 I2C_SendData(I2C2,*data); 
		 data++;
		 numToWrite--;
		 while(!I2C_CheckEvent(I2C2,I2C_EVENT_MASTER_BYTE_TRANSMITTED))//检测EV8事件
	   {
		  if((timeout--)==0)
			{
#ifdef DEBUG
				printf("(WRITE_ERROR_04) ADS IIC write EV8 Fail\n");
#elif RELEASE
			
#endif
			}
	   }timeout=ADS_IIC_TIMEOUT;
	}
	
	I2C_GenerateSTOP(I2C2,ENABLE);//发送结束信号
	return TRUE;
}

/**
 * @brief Read data from ADS sensor
 * @parm: addr: ADS sensor slave device address
 *        reg: register address
 *        data: byte data pointer 
 *        numToWrite: the number of bytes to red
 */
BOOL IIC2_Read(u8 addr,u8 reg, u8 *buf,u8 numToRead)
{
	u8 readtemp;

	while(I2C_GetFlagStatus(I2C2,I2C_FLAG_BUSY)==SET)
	{
		if((timeout--)==0)
		{
			I2C2_CLEAR_BUSY();
		}
	}timeout=ADS_IIC_TIMEOUT;
		
	I2C_GenerateSTART(I2C2,ENABLE);
	while(!I2C_CheckEvent(I2C2,I2C_EVENT_MASTER_MODE_SELECT))//检测	EV5事件
	{
		if((timeout--)==0)
		{
#ifdef DEBUG
			printf("(READ_ERROR_01) ADS IIC read EV5 Fail\n");
#elif RELEASE
			
#endif
		}
	}	timeout=ADS_IIC_TIMEOUT;
	
	I2C_Send7bitAddress(I2C2,addr << 1,I2C_Direction_Transmitter);
	while(!I2C_CheckEvent(I2C2,I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))//检测	EV6 事件
	{
		if((timeout--)==0)
		{
#ifdef DEBUG
			printf("(READ_ERROR_02) ADS IIC read EV6 Fail\n");
#elif RELEASE
			
#endif
		}
	}	timeout=ADS_IIC_TIMEOUT;
	
	I2C_SendData(I2C2,reg);
  while(I2C_CheckEvent(I2C2,I2C_EVENT_MASTER_BYTE_TRANSMITTED) != SUCCESS )
  {
		if((timeout--)==0)
		{
#ifdef DEBUG
			printf("(READ_ERROR_03) ADS IIC read EV8 Fail\n");
#elif RELEASE
			
#endif
		}
	}	timeout=ADS_IIC_TIMEOUT;
	
	I2C_GenerateSTART(I2C2,ENABLE);
	while(!I2C_CheckEvent(I2C2,I2C_EVENT_MASTER_MODE_SELECT))
	{
		if((timeout--)==0)
		{
#ifdef DEBUG
			printf("(READ_ERROR_04) ADS IIC read EV5 Fail\n");
#elif RELEASE
			
#endif
		}
	}	timeout=ADS_IIC_TIMEOUT;
	I2C_Send7bitAddress(I2C2,addr << 1,I2C_Direction_Receiver);
	while(!I2C_CheckEvent(I2C2,I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED))
	{
		if((timeout--)==0)
		{
#ifdef DEBUG
			printf("(READ_ERROR_05) ADS IIC read EV6 Fail\n");
#elif RELEASE
			
#endif
		}
	}	timeout=ADS_IIC_TIMEOUT;
//	I2C_AcknowledgeConfig(I2C2,ENABLE); 
  /* 读取接收数据 */
	while(numToRead-1)
	{
		/* ACK */
		I2C_AcknowledgeConfig(I2C2,ENABLE);
		while(I2C_CheckEvent(I2C2,I2C_EVENT_MASTER_BYTE_RECEIVED) != SUCCESS )
    {
			if((timeout--)==0)
			{
#ifdef DEBUG
				printf("(READ_ERROR_06) ADS IIC read EV7 Fail\n");
#elif RELEASE
			
#endif
			}
	  }timeout=ADS_IIC_TIMEOUT;
		*buf = I2C_ReceiveData(I2C2);
		buf++;
		numToRead--;
	}
	/* NACK */
	I2C_AcknowledgeConfig(I2C2,DISABLE);
	/* 停止信号 */
	I2C_GenerateSTOP(I2C2,ENABLE);
	while(I2C_CheckEvent(I2C2,I2C_EVENT_MASTER_BYTE_RECEIVED) != SUCCESS )
  {
		if((timeout--)==0)
		{
#ifdef DEBUG
			printf("(READ_ERROR_06) ADS IIC read EV7 Fail\n");
#elif RELEASE
			
#endif
		}
	}timeout=ADS_IIC_TIMEOUT;
	*buf = I2C_ReceiveData(I2C2);
	
  return TRUE;
}

/**
 * @brief Millisecond delay routine.
 */
void ads_hal_delay(uint16_t ms)
{
	Delay_ms(ms);
}

/**
 * @brief Enable/Disable the pin change data ready interrupt
 *
 * @param enable		true = enable, false = disable
 */
void ads_hal_pin_int_enable(BOOL enable)
{
	// Copy enable to local variable to store enabled state of pin change interrupt
	_ads_int_enabled = enable;
	EXTI_InitTypeDef EXTI_InitStructure;
	// Enable/Disable the pin change data ready interrupt
	if(enable)
	{
		EXTI_InitStructure.EXTI_Line = EXTI_Line4|EXTI_Line5|EXTI_Line6|EXTI_Line8|EXTI_Line12|EXTI_Line15;									  //设置引脚所有的外部线路
		EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;						//设外外部中断模式:EXTI线路为中断请求
		EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;  			//下降沿触发
		EXTI_InitStructure.EXTI_LineCmd = ENABLE;										  //中断使能
		EXTI_Init(&EXTI_InitStructure);																//初始化外部中断		
	}
	else
	{
		EXTI_InitStructure.EXTI_Line = EXTI_Line4|EXTI_Line5|EXTI_Line6|EXTI_Line8|EXTI_Line12|EXTI_Line15;									  //设置引脚所有的外部线路
		EXTI_InitStructure.EXTI_LineCmd = DISABLE;										//中断使能
		EXTI_Init(&EXTI_InitStructure);																//初始化外部中断
	}
}

/**
 * @brief Configure I2C bus, 7 bit address, 400kHz frequency enable clock stretching
 *			if available.
 */

static void ads_hal_i2c_init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	I2C_InitTypeDef I2C_InitStruct;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	//GPIO的配置
	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_AF_OD;
	GPIO_InitStruct.GPIO_Pin=GPIO_Pin_10|GPIO_Pin_11;
	GPIO_InitStruct.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&GPIO_InitStruct);
	
	if(I2C_GetFlagStatus(I2C2,I2C_FLAG_BUSY)==SET)
	{
			I2C2_CLEAR_BUSY();
	}
	I2C_InitStruct.I2C_Ack=I2C_Ack_Enable;
	I2C_InitStruct.I2C_AcknowledgedAddress=I2C_AcknowledgedAddress_7bit;
	I2C_InitStruct.I2C_ClockSpeed=400000;
	I2C_InitStruct.I2C_DutyCycle=I2C_DutyCycle_2;
	I2C_InitStruct.I2C_Mode=I2C_Mode_I2C;
	I2C_InitStruct.I2C_OwnAddress1=0x55;
	I2C_Init(I2C2,&I2C_InitStruct);
	
	I2C_ClearFlag(I2C2, I2C_FLAG_AF);
	I2C_Cmd(I2C2,ENABLE);
}

/**
 * @brief Clear the BUSY bit in case of IIC bus lock to resume operation
 */

#define  SCL_H         GPIOB->BSRR = GPIO_Pin_10  
#define  SCL_L         GPIOB->BRR  = GPIO_Pin_10 
void I2C2_CLEAR_BUSY(void)
{
	//1. Clear PE bit
	I2C2->CR1 &= ~(0x0001);
	
	//2. Configure the SCL and SDA I/Os as General Purpose Output Open-Drain, High level (Write 1 to GPIOx_ODR).
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_Out_OD;
	GPIO_InitStruct.GPIO_Pin=GPIO_Pin_10|GPIO_Pin_11;
	GPIO_InitStruct.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&GPIO_InitStruct);
	
	GPIO_SetBits(GPIOB,GPIO_Pin_10);
	GPIO_SetBits(GPIOB,GPIO_Pin_11);
	
	//3. Check SCL and SDA High level in GPIOx_IDR.
	while(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_10)==0);
	while(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_11)==0)
	{
		SCL_L;
    Delay_us(20);		
		SCL_H;
	  Delay_us(20);		
	}
	//4. Configure the SDA I/O as General Purpose Output Open-Drain, Low level (Write 0 to GPIOx_ODR).
	GPIO_ResetBits(GPIOB,GPIO_Pin_10);
	//5. Check SDA Low level in GPIOx_IDR.
	while(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_10)==1){GPIO_ResetBits(GPIOB,GPIO_Pin_10);}
	//6. Configure the SCL I/O as General Purpose Output Open-Drain, Low level (Write 0 to GPIOx_ODR).
	GPIO_ResetBits(GPIOB,GPIO_Pin_11);
	//7. Check SCL Low level in GPIOx_IDR.
	while(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_11)==1);
	// 8. Configure the SCL I/O as General Purpose Output Open-Drain, High level (Write 1 to GPIOx_ODR).
	GPIO_SetBits(GPIOB,GPIO_Pin_10);
	// 9. Check SCL High level in GPIOx_IDR.
	while(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_10)==0);
	// 10. Configure the SDA I/O as General Purpose Output Open-Drain , High level (Write 1 to GPIOx_ODR).
	GPIO_SetBits(GPIOB,GPIO_Pin_11);
	// 11. Check SDA High level in GPIOx_IDR.
	while(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_11)==0);
	// 12. Configure the SCL and SDA I/Os as Alternate function Open-Drain.	
	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_AF_OD;
	GPIO_InitStruct.GPIO_Pin=GPIO_Pin_10|GPIO_Pin_11;
	GPIO_InitStruct.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&GPIO_InitStruct);
	//13. Set SWRST bit in I2Cx_CR1 register.
	I2C2->CR1 |= I2C_CR1_SWRST;
	//14. Clear SWRST bit in I2Cx_CR1 register.
	I2C2->CR1 &= ~I2C_CR1_SWRST;
	//15. Enable the I2C peripheral by setting the PE bit in I2Cx_CR1 register
	I2C2->CR1 |= (0x0001);
	
	I2C_InitTypeDef I2C_InitStruct;
	I2C_InitStruct.I2C_Ack=I2C_Ack_Enable;
	I2C_InitStruct.I2C_AcknowledgedAddress=I2C_AcknowledgedAddress_7bit;
	I2C_InitStruct.I2C_ClockSpeed=400000;
	I2C_InitStruct.I2C_DutyCycle=I2C_DutyCycle_2;
	I2C_InitStruct.I2C_Mode=I2C_Mode_I2C;
	I2C_InitStruct.I2C_OwnAddress1=0x55;
	I2C_Init(I2C2,&I2C_InitStruct);
	I2C_ClearFlag(I2C2, I2C_FLAG_AF);
	I2C_Cmd(I2C2,ENABLE);
	
	if (I2C_GetFlagStatus(I2C2,I2C_FLAG_BUSY)==SET)
	{
#ifdef DEBUG
			printf("Reset IIC ERROR!\n");
#elif RELEASE
			
#endif
	}
}

/**
 * @brief Write buffer of data to the Angular Displacement Sensor
 *
 * @param buffer[in]	Write buffer
 * @param len			Length of buffer.
 * @return	ADS_OK if successful ADS_ERR_IO if failed
 */
int ads_hal_write_buffer(uint8_t * buffer, uint8_t len)
{
	BOOL STATE;	
	u8 reg_addr = buffer[0];
	
	// Disable the interrupt, if interrupt is enabled
	if(_ads_int_enabled)
	{
    ads_hal_pin_int_enable(FALSE);
	}
	
	// Write the the buffer to the ADS sensor
	if(len>1)
	{
		u8 *data = buffer+1;
		STATE = IIC2_Write(_address,reg_addr,data,len-1);
	}
	else
	{
	 STATE = IIC2_Write(_address,reg_addr,0,0);
  }
	
	// Re-enable the interrupt, if the interrupt was disabled
	if(!_ads_int_enabled)
	{
    ads_hal_pin_int_enable(TRUE); 		
		// Read data packet if interrupt was missed
//		if(GPIO_ReadInputDataBit(GPIOA, ADS_INTERRUPT_PIN) == 0)
//		{
//			if(ads_hal_read_buffer(read_buffer, ADS_TRANSFER_SIZE) == ADS_OK)
//			{
//				ads_read_callback(read_buffer,_devices);
//			}
//		}
	}
	if(STATE)
		return ADS_OK;
	else
		return ADS_ERR_IO;
}

/**
 * @brief Read buffer of data from the Angular Displacement Sensor
 *
 * @param buffer[out]	Read buffer
 * @param len			Length of data to read in number of bytes.
 * @return	ADS_OK if successful ADS_ERR_IO if failed
 */
int ads_hal_read_buffer(uint8_t * buffer, uint8_t len)
{
	int ret_val = IIC2_Read(_address, _address, buffer, len);
	
	if(ret_val)
		return ADS_OK;
	else
		return ADS_ERR_IO;
}

/**
 * @brief Reset the Angular Displacement Sensor
 */
void ads_hal_reset(void)
{
	// Bring reset low for 10ms then release
	GPIO_ResetBits(ADS_RESET_GPIO, ADS_RESET_PIN);
	ads_hal_delay(100);
	GPIO_SetBits(ADS_RESET_GPIO, ADS_RESET_PIN);
}\

/**
 * @brief Initializes the hardware abstraction layer 
 *
 * @return	ADS_OK if successful ADS_ERR_IO if failed
 */
int ads_hal_init(void (*callback)(uint8_t*,uint8_t), uint32_t reset_pin, uint32_t datardy_pin)
{
	// Copy pin numbers for reset and data ready to local variables
	ADS_RESET_PIN     = reset_pin;
	ADS_INTERRUPT_PIN = datardy_pin;
	
	// Set callback pointer
	ads_read_callback = callback;
	
	// Reset the ads
	ads_hal_reset();
	
	// Wait for ads to initialize
	ads_hal_delay(2000);
	
	// Configure and enable interrupt pin
	ads_hal_pin_int_init();
	
	// Initialize the I2C bus
	ads_hal_i2c_init();

	return ADS_OK;
}

BOOL ADS_change_adress(u8 old_address,u8 new_address)
{
	ads_hal_i2c_init();

	int ret_val = IIC2_Write(old_address,ADS_SET_ADDRESS,&new_address,1);
	
	if(ret_val == FALSE)
		return ADS_ERR_IO;
	
	ads_hal_set_address(new_address);
	
	return ADS_OK;
}

/**
 * @brief Gets the current i2c address that the hal layer is addressing. 	
 *				Used by device firmware update (dfu)
 * @return	uint8_t _address
 */
uint8_t ads_hal_get_address(void)
{
	return _address;
}

/**
 * @brief Sets the i2c address that the hal layer is addressing *	
 *				Used by device firmware update (dfu)
 *
 * @param address		i2c address hal to communicate with
 */
void ads_hal_set_address(uint8_t address)
{
	_address = address;
}

int ads_hal_select_device(uint8_t device)
{
	if(device <= 6)
	{
		switch(device)
		{
		  case 0x01: 
								_address = 0x11;
								_devices = 0x01;
				        ADS_INTERRUPT_GPIO=GPIOA;
   			        ADS_INTERRUPT_PIN=GPIO_Pin_4; 
			          ADS_RESET_GPIO=GPIOA;
			          ADS_RESET_PIN=GPIO_Pin_5;
								break;
			case 0x02: 
								_address = 0x12;
								_devices = 0x02;
								ADS_INTERRUPT_GPIO=GPIOA;
   			        ADS_INTERRUPT_PIN=GPIO_Pin_6; 
			          ADS_RESET_GPIO=GPIOA;
			          ADS_RESET_PIN=GPIO_Pin_7;
								break;
			case 0x03: 
								_address = 0x13;
								_devices = 0x03;
								ADS_INTERRUPT_GPIO=GPIOB;
   			        ADS_INTERRUPT_PIN=GPIO_Pin_12; 
			          ADS_RESET_GPIO=GPIOB;
			          ADS_RESET_PIN=GPIO_Pin_13;
								break;
			case 0x04:
								_address = 0x14;
								_devices = 0x04;
								ADS_INTERRUPT_GPIO=GPIOA;
   			        ADS_INTERRUPT_PIN=GPIO_Pin_15; 
			          ADS_RESET_GPIO=GPIOB;
			          ADS_RESET_PIN=GPIO_Pin_3;
								break;
			case 0x05: 
								_address = 0x15;
								_devices = 0x05;
								ADS_INTERRUPT_GPIO=GPIOB;
   			        ADS_INTERRUPT_PIN=GPIO_Pin_5; 
			          ADS_RESET_GPIO=GPIOB;
			          ADS_RESET_PIN=GPIO_Pin_4;
								break;
			case 0x06:
								_address = 0x16;
								_devices = 0x06;
								ADS_INTERRUPT_GPIO=GPIOB;
   			        ADS_INTERRUPT_PIN=GPIO_Pin_8; 
			          ADS_RESET_GPIO=GPIOB;
			          ADS_RESET_PIN=GPIO_Pin_9;
								break;
		}
	}
	else
		return ADS_ERR_BAD_PARAM;
		
	return ADS_OK;
}

