/***************************************************

*文件描述:
*         STM32 模拟I2C总线驱动用于板载MPU9250通讯         
*Author:
*         LRK
*Time:
*					2017.11.12
*version:
*         v1.2
***************************************************/

#include "stm32_iic.h"
#define  SCL_H         GPIOB->BSRR = GPIO_Pin_6  
#define  SCL_L         GPIOB->BRR  = GPIO_Pin_6  
#define  SDA_H         GPIOB->BSRR = GPIO_Pin_7  
#define  SDA_L         GPIOB->BRR  = GPIO_Pin_7 
#define  SCL_read      GPIOB->IDR  & GPIO_Pin_6  
#define  SDA_read      GPIOB->IDR  & GPIO_Pin_7  

//IIC专用延时函数
static void I2C_delay(void)
{
    volatile int i = 10;
    while (i)
    i--;
}

//IIC接口初始化
void i2cInit(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
 
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
}
//产生一个起始位
bool I2C_Start(void)
{
    SDA_H;
    SCL_H;
    I2C_delay();
    if (!SDA_read)
        return false;
    SDA_L;
    I2C_delay();
    if (SDA_read)
        return false;
    SDA_L;
    I2C_delay();
    return true;
}

//产生一个停止位
void I2C_Stop(void)
{
    SCL_L;
    I2C_delay();
    SDA_L;
    I2C_delay();
    SCL_H;
    I2C_delay();
    SDA_H;
    I2C_delay();
}

//产生一个ACK信号
void I2C_Ack(void)
{
    SCL_L;
    I2C_delay();
    SDA_L;
    I2C_delay();
    SCL_H;
    I2C_delay();
    SCL_L;
    I2C_delay();
}

//产生NoACK信号
void I2C_NAck(void)
{
    SCL_L;
    I2C_delay();
    SDA_H;
    I2C_delay();
    SCL_H;
    I2C_delay();
    SCL_L;
    I2C_delay();
}

//产生ACK等待信号
bool I2C_WaitAck(void)
{
    SCL_L;
    I2C_delay();
    SDA_H;
    I2C_delay();
    SCL_H;
    I2C_delay();
    if (SDA_read) 
    {
        SCL_L;
        return false;
    }
    SCL_L;
    return true;
}

//发送一个字节
void I2C_SendByte(uint8_t byte)
{
    uint8_t i = 8;
    while (i--) 
    {
        SCL_L;
        I2C_delay();
        if (byte & 0x80)
            SDA_H;
        else
            SDA_L;
        byte <<= 1;
        I2C_delay();
        SCL_H;
        I2C_delay();
    }
    SCL_L;
}

//接收一个字节
uint8_t I2C_ReceiveByte(void)
{
    uint8_t i = 8;
    uint8_t byte = 0;

    SDA_H;
    while (i--)
    {
        byte <<= 1;
        SCL_L;
        I2C_delay();
        SCL_H;
        I2C_delay();
        if (SDA_read) 
        {
            byte |= 0x01;
        }
    }
    SCL_L;
    return byte;
}

//发送字符串
bool i2cWriteBuffer(uint8_t addr, uint8_t reg, uint8_t len, uint8_t * data)
{
    int i;
    if (!I2C_Start())
        return false;
    I2C_SendByte(addr << 1 | I2C_Direction_Transmitter);
    if (!I2C_WaitAck()) 
    {
        I2C_Stop();
        return false;
    }
    I2C_SendByte(reg);
    I2C_WaitAck();
    for (i = 0; i < len; i++) 
    {
        I2C_SendByte(data[i]);
        if (!I2C_WaitAck()) 
        {
            I2C_Stop();
            return false;
        }
    }
    I2C_Stop();
    return true;
}

bool i2cWrite(uint8_t addr, uint8_t reg, uint8_t data)
{
    if (!I2C_Start())
        return false;
    I2C_SendByte(addr << 1 | I2C_Direction_Transmitter);
    if (!I2C_WaitAck()) 
    {
        I2C_Stop();
        return false;
    }
    I2C_SendByte(reg);
    I2C_WaitAck();
    I2C_SendByte(data);
    I2C_WaitAck();
    I2C_Stop();
    return true;
}


bool i2cRead(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf)
{
    if (!I2C_Start())
        return false;
    I2C_SendByte(addr << 1 | I2C_Direction_Transmitter);
    if (!I2C_WaitAck()) 
    {
        I2C_Stop();
        return false;
    }
    I2C_SendByte(reg);
    I2C_WaitAck();
    I2C_Start();
    I2C_SendByte(addr << 1 | I2C_Direction_Receiver);
    I2C_WaitAck();
    while (len) 
    {
        *buf = I2C_ReceiveByte();
        if (len == 1)
            I2C_NAck();
        else
            I2C_Ack();
        buf++;
        len--;
    }
    I2C_Stop();
    return true;
}


uint16_t i2cGetErrorCounter(void)
{
    // TODO maybe fix this, but since this is test code, doesn't matter.
    return 0;
}




