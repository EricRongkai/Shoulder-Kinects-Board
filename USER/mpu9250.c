/***************************************************

*�ļ�����:
*         MPU9250����
*         ԭʼ���ݶ�ȡ
*         У׼
*Author:
*         ���F��
*Time:
*					2018.5.12
*version:
*         v1.0
***************************************************/

#include "mpu9250.h"

/*====================================================================================================*/
#define MPU_CORRECTION_FLASH     0x0800E000       //�洢У�����ݵ�FLASH��ַ��SIZE=6*3*4�ֽ�
#define FLASH_ACCEL_OFFSET_ADDR  0X0800E000 		  //ģ��EEPROM����ʼ��ַ
#define FLASH_GYRO_OFFSET_ADDR   0X0800E008 		  //ģ��EEPROM����ʼ��ַ
#define FLASH_MAG_OFFSET_ADDR    0X0800E010 		  //ģ��EEPROM����ʼ��ַ
/*====================================================================================================*/
#define Accel_2G_Ref  16384
#define Accel_4G_Ref  8192
#define Accel_8G_Ref  4096
#define Accel_16G_Ref  2048

/*
|     |      ACCELEROMETER      |        GYROSCOPE        |
| LPF | BandW | Delay  | Sample | BandW | Delay  | Sample |
+-----+-------+--------+--------+-------+--------+--------+
|  0  | 260Hz |    0ms |  1kHz  | 256Hz | 0.98ms |  8kHz  |
|  1  | 184Hz |  2.0ms |  1kHz  | 188Hz |  1.9ms |  1kHz  |
|  2  |  94Hz |  3.0ms |  1kHz  |  98Hz |  2.8ms |  1kHz  |
|  3  |  44Hz |  4.9ms |  1kHz  |  42Hz |  4.8ms |  1kHz  |
|  4  |  21Hz |  8.5ms |  1kHz  |  20Hz |  8.3ms |  1kHz  |
|  5  |  10Hz | 13.8ms |  1kHz  |  10Hz | 13.4ms |  1kHz  |
|  6  |   5Hz | 19.0ms |  1kHz  |   5Hz | 18.6ms |  1kHz  |
|  7  | -- Reserved -- |  1kHz  | -- Reserved -- |  8kHz  |
*/

//���ٶȴ������ú�����������üĴ���
/*
+-------+----+----+----+
|FCHOICE|DLPF|BW  |RATE|
+-------+----+----+----+
|0      |X   |1.13|4K  |
|1      |0   |460 |1K  |
|1      |1   |184 |1K  |
|1      |2   |92  |1K  |
|1      |3   |41  |1K  |
|1      |4   |20  |1K  |
|1      |5   |10  |1K  |
|1      |6   |10  |1K  |
|1      |7   |460 |1K  |
+-------+----+----+----+
*/

//���������ص�ԭʼ�Ĵ������ݣ��ڶ�ʱ�ж��ж�ȡ
extern signed short mpu_temp;


signed short mpu_accel_cali[3];
signed short mpu_gyro_cali[3];
signed short mpu_magn_cali[6];

#define BYTE0(dwTemp)       (*(char *)(&dwTemp))
#define BYTE1(dwTemp)       (*((char *)(&dwTemp) + 1))
#define BYTE2(dwTemp)       (*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp)       (*((char *)(&dwTemp) + 3))
	
u8 Sensitivity_adj_data[3]; 
float ASA[3] = {1,1,1};
unsigned static short buffer[7];
signed static short cali_data[15];
MPU9250_Data Raw_Data,Cali_Data;

u8 MPU_Write(u8 MPU_Adr,u8 address,u8 val)
{
    I2C_Start();
    I2C_SendByte(MPU_Adr);//����������ַ
	   if(!I2C_WaitAck())
    {
        I2C_Stop(); 
        return 0;
    }  
//    I2C_Ack();
    I2C_SendByte(address);   //���õ���ʼ��ַ
		 if(!I2C_WaitAck())
    {
        I2C_Stop(); 
        return 0;
    }  
//    I2C_Ack();
    I2C_SendByte(val);
    Delay_ms(10);
		if(!I2C_WaitAck())
    {
        I2C_Stop(); 
        return 0;
    }  
//    I2C_Ack();
    I2C_Stop();
    Delay_ms(10);
    //ע�⣺��Ϊ����Ҫ�ȴ�EEPROMд�꣬���Բ��ò�ѯ����ʱ��ʽ(10ms)
    
    return 1;
}

u8 MPU_Read(u8 MPU_Adr,u8 address)//���ֽ�
{
 
    u8  temp=0;
    I2C_Start();
    I2C_SendByte(MPU_Adr);//����������ַ
    I2C_Ack();
    I2C_SendByte(address);   //���õ���ʼ��ַ

    I2C_Ack();
    I2C_Start();
    I2C_SendByte(MPU_Adr|0x01);//����������ַ   
  
    I2C_Ack();
    temp=I2C_ReceiveByte();    //
    I2C_NAck();
    I2C_Stop();
    return temp;     
}

u8 MPU_write_String (u8 MPU_Adr,u8 *buff,u8 address,u8 length)
{
    
    I2C_SendByte(MPU_Adr);//����������ַ 
    if(!I2C_WaitAck())
    {
        I2C_Stop(); 
        return 0;
    }   
    I2C_SendByte(address);   //���õ���ʼ��ַ      
    I2C_WaitAck();
    while(length--)
    {
        I2C_SendByte(*buff);
        I2C_WaitAck();
        buff++;
    }       
    I2C_Stop(); 
    //ע�⣺��Ϊ����Ҫ�ȴ�EEPROMд�꣬���Բ��ò�ѯ����ʱ��ʽ(10ms)
    Delay_ms(10);
    return 1;   
}
u8 MPU_Read_String(u8 MPU_Adr,u8 address,u8 *buff,u8 length)//���ַ���

{
   I2C_Start();
   I2C_SendByte(MPU_Adr);//����������ַ
   I2C_WaitAck();
   I2C_SendByte(address);   //���õ���ʼ��ַ      
   I2C_WaitAck();
   I2C_Start();
   I2C_SendByte(MPU_Adr|0x01);//����������ַ
   I2C_WaitAck();
    while(length)
    {
        *buff=I2C_ReceiveByte();
        if(length==1)
            I2C_NAck();
        else
            I2C_Ack();
        buff++;
        length--;
    }
    I2C_Stop();
    return 1;    
}


u8 i2c_dev;  
int Mpu9250_Work_Mode_Init(void)
{
	Delay_ms(300);	
	MPU_Write(MPU6500_I2C_ADDR,MPU6500_PWR_MGMT_1,0x00);   // PWR_MGMT_1  MPU9250��Դ����Ĵ����������
	Delay_ms(100);
	i2c_dev=MPU_Read(MPU6500_I2C_ADDR,MPU6500_WHO_AM_I);   //i2c_dev  ��ȡMPU_Read ����ֵ
	if(i2c_dev == 0x73 )    //�豸ID
	{ 
		Delay_ms(1000);   //   �˴�����ʱ����Ҫ����ʼ�Ƚ���ʱС�����¶���ȫ��0XFF��
		MPU_Write(MPU6500_I2C_ADDR,MPU6500_PWR_MGMT_1,0x80);   //��λMPU9250
		MPU_Write(MPU6500_I2C_ADDR,MPU6500_PWR_MGMT_2,0x00);   //ʹ�ܼĴ��� X��Y��Z���ٶ�
		MPU_Write(MPU6500_I2C_ADDR,MPU6500_SMPLRT_DIV,0x01);   //SMPLRT_DIV �����ʷ�Ƶ�Ĵ������������ʱ��Ϊ1kHz�������ǲ�����1000/(1+7)=125HZ
		MPU_Write(MPU6500_I2C_ADDR,MPU6500_CONFIG,MPU_GYRO_LPS_20HZ);//��Ϊ0x05ʱ��Gyro�Ĵ���Ϊ10Hz����ʱΪ17.85ms����Ϊ0x06ʱ������5Hz����ʱ33.48ms������ʹ��0x05��
		MPU_Write(MPU6500_I2C_ADDR,MPU6500_GYRO_CONFIG,MPU_GYRO_FS_2000);//=>��1000dps  
		MPU_Write(MPU6500_I2C_ADDR,MPU6500_USER_CTRL,0x00);          // ��ʼ��I2C
		MPU_Write(MPU6500_I2C_ADDR,MPU6500_ACCEL_CONFIG,MPU_ACCE_FS_8G);// ���ٶȼƲ�����Χ����8g 
		MPU_Write(MPU6500_I2C_ADDR,MPU6500_ACCEL_CONFIG_2, MPU_GYRO_LPS_20HZ);  //���ٶȼ�92Hz��ͨ�˲���
		//��ʼ����������
//		MPU_Write(MPU6500_I2C_ADDR,MPU6500_INT_PIN_CFG,0x02);       //����Bypassģʽ������ֱ�ӿ��ƴ�����
//		MPU_Write(AK8963_I2C_ADDR,AK8963_CNTL2,0x01); //RESET
//		Delay_ms(20);
//		MPU_Write(AK8963_I2C_ADDR,AK8963_CNTL1,0x0F); //Fuse ROM access mode��������������Ƚ�������
//		Delay_ms(20);
//		Sensitivity_adj_data[0] = MPU_Read(AK8963_I2C_ADDR,AK8963_ASAX);
//		Sensitivity_adj_data[1] = MPU_Read(AK8963_I2C_ADDR,AK8963_ASAX);
//		Sensitivity_adj_data[2] = MPU_Read(AK8963_I2C_ADDR,AK8963_ASAX);
//		MPU_Write(AK8963_I2C_ADDR,AK8963_CNTL1,0x00);  //Power-down mode����ý������ݺ���Ҫ���óɵ���ģʽ
//		Delay_ms(20);
//		//����������У׼����
//		ASA[0] = ((Sensitivity_adj_data[0] - 128) * 0.5)/128 + 1;  
//		ASA[1] = ((Sensitivity_adj_data[1] - 128) * 0.5)/128 + 1;
//		ASA[2] = ((Sensitivity_adj_data[2] - 128) * 0.5)/128 + 1;
//		MPU_Write(AK8963_I2C_ADDR,AK8963_CNTL1,0x11);  //���������Ƶ��β���ģʽ 16λ����
		Delay_ms(300);
		return 0;
	}
	return 1;
}



/*��ȡMPU9250����*/
u8 TX_DATA[4];//��ʾ�ݻ����� 
u8 BUF[10];//�������ݻ����� 

short T_X,T_Y,T_Z,T_T;//X,Y,Z�ᣬ�¶� 
/*ģ��IIC�˿�������붨��*/
/****************************************/

//��ȡ���ٶȼ�����
u8* READ_MPU9250_ACCEL(void) 
{ 
    //��ȡ����X������    T_X =advalue/
    BUF[0]=MPU_Read(MPU6500_I2C_ADDR,MPU6500_ACCEL_XOUT_L ); 
    BUF[1]=MPU_Read(MPU6500_I2C_ADDR,MPU6500_ACCEL_XOUT_H ); 
    T_X= (BUF[1]<<8)|BUF[0];
    //T_X/=8;
    
    //��ȡ����Y������
    BUF[2]=MPU_Read(MPU6500_I2C_ADDR,MPU6500_ACCEL_YOUT_L); 
    BUF[3]=MPU_Read(MPU6500_I2C_ADDR,MPU6500_ACCEL_YOUT_H); 
    T_Y= (BUF[3]<<8)|BUF[2]; 
    //T_Y/=8; 
    
    //��ȡ����Z������
    BUF[4]=MPU_Read(MPU6500_I2C_ADDR,MPU6500_ACCEL_ZOUT_L);
    BUF[5]=MPU_Read(MPU6500_I2C_ADDR,MPU6500_ACCEL_ZOUT_H); 
    T_Z= (BUF[5]<<8)|BUF[4]; 
	  //T_Z/=8; 
	  if(cali_data[0]==0x1111) //Flash���ڼ��ٶȽ�������
		{
			mpu_acce[0]=T_X - cali_data[1];
			mpu_acce[1]=T_Y - cali_data[2];
			mpu_acce[2]=T_Z - cali_data[3];
			Cali_Data.Acce_Data[0] = mpu_acce[0] * MPU_ACCE_K;
			Cali_Data.Acce_Data[1] = mpu_acce[1] * MPU_ACCE_K;
			Cali_Data.Acce_Data[2] = mpu_acce[2] * MPU_ACCE_K;
		}
		else 
		{
			mpu_acce[0]=T_X;
			mpu_acce[1]=T_Y;
			mpu_acce[2]=T_Z;
			Raw_Data.Acce_Data[0] = mpu_acce[0] * MPU_ACCE_K;
			Raw_Data.Acce_Data[1] = mpu_acce[1] * MPU_ACCE_K;
			Raw_Data.Acce_Data[2] = mpu_acce[2] * MPU_ACCE_K;
		}
		return BUF;
} 

//��ȡ����������
u8* READ_MPU9250_GYRO(void) 
{ 
    //��ȡ����X������
    BUF[0]=MPU_Read(MPU6500_I2C_ADDR,MPU6500_GYRO_XOUT_L); 
    BUF[1]=MPU_Read(MPU6500_I2C_ADDR,MPU6500_GYRO_XOUT_H); 
    T_X=(BUF[1]<<8)|BUF[0]; 
    //T_X/=32.8; 
    
    //��ȡ����Y������
    BUF[2]=MPU_Read(MPU6500_I2C_ADDR,MPU6500_GYRO_YOUT_L); 
    BUF[3]=MPU_Read(MPU6500_I2C_ADDR,MPU6500_GYRO_YOUT_H); 
    T_Y= (BUF[3]<<8)|BUF[2];
   // T_Y/=32.8; 
    
    //��ȡ����Z������
    BUF[4]=MPU_Read(MPU6500_I2C_ADDR,MPU6500_GYRO_ZOUT_L); 
    BUF[5]=MPU_Read(MPU6500_I2C_ADDR,MPU6500_GYRO_ZOUT_H); 
    T_Z=(BUF[5]<<8)|BUF[4]; 
    //T_Z/=32.8; 
		if(cali_data[4]==0x2222) //Flash���������ǽ�������
		{
			mpu_gyro[0]=T_X - cali_data[5];
			mpu_gyro[1]=T_Y - cali_data[6];
			mpu_gyro[2]=T_Z - cali_data[7];
			Cali_Data.Gyro_Data[0] = mpu_gyro[0] * MPU_GYRO_K;
			Cali_Data.Gyro_Data[1] = mpu_gyro[1] * MPU_GYRO_K;
			Cali_Data.Gyro_Data[2] = mpu_gyro[2] * MPU_GYRO_K;
		}
		else
		{
			mpu_gyro[0]=T_X;
			mpu_gyro[1]=T_Y;
			mpu_gyro[2]=T_Z;
			Raw_Data.Gyro_Data[0] = mpu_gyro[0] * MPU_GYRO_K;
			Raw_Data.Gyro_Data[1] = mpu_gyro[1] * MPU_GYRO_K;
			Raw_Data.Gyro_Data[2] = mpu_gyro[2] * MPU_GYRO_K;
		}
		return BUF;
} 

//��ȡ����������
void READ_MPU9250_MAG(void) 
{ 
//	  BUF[6]=MPU_Read(AK8963_I2C_ADDR,AK8963_ST1);
//	  if(BUF[6] == AK8963_ST1_DRDY)
//		{
			MPU_Write(AK8963_I2C_ADDR,AK8963_CNTL1,0x11);
			// ��ȡ����X������
			BUF[0]=MPU_Read (AK8963_I2C_ADDR,AK8963_HXL); 
			BUF[1]=MPU_Read (AK8963_I2C_ADDR,AK8963_HXH); 
			T_X=(BUF[1]<<8)|BUF[0]; 
			// ��ȡ����Y������
			BUF[2]=MPU_Read(AK8963_I2C_ADDR,AK8963_HYL); 
			BUF[3]=MPU_Read(AK8963_I2C_ADDR,AK8963_HYH); 
			T_Y=(BUF[3]<<8)|BUF[2]; 
			// ��ȡ����Z������
			BUF[4]=MPU_Read(AK8963_I2C_ADDR,AK8963_HZL); 
			BUF[5]=MPU_Read(AK8963_I2C_ADDR,AK8963_HZH); 
			T_Z= (BUF[5]<<8)|BUF[4]; 
//		}
		if(cali_data[8]==0x3333) //Flash���ڴ����ƽ�������
		{
			mpu_magn[0]=T_X;
			mpu_magn[1]=T_Y;
 			mpu_magn[2]=T_Z;
			Cali_Data.Magn_data[0] = ((float)mpu_magn[0] - (float)cali_data[9]/100) * ((float)cali_data[13]/(float)cali_data[12]) * MPU_MAGN_K * ASA[0];
		  Cali_Data.Magn_data[1] = ((float)mpu_magn[1] + (float)cali_data[10]/100) * ((float)cali_data[13]/(float)cali_data[13]) * MPU_MAGN_K * ASA[1];
			Cali_Data.Magn_data[2] = ((float)mpu_magn[2] + (float)cali_data[11]/100) * ((float)cali_data[13]/(float)cali_data[14]) * MPU_MAGN_K * ASA[2];
		}
		else
		{
			mpu_magn[0]=T_X;
			mpu_magn[1]=T_Y;
			mpu_magn[2]=T_Z;
			Raw_Data.Magn_data[0] = mpu_magn[0] * MPU_MAGN_K * ASA[0];
			Raw_Data.Magn_data[1] = mpu_magn[1] * MPU_MAGN_K * ASA[1];
			Raw_Data.Magn_data[2] = mpu_magn[2] * MPU_MAGN_K * ASA[2];
		}
} 


//���ٶȼ����������ݴ洢
void calibrate_accel()
{
  int count=0;
	signed int accel_x=0, accel_y=0, accel_z=0;
	while(count<MPU_GYRO_ZERO_CALI_FACT)
	{ 
		READ_MPU9250_ACCEL(); 
	  count++;
		accel_x += mpu_acce[0];
		accel_y += mpu_acce[1];
		accel_z += mpu_acce[2];
	}
  
	mpu_accel_cali[0]=(accel_x / count);
	mpu_accel_cali[1]=(accel_y / count);
	mpu_accel_cali[2]=(accel_z / count);
	
	buffer[0]=0x1111;
	buffer[1]=(short) mpu_accel_cali[0];
	buffer[2]=(short) mpu_accel_cali[1];
	buffer[3]=(short)(mpu_accel_cali[2] - Accel_8G_Ref);
	STMFLASH_Write(FLASH_ACCEL_OFFSET_ADDR,buffer,4);
	count=0;
}

//���������������ݴ洢
void calibrate_gyro()
{
  int count=0;
	signed int gyro_x=0, gyro_y=0, gyro_z=0;
	while(count<MPU_GYRO_ZERO_CALI_FACT)
	{ 
		READ_MPU9250_GYRO(); 
	  count++;
		gyro_x += mpu_gyro[0];
		gyro_y += mpu_gyro[1];
		gyro_z += mpu_gyro[2];
	}
  
	mpu_gyro_cali[0]=(gyro_x / count);
	mpu_gyro_cali[1]=(gyro_y / count);
	mpu_gyro_cali[2]=(gyro_z / count);
	buffer[0]=0x2222;
	buffer[1]=(short)mpu_gyro_cali[0];
	buffer[2]=(short)mpu_gyro_cali[1];
	buffer[3]=(short)mpu_gyro_cali[2];
	STMFLASH_Write(FLASH_GYRO_OFFSET_ADDR,buffer,4);
	count=0;
}

//���������������ݴ洢
void calibrate_Meg(float X0, float Y0, float Z0 ,float a, float b, float c)
{ 
	mpu_magn_cali[0]=(short)(X0*1);
	mpu_magn_cali[1]=(short)(Y0*1);
	mpu_magn_cali[2]=(short)(Z0*1);
  mpu_magn_cali[3]=(short)(a*1);
	mpu_magn_cali[4]=(short)(b*1);
	mpu_magn_cali[5]=(short)(c*1);
	
	buffer[0]=0x3333;
	buffer[1]=(short)mpu_magn_cali[0];
	buffer[2]=(short)mpu_magn_cali[1];
	buffer[3]=(short)mpu_magn_cali[2];
	buffer[4]=(short)mpu_magn_cali[3];
	buffer[5]=(short)mpu_magn_cali[4];
	buffer[6]=(short)mpu_magn_cali[5];
	
	STMFLASH_Write(FLASH_MAG_OFFSET_ADDR,buffer,7);
}

void ReadCalibrateData()
{
	STMFLASH_Read(MPU_CORRECTION_FLASH,(u16*)cali_data,15);	
}

void MPU9250_gyromagSleep()
{
  uint8_t temp = 0;
  temp = MPU_Read(AK8963_I2C_ADDR, AK8963_CNTL1);
  MPU_Write(AK8963_I2C_ADDR, AK8963_CNTL1, temp & ~(0x0F) ); // Clear bits 0- 3 to power down magnetometer
  temp = MPU_Read(MPU6500_I2C_ADDR, MPU6500_PWR_MGMT_1);
  MPU_Write(MPU6500_I2C_ADDR, MPU6500_PWR_MGMT_1, temp | 0x10);     // Write bit 4 to enable gyro standby
  Delay_ms(10); // Wait for all registers to reset
}

void MPU9250_gyromagWake()
{
//  temp = MPU_Read(AK8963_I2C_ADDR, AK8963_CNTL1);
//  MPU_Write(AK8963_I2C_ADDR, AK8963_CNTL1, temp | _Mmode ); // Reset normal mode for  magnetometer
  uint8_t temp = MPU_Read(MPU6500_I2C_ADDR, MPU6500_PWR_MGMT_1);
  MPU_Write(MPU6500_I2C_ADDR, MPU6500_PWR_MGMT_1, 0x01);   // return gyro and accelnormal mode
  Delay_ms(10); // Wait for all registers to reset
}


void MPU9250_MPU9250wake()
{
  // Clear sleep bit to wake MPU9250
  uint8_t c = MPU_Read(MPU6500_I2C_ADDR, MPU6500_PWR_MGMT_1);
  MPU_Write(MPU6500_I2C_ADDR, MPU6500_PWR_MGMT_1, c & ~(0x40) ); // Clear sleep modebit (6), enable all sensors
  Delay_ms(100); // Wait for all registers to reset
}

void MPU9250_MPU9250sleep()
{
  // Put the device to sleep
  uint8_t c = MPU_Read(MPU6500_I2C_ADDR, MPU6500_PWR_MGMT_1);
  MPU_Write(MPU6500_I2C_ADDR, MPU6500_PWR_MGMT_1, c | 0x40); // Set sleep mode bit(6), disable all sensors
  Delay_ms(100); // Wait for all registers to reset
}
