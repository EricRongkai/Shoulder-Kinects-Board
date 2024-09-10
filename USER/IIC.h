#include "stm32f10x.h"

void I2C_EE_WaitEepromStandbyState(void);
void IIC2_Init();
void IIC_WriteByte(u8 addr,u8 data);