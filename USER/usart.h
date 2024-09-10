#ifndef _USART_H_
#define _USART_H_

#include "stm32f10x.h"
void USART1_Config(void);
void UsartSend(uint8_t ch);
void UsartSendPackage(uint8_t *data, uint8_t len);

void USART_NVIC_init(void);
void Print(uint8_t num);
void PrintInt(uint16_t num);
void PrintChar(char *s);
void PrintHexInt16(int16_t num);
void usart_INIT(void);



#endif

