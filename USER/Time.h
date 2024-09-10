#ifndef _TIME_H_
#define _TIME_H_

#include "stm32f10x.h"


void TIM2_IRQ(void);
void SYSTICK_INIT(void);
void get_ms(unsigned long  *time);

void TIM3_Int_Init(u16 arr,u16 psc);
void SysTick_Init(void);
float GET_NOWTIME(void);
float GET_NOWTIME1(void);


#endif

