#include "stm32f10x.h"
static uint16_t ADsampleFilter(__IO uint16_t *_ADCConvertedValue);
void AdcDmaIrq(void);
void AdcDmaInit(void);
