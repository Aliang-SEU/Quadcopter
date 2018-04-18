#ifndef TIM_H_
#define TIM_H_
#include "stm32f10x.h"

extern uint32_t TIM2_IRQCNT;

void Tim2_Init(u16 period_num);//用于精确延时

#endif
