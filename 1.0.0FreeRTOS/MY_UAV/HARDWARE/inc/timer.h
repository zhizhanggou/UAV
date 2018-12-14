#ifndef __TIMER_H
#define __TIMER_H

#include "includes.h"
void TIM6_Int_Init(u16 arr,u16 psc);
extern TIM_HandleTypeDef TIM6_Handler;      //¶¨Ê±Æ÷¾ä±ú
#endif
