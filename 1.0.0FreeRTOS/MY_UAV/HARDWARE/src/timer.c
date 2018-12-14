#include "timer.h"

TIM_HandleTypeDef TIM6_Handler;      //定时器句柄 
uint8_t timerInitStatus;
void TIM6_Int_Init(u16 arr,u16 psc)
{
	__HAL_RCC_TIM6_CLK_ENABLE();            //使能TIM3时钟
    TIM6_Handler.Instance=TIM6;                          //通用定时器3
    TIM6_Handler.Init.Prescaler=psc;                     //分频系数
    TIM6_Handler.Init.CounterMode=TIM_COUNTERMODE_UP;    //向上计数器
    TIM6_Handler.Init.Period=arr;                        //自动装载值
    TIM6_Handler.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;//时钟分频因子
    timerInitStatus=HAL_TIM_Base_Init(&TIM6_Handler);
	timerInitStatus=HAL_TIM_Base_Start(&TIM6_Handler);
}
