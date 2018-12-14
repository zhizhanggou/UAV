#include "timer.h"

TIM_HandleTypeDef TIM6_Handler;      //��ʱ����� 
uint8_t timerInitStatus;
void TIM6_Int_Init(u16 arr,u16 psc)
{
	__HAL_RCC_TIM6_CLK_ENABLE();            //ʹ��TIM3ʱ��
    TIM6_Handler.Instance=TIM6;                          //ͨ�ö�ʱ��3
    TIM6_Handler.Init.Prescaler=psc;                     //��Ƶϵ��
    TIM6_Handler.Init.CounterMode=TIM_COUNTERMODE_UP;    //���ϼ�����
    TIM6_Handler.Init.Period=arr;                        //�Զ�װ��ֵ
    TIM6_Handler.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;//ʱ�ӷ�Ƶ����
    timerInitStatus=HAL_TIM_Base_Init(&TIM6_Handler);
	timerInitStatus=HAL_TIM_Base_Start(&TIM6_Handler);
}
