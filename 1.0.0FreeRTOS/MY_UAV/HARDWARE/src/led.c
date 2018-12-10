#include "led.h"
#include "includes.h"
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK STM32F429������
//LED��������	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//��������:2015/11/23
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	

//��ʼ��PE14\15Ϊ���.��ʹ��ʱ��	    
//LED IO��ʼ��

void LED_Init(void)
{
    GPIO_InitTypeDef GPIO_Initure;
    __HAL_RCC_GPIOE_CLK_ENABLE();           //����GPIOEʱ��
	
    GPIO_Initure.Pin=GPIO_PIN_14|GPIO_PIN_15; //PE14,15
    GPIO_Initure.Mode=GPIO_MODE_OUTPUT_PP;  //�������
    GPIO_Initure.Pull=GPIO_PULLUP;          //����
    GPIO_Initure.Speed=GPIO_SPEED_HIGH;     //����
    HAL_GPIO_Init(GPIOE,&GPIO_Initure);
	
    HAL_GPIO_WritePin(GPIOE,GPIO_PIN_14,GPIO_PIN_RESET);	
    HAL_GPIO_WritePin(GPIOE,GPIO_PIN_15,GPIO_PIN_RESET);	
}


void ledCtrl(led Led,ledStatus LedStatus)
{
   if(LedStatus==LED_ON)
   {
      if(Led==LED1)
         HAL_GPIO_WritePin(GPIOE,GPIO_PIN_14,GPIO_PIN_SET);	
      else if(Led==LED2)
         HAL_GPIO_WritePin(GPIOE,GPIO_PIN_15,GPIO_PIN_SET);
   }
   else if(LedStatus==LED_OFF)
   {
      if(Led==LED1)
         HAL_GPIO_WritePin(GPIOE,GPIO_PIN_14,GPIO_PIN_RESET);	
      else if(Led==LED2)
         HAL_GPIO_WritePin(GPIOE,GPIO_PIN_15,GPIO_PIN_RESET);
   }
   
}



