#include "led.h"
#include "includes.h"
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32F429开发板
//LED驱动代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//创建日期:2015/11/23
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	

//初始化PE14\15为输出.并使能时钟	    
//LED IO初始化

void LED_Init(void)
{
    GPIO_InitTypeDef GPIO_Initure;
    __HAL_RCC_GPIOE_CLK_ENABLE();           //开启GPIOE时钟
	
    GPIO_Initure.Pin=GPIO_PIN_14|GPIO_PIN_15; //PE14,15
    GPIO_Initure.Mode=GPIO_MODE_OUTPUT_PP;  //推挽输出
    GPIO_Initure.Pull=GPIO_PULLUP;          //上拉
    GPIO_Initure.Speed=GPIO_SPEED_HIGH;     //高速
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



