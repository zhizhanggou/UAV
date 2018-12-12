#include "sysInit.h"
#include "includes.h"
void sysInit()
{
	HAL_Init();                     //初始化HAL库   
	Stm32_Clock_Init(360,25,2,8);   //设置时钟,180Mhz
	delay_init(180);                //初始化延时函数
	uart_init(500000);
   delay_ms(20);
	LED_Init();                     //初始化LED  
	mpu9250_Init();						//9250初始化
	sensorsQueueInit();					//传感器消息队列初始化
	
   
}
