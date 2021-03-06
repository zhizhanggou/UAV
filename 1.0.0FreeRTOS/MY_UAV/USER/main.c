#include "includes.h"

 

/*下面主函数是使用HAL库函数实现控制IO口输出*/

TaskHandle_t startTaskHandle;
static void startTask(void *arg);
static void AppObjCreate (void);
int main(void)
{
  
   sysInit();
  // taskENTER_CRITICAL();
   xTaskCreate(startTask, "START_TASK", 300, NULL, 2, &startTaskHandle);	/*创建起始任务*/
   AppObjCreate (); //
   vTaskStartScheduler();	/*开启任务调度*/
   while(1){};
  // taskEXIT_CRITICAL();	/*退出临界区*/
}
 

void startTask(void *arg)
{
	taskENTER_CRITICAL();	/*进入临界区*/
	xTaskCreate(vTaskIndicatorLED, "vTaskIndicatorLED", 150, NULL, 2, NULL);
   xTaskCreate(vTaskDataUpload, "vTaskDataUpload", 150, NULL, 3, NULL);
   xTaskCreate(vTaskAttitudeAlgorithm, "vTaskAttitudeAlgorithm", 150, NULL, 5, NULL);
   xTaskCreate(vTaskReadSenser, "vTaskReadSenser", 150, NULL, 4, NULL);
   
	vTaskDelete(startTaskHandle);										/*删除开始任务*/
		
	taskEXIT_CRITICAL();	/*退出临界区*/
}  


/*******************************
创建任务通讯机制
*******************************/
static void AppObjCreate (void)
{
/* 创建事件标志组 */
   xSensorEventGroup = xEventGroupCreate();
   xUploadEventGroup = xEventGroupCreate();
   if(xSensorEventGroup == NULL)
   {
      /* 没有创建成功，用户可以在这里加入创建失败的处理机制 */
   }
    if(xUploadEventGroup == NULL)
   {
      /* 没有创建成功，用户可以在这里加入创建失败的处理机制 */
   }
}



 
/*下面主函数使用位带操作实现：*/
/*
int main(void)
{ 
    HAL_Init();                     //初始化HAL库   
    Stm32_Clock_Init(360,25,2,8);   //设置时钟,180Mhz
    delay_init(180);                //初始化延时函数
    LED_Init();                     //初始化LED    while(1)
	{
         LED0=0;			     //LED0亮
	     LED1=1;				 //LED1灭
		 delay_ms(500);
		 LED0=1;				//LED0灭
		 LED1=0;				//LED1亮
		 delay_ms(500);
	 }
}*/




/*
下面主函数使用直接操作结存器方式实现跑马灯
*/
/*
int main(void)
{ 
    HAL_Init();                     //初始化HAL库   
    Stm32_Clock_Init(360,25,2,8);   //设置时钟,180Mhz
    delay_init(180);                //初始化延时函数
    LED_Init();                     //初始化LED  
	while(1)
	{
      GPIOB->BSRR=GPIO_PIN_1;     //LED0亮
	  GPIOB->BSRR=GPIO_PIN_0<<16; //LED1灭
	  delay_ms(500);
      GPIOB->BSRR=GPIO_PIN_1<<16; //LED0灭
	  GPIOB->BSRR=GPIO_PIN_0;     //LED1亮
	  delay_ms(500);
	 }
 }	
*/




