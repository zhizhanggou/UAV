#include "includes.h"

 

/*������������ʹ��HAL�⺯��ʵ�ֿ���IO�����*/
TaskHandle_t startTaskHandle;
static void startTask(void *arg);
int main(void)
{
  
   sysInit();
  // taskENTER_CRITICAL();
   xTaskCreate(startTask, "START_TASK", 300, NULL, 2, &startTaskHandle);	/*������ʼ����*/
   vTaskStartScheduler();	/*�����������*/
   while(1){};
  // taskEXIT_CRITICAL();	/*�˳��ٽ���*/
}
 

void startTask(void *arg)
{
	taskENTER_CRITICAL();	/*�����ٽ���*/
	
   xTaskCreate(vTaskLED, "vTaskLED", 150, NULL, 3, NULL);
	vTaskDelete(startTaskHandle);										/*ɾ����ʼ����*/
		
	taskEXIT_CRITICAL();	/*�˳��ٽ���*/
} 




 
/*����������ʹ��λ������ʵ�֣�*/
/*
int main(void)
{ 
    HAL_Init();                     //��ʼ��HAL��   
    Stm32_Clock_Init(360,25,2,8);   //����ʱ��,180Mhz
    delay_init(180);                //��ʼ����ʱ����
    LED_Init();                     //��ʼ��LED    while(1)
	{
         LED0=0;			     //LED0��
	     LED1=1;				 //LED1��
		 delay_ms(500);
		 LED0=1;				//LED0��
		 LED1=0;				//LED1��
		 delay_ms(500);
	 }
}*/




/*
����������ʹ��ֱ�Ӳ����������ʽʵ�������
*/
/*
int main(void)
{ 
    HAL_Init();                     //��ʼ��HAL��   
    Stm32_Clock_Init(360,25,2,8);   //����ʱ��,180Mhz
    delay_init(180);                //��ʼ����ʱ����
    LED_Init();                     //��ʼ��LED  
	while(1)
	{
      GPIOB->BSRR=GPIO_PIN_1;     //LED0��
	  GPIOB->BSRR=GPIO_PIN_0<<16; //LED1��
	  delay_ms(500);
      GPIOB->BSRR=GPIO_PIN_1<<16; //LED0��
	  GPIOB->BSRR=GPIO_PIN_0;     //LED1��
	  delay_ms(500);
	 }
 }	
*/




