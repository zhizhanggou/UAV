#include "includes.h"

 

/*������������ʹ��HAL�⺯��ʵ�ֿ���IO�����*/

TaskHandle_t startTaskHandle;
static void startTask(void *arg);
static void AppObjCreate (void);
int main(void)
{
  
   sysInit();
  // taskENTER_CRITICAL();
   xTaskCreate(startTask, "START_TASK", 300, NULL, 2, &startTaskHandle);	/*������ʼ����*/
   AppObjCreate (); //
   vTaskStartScheduler();	/*�����������*/
   while(1){};
  // taskEXIT_CRITICAL();	/*�˳��ٽ���*/
}
 

void startTask(void *arg)
{
	taskENTER_CRITICAL();	/*�����ٽ���*/
	xTaskCreate(vTaskIndicatorLED, "vTaskIndicatorLED", 150, NULL, 3, NULL);
   xTaskCreate(vTaskDataUpload, "vTaskDataUpload", 150, NULL, 4, NULL);
   //xTaskCreate(vTaskAttitudeAlgorithm, "vTaskAttitudeAlgorithm", 150, NULL, 4, NULL);
   xTaskCreate(vTaskReadSenser, "vTaskReadSenser", 150, NULL, 5, NULL);
   
	vTaskDelete(startTaskHandle);										/*ɾ����ʼ����*/
		
	taskEXIT_CRITICAL();	/*�˳��ٽ���*/
}  


/*******************************
��������ͨѶ����
*******************************/
static void AppObjCreate (void)
{
/* �����¼���־�� */
   xCreatedEventGroup = xEventGroupCreate();
   if(xCreatedEventGroup == NULL)
   {
      /* û�д����ɹ����û�������������봴��ʧ�ܵĴ������ */
   }
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




