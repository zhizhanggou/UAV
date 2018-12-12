#include "sysInit.h"
#include "includes.h"
void sysInit()
{
	HAL_Init();                     //��ʼ��HAL��   
	Stm32_Clock_Init(360,25,2,8);   //����ʱ��,180Mhz
	delay_init(180);                //��ʼ����ʱ����
	uart_init(500000);
   delay_ms(20);
	LED_Init();                     //��ʼ��LED  
	mpu9250_Init();						//9250��ʼ��
	sensorsQueueInit();					//��������Ϣ���г�ʼ��
	
   
}
