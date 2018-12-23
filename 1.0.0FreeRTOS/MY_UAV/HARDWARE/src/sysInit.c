#include "sysInit.h"
#include "includes.h"
void sysInit()
{
	HAL_Init();                     //��ʼ��HAL��   
	Stm32_Clock_Init(360,25,2,8);   //����ʱ��,180Mhz
	delay_init(180);                //��ʼ����ʱ����
	uart_init(500000);
	delay_ms(20);
	TIM6_Int_Init(65535-1,90-1);
	LED_Init();                     //��ʼ��LED  
	mpu9250_Init();						//9250��ʼ��
	GY_US42_Init();
  MS5611_Init();
  sensorsInit();
	sensorsQueueInit();					//��������Ϣ���г�ʼ��
//	DataToSend=USER_DATA;
   
}
