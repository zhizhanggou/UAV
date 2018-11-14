#include "iic.h"
#include "includes.h"


iic iicNo1PortDef()
{
   iic iicNo1;
   iicNo1.sclPort=GPIO_PIN_3;
   iicNo1.sdaPort=GPIO_PIN_4;
   iicNo1.iic_Port=GPIOD;
   __HAL_RCC_GPIOD_CLK_ENABLE();           //����GPIOEʱ��
   return iicNo1;
}
iic iicNo2PortDef()
{
   iic iicNo2;
   iicNo2.sclPort=GPIO_PIN_3;
   iicNo2.sdaPort=GPIO_PIN_4;
   iicNo2.iic_Port=GPIOD;
   __HAL_RCC_GPIOD_CLK_ENABLE();           //����GPIOEʱ��
   return iicNo2;
}

void iicSingleInit(iic iicPort)
{
   GPIO_InitTypeDef GPIO_Initure;
//   __HAL_RCC_GPIOD_CLK_ENABLE();           //����GPIOEʱ��
   
   GPIO_Initure.Pin = iicPort.sclPort | iicPort.sdaPort;
   GPIO_Initure.Mode = GPIO_MODE_OUTPUT_PP;//�������
   GPIO_Initure.Speed = GPIO_SPEED_HIGH;//100MHz
   GPIO_Initure.Pull = GPIO_PULLUP;//����
   HAL_GPIO_Init(iicPort.iic_Port,&GPIO_Initure);
}
void iicInit()
{
   iicSingleInit(iicNo1PortDef());
}
