#include "ultrasonic.h"
iic GY_US42_iic;
void GY_US42_Init()
{
    iicSingleInit(GY_US42_iic=iicPortDef(GPIOA,GPIO_PIN_4,GPIOA,GPIO_PIN_5));
}
