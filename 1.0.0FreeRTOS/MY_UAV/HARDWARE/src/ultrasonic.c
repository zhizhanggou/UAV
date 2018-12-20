#include "ultrasonic.h"
iic GY_US42_iic;


void GY_US42_IntPortInit()
{
		GPIO_InitTypeDef GPIO_Initure;
    __HAL_RCC_GPIOA_CLK_ENABLE();           //开启GPIOE时钟
	
    GPIO_Initure.Pin=GPIO_PIN_6; //PE14,15
    GPIO_Initure.Mode=GPIO_MODE_INPUT;  //推挽输出
    GPIO_Initure.Pull=GPIO_PULLUP;          //上拉
    GPIO_Initure.Speed=GPIO_SPEED_HIGH;     //高速
    HAL_GPIO_Init(GPIOA,&GPIO_Initure);
}
void GY_US42_Init()
{
    iicSingleInit(GY_US42_iic=iicPortDef(GPIOA,GPIO_PIN_4,GPIOA,GPIO_PIN_5,20));
		GY_US42_IntPortInit();
}

void GY_US42_StartMeasure()
{
		
}

	
u8 GY_US42_IIC_Read(iic iicPort,u8 Slaveaddress,u8 REG_Address)
{
	u8 REG_data;
	IIC_Start(iicPort); 
	IIC_Send_Byte(iicPort,Slaveaddress); 
	REG_data=IIC_Wait_Ack(iicPort);	
	IIC_Send_Byte(iicPort,REG_Address);
	REG_data=IIC_Wait_Ack(iicPort);	
	IIC_Start(iicPort); 
	IIC_Send_Byte(iicPort,Slaveaddress|1);
	REG_data=IIC_Wait_Ack(iicPort);	
	REG_data=IIC_Read_Byte(iicPort,0);
	IIC_Stop(iicPort);	
	return REG_data;
}

void GY_US42_IIC_Write(iic iicPort,u8 Slaveaddress,u8 REG_Address,u8 REG_data)
{
	IIC_Start(iicPort); 
	IIC_Send_Byte(iicPort,Slaveaddress); 
	IIC_Wait_Ack(iicPort);	
	IIC_Send_Byte(iicPort,REG_Address); 
	IIC_Wait_Ack(iicPort); 
	IIC_Send_Byte(iicPort,REG_data);
	IIC_Wait_Ack(iicPort); 
	IIC_Stop(iicPort);
}

