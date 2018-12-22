#include "ultrasonic.h"
iic GY_US42_iic;
int16_t ulterDistance;
bool isUlterDistanceReadFinish=true;
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
    iicSingleInit(GY_US42_iic=iicPortDef(GPIOA,GPIO_PIN_4,GPIOA,GPIO_PIN_5,10));
		GY_US42_IntPortInit();
}


uint16_t GY_US42_IIC_Read(iic iicPort,u8 Slaveaddress)
{
	u8 tempHigh,tempLow;
	IIC_Start(iicPort); 
	IIC_Send_Byte(iicPort,Slaveaddress|1); 
	IIC_Wait_Ack(iicPort);	
	tempHigh=IIC_Read_Byte(iicPort,1);
  tempLow=IIC_Read_Byte(iicPort,0);
	IIC_Stop(iicPort);	
	return (tempHigh<<8)|tempLow;
}
    u8 temp;
void GY_US42_IIC_Write(iic iicPort,u8 Slaveaddress,u8 REG_data)
{
  uint16_t i=100;
	IIC_Start(iicPort); 
	IIC_Send_Byte(iicPort,Slaveaddress); 
  while(i) 
   { 
     i--; 
   } 
	temp=IIC_Wait_Ack(iicPort);	
	IIC_Send_Byte(iicPort,REG_data);
	IIC_Wait_Ack(iicPort); 
	IIC_Stop(iicPort);
}


void GY_US42_StartMeasure()
{
    if(isUlterDistanceReadFinish)
    {
        GY_US42_IIC_Write(GY_US42_iic,GY_US42_ADDR,GY_US42_START_CMD);
        isUlterDistanceReadFinish=false;
    }
    
}

void GY_US42_ReadData()
{

  
    GY_US42_StartMeasure();
    temp=PAin(6);
    if(temp==GY_US42_DATA_READY)
    {
        ulterDistance=GY_US42_IIC_Read(GY_US42_iic,GY_US42_ADDR);
        isUlterDistanceReadFinish=true;
    }
    else
    {
        isUlterDistanceReadFinish=false;
    }
    
}

	

