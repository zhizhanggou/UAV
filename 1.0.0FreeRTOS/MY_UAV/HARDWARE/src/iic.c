#include "iic.h"
#include "includes.h"
#include <assert.h>

/***********定义IIC端口**********/

iic iicPortDef(GPIO_TypeDef *sclGPIOx,uint16_t sclPort,GPIO_TypeDef *sdaGPIOx,uint16_t sdaPort)
{
   iic iicTemp;
   iicTemp.sclPort=sclPort;
   iicTemp.sdaPort=sdaPort;
   iicTemp.iic_sclPort=sclGPIOx;
	iicTemp.iic_sdaPort=sdaGPIOx;
	switch(sclPort)
	{
		case GPIO_PIN_0:iicTemp.iicSCLPortNum=0; break;
		case GPIO_PIN_1:iicTemp.iicSCLPortNum=1; break;
		case GPIO_PIN_2:iicTemp.iicSCLPortNum=2; break;
		case GPIO_PIN_3:iicTemp.iicSCLPortNum=3; break;
		case GPIO_PIN_4:iicTemp.iicSCLPortNum=4; break;
		case GPIO_PIN_5:iicTemp.iicSCLPortNum=5; break;
		case GPIO_PIN_6:iicTemp.iicSCLPortNum=6; break;
		case GPIO_PIN_7:iicTemp.iicSCLPortNum=7; break;
		case GPIO_PIN_8:iicTemp.iicSCLPortNum=8; break;
		case GPIO_PIN_9:iicTemp.iicSCLPortNum=9; break;
		case GPIO_PIN_10:iicTemp.iicSCLPortNum=10; break;
		case GPIO_PIN_11:iicTemp.iicSCLPortNum=11; break;
		case GPIO_PIN_12:iicTemp.iicSCLPortNum=12; break;
		case GPIO_PIN_13:iicTemp.iicSCLPortNum=13; break;
		case GPIO_PIN_14:iicTemp.iicSCLPortNum=14; break;
		case GPIO_PIN_15:iicTemp.iicSCLPortNum=15; break;
		default: assert(0); //如果不是这些端口则断言,终止程序运行
	}
		switch(sdaPort)
	{
		case GPIO_PIN_0:iicTemp.iicSDAPortNum=0; break;
		case GPIO_PIN_1:iicTemp.iicSDAPortNum=1; break;
		case GPIO_PIN_2:iicTemp.iicSDAPortNum=2; break;
		case GPIO_PIN_3:iicTemp.iicSDAPortNum=3; break;
		case GPIO_PIN_4:iicTemp.iicSDAPortNum=4; break;
		case GPIO_PIN_5:iicTemp.iicSDAPortNum=5; break;
		case GPIO_PIN_6:iicTemp.iicSDAPortNum=6; break;
		case GPIO_PIN_7:iicTemp.iicSDAPortNum=7; break;
		case GPIO_PIN_8:iicTemp.iicSDAPortNum=8; break;
		case GPIO_PIN_9:iicTemp.iicSDAPortNum=9; break;
		case GPIO_PIN_10:iicTemp.iicSDAPortNum=10; break;
		case GPIO_PIN_11:iicTemp.iicSDAPortNum=11; break;
		case GPIO_PIN_12:iicTemp.iicSDAPortNum=12; break;
		case GPIO_PIN_13:iicTemp.iicSDAPortNum=13; break;
		case GPIO_PIN_14:iicTemp.iicSDAPortNum=14; break;
		case GPIO_PIN_15:iicTemp.iicSDAPortNum=15; break;
		default: assert(0); //如果不是这些端口则断言,终止程序运行
	}
	
		switch((uint32_t)sclGPIOx)
	{
		case (uint32_t)GPIOA:__HAL_RCC_GPIOA_CLK_ENABLE(); break;
		case (uint32_t)GPIOB:__HAL_RCC_GPIOB_CLK_ENABLE(); break;
		case (uint32_t)GPIOC:__HAL_RCC_GPIOC_CLK_ENABLE(); break;
		case (uint32_t)GPIOD:__HAL_RCC_GPIOD_CLK_ENABLE(); break;
		case (uint32_t)GPIOE:__HAL_RCC_GPIOE_CLK_ENABLE(); break;
		case (uint32_t)GPIOF:__HAL_RCC_GPIOF_CLK_ENABLE(); break;
		case (uint32_t)GPIOG:__HAL_RCC_GPIOG_CLK_ENABLE(); break;
		default: assert(0); //如果不是这些端口则断言,终止程序运行
	}
		switch((uint32_t)sdaGPIOx)
	{
		case (uint32_t)GPIOA:__HAL_RCC_GPIOA_CLK_ENABLE(); break;
		case (uint32_t)GPIOB:__HAL_RCC_GPIOB_CLK_ENABLE(); break;
		case (uint32_t)GPIOC:__HAL_RCC_GPIOC_CLK_ENABLE(); break;
		case (uint32_t)GPIOD:__HAL_RCC_GPIOD_CLK_ENABLE(); break;
		case (uint32_t)GPIOE:__HAL_RCC_GPIOE_CLK_ENABLE(); break;
		case (uint32_t)GPIOF:__HAL_RCC_GPIOF_CLK_ENABLE(); break;
		case (uint32_t)GPIOG:__HAL_RCC_GPIOG_CLK_ENABLE(); break;
		default: assert(0); //如果不是这些端口则断言,终止程序运行
	}
	
              
   return iicTemp;
}
/*******************************/




void iicSingleInit(iic iicPort)
{
   GPIO_InitTypeDef GPIO_Initure;
   
   GPIO_Initure.Pin = iicPort.sclPort;
   GPIO_Initure.Mode = GPIO_MODE_OUTPUT_PP;//推挽输出
   GPIO_Initure.Speed = GPIO_SPEED_HIGH;//100MHz
   GPIO_Initure.Pull = GPIO_PULLUP;//上拉
   HAL_GPIO_Init(iicPort.iic_sclPort,&GPIO_Initure);
	
	GPIO_Initure.Pin = iicPort.sdaPort;
   GPIO_Initure.Mode = GPIO_MODE_OUTPUT_PP;//推挽输出
   GPIO_Initure.Speed = GPIO_SPEED_HIGH;//100MHz
   GPIO_Initure.Pull = GPIO_PULLUP;//上拉
   HAL_GPIO_Init(iicPort.iic_sdaPort,&GPIO_Initure);
}








void SDA_IN(iic iicPort)  
{
	iicPort.iic_sdaPort->MODER&=~(3<<(iicPort.iicSDAPortNum<<1));
	iicPort.iic_sdaPort->MODER|=0<<(iicPort.iicSDAPortNum<<1);
}	
void SDA_OUT(iic iicPort)  
{
	iicPort.iic_sdaPort->MODER&=~(3<<(iicPort.iicSDAPortNum<<1));
	iicPort.iic_sdaPort->MODER|=1<<(iicPort.iicSDAPortNum<<1);
}	

void IIC_SCL(iic iicPort,bool level)
{
  BIT_ADDR((int)iicPort.iic_sclPort+20,iicPort.iicSCLPortNum)=level;
}

void IIC_SDA(iic iicPort,bool level)
{
  BIT_ADDR((uint32_t)iicPort.iic_sdaPort+20,iicPort.iicSDAPortNum)=level;
}
uint8_t IIC_READ_SDA(iic iicPort)
{
	uint8_t temp=0;
  temp=BIT_ADDR((uint32_t)iicPort.iic_sdaPort+16,iicPort.iicSDAPortNum);
	return temp;
}

void IIC_delay(void)
{
   u8 i=3; 
   while(i) 
   { 
     i--; 
   } 
	
}

int IIC_Start(iic iicPort)
{
	SDA_OUT(iicPort);     //sda线输出
	IIC_SDA(iicPort,HIGH);	  	  
	IIC_SCL(iicPort,HIGH);
	IIC_delay();
 	IIC_SDA(iicPort,LOW);//START:when CLK is high,DATA change form high to low 
	IIC_SCL(iicPort,LOW);//钳住I2C总线，准备发送或接收数据 
	return 1;
}	

int IIC_Stop(iic iicPort)
{
	SDA_OUT(iicPort);     //sda线输出
	IIC_SCL(iicPort,LOW);
	IIC_SDA(iicPort,LOW);//STOP:when CLK is high DATA change form low to high
 	IIC_delay();
	IIC_SCL(iicPort,HIGH); 
	IIC_SDA(iicPort,HIGH);//发送I2C总线结束信号
	IIC_delay();		
	return 1;	
}

u8 IIC_Wait_Ack(iic iicPort)
{
	u8 ucErrTime=0;
	SDA_IN(iicPort);      //SDA设置为输入  
	IIC_SDA(iicPort,HIGH);   
	IIC_SCL(iicPort,HIGH);
	IIC_delay();	 
	while(IIC_READ_SDA(iicPort))
	{
		ucErrTime++;
		if(ucErrTime>200)
		{
			IIC_Stop(iicPort);
			return 1;
		}
	}
	IIC_SCL(iicPort,LOW);//时钟输出0 	   
	return 0;  
} 

void IIC_Ack(iic iicPort)
{
	IIC_SCL(iicPort,LOW);
	SDA_OUT(iicPort);     //sda线输出
	IIC_SDA(iicPort,LOW);
	IIC_SCL(iicPort,HIGH);
	IIC_delay();
	IIC_SCL(iicPort,LOW);
}

void IIC_NAck(iic iicPort)
{
	IIC_SCL(iicPort,LOW);
	SDA_OUT(iicPort);     //sda线输出
	IIC_SDA(iicPort,HIGH);
	IIC_SCL(iicPort,HIGH);
	IIC_delay();
	IIC_SCL(iicPort,LOW);
}	

//IIC发送一个字节
//返回从机有无应答
//1，有应答
//0，无应答			  
void IIC_Send_Byte(iic iicPort,u8 txd)
{                        
	u8 t;   
	SDA_OUT(iicPort);     //sda线输出 	    
	IIC_SCL(iicPort,LOW);//拉低时钟开始数据传输
	for(t=0;t<8;t++)
	{              
		IIC_SDA(iicPort,(txd&0x80)>>7);
		txd<<=1; 	   
		IIC_SCL(iicPort,HIGH);
		IIC_delay(); 
		IIC_SCL(iicPort,LOW);	
		IIC_delay();
	}	 
} 
//读1个字节，ack=1时，发送ACK，ack=0，发送nACK   
u8 IIC_Read_Byte(iic iicPort, unsigned char ack)
{
	unsigned char i,receive=0;
	SDA_IN(iicPort);      //SDA设置为输入 
	for(i=0;i<8;i++ )
	{
		IIC_SCL(iicPort,LOW); 
		IIC_delay();
		IIC_SCL(iicPort,HIGH);
		receive<<=1;
		if(IIC_READ_SDA(iicPort))receive++;   
		IIC_delay(); 
	}					 
    if (!ack)
       IIC_NAck(iicPort);//发送nACK
    else
       IIC_Ack(iicPort); //发送ACK   
    return receive;
}

