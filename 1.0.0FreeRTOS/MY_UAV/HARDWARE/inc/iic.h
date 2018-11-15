#ifndef _IIC_H
#define _IIC_H
#include "sys.h"

typedef struct iic{
   
	uint16_t sclPort;
	uint16_t sdaPort;  
	uint16_t iicSCLPortNum;  
	uint16_t iicSDAPortNum;  
	GPIO_TypeDef * iic_Port;  
   
}iic;
typedef enum GPIO_level{
	LOW=0,
   HIGH=1 
}GPIO_level;

void iicInit(void);
int IIC_Start(iic iicPort);
int IIC_Stop(iic iicPort);
u8 IIC_Wait_Ack(iic iicPort);
void IIC_Ack(iic iicPort);
void IIC_NAck(iic iicPort);
u8 IIC_Read_Byte(iic iicPort, unsigned char ack);
void IIC_Send_Byte(iic iicPort,u8 txd);
#endif

