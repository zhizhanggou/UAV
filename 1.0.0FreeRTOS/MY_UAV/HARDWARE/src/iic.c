#include "iic.h"
#include "includes.h"

iic mpu9250_iic;
iic ms5611_iic;

/***********����IIC�˿�**********/

iic iicNo1PortDef()
{
   iic iicNo1;
   iicNo1.sclPort=GPIO_PIN_3;
   iicNo1.sdaPort=GPIO_PIN_4;
   iicNo1.iic_Port=GPIOD;
	iicNo1.iicSCLPortNum=3;
	iicNo1.iicSDAPortNum=4;
	mpu9250_iic=iicNo1;    //���ӿ���Ϣ����MPU9250
   __HAL_RCC_GPIOD_CLK_ENABLE();           
   return iicNo1;
}
iic iicNo2PortDef()
{
   iic iicNo2;
   iicNo2.sclPort=GPIO_PIN_0;
   iicNo2.sdaPort=GPIO_PIN_1;
   iicNo2.iic_Port=GPIOD;  
	iicNo2.iicSCLPortNum=0;
	iicNo2.iicSDAPortNum=1;
	ms5611_iic=iicNo2;		//���ӿ���Ϣ����MS5611
   __HAL_RCC_GPIOD_CLK_ENABLE();         
   return iicNo2;
}
/*******************************/




void iicSingleInit(iic iicPort)
{
   GPIO_InitTypeDef GPIO_Initure;
   
   GPIO_Initure.Pin = iicPort.sclPort | iicPort.sdaPort;
   GPIO_Initure.Mode = GPIO_MODE_OUTPUT_PP;//�������
   GPIO_Initure.Speed = GPIO_SPEED_HIGH;//100MHz
   GPIO_Initure.Pull = GPIO_PULLUP;//����
   HAL_GPIO_Init(iicPort.iic_Port,&GPIO_Initure);
}
void iicInit()
{
   iicSingleInit(iicNo1PortDef());
	iicSingleInit(iicNo2PortDef());
}



void SDA_IN(iic iicPort)  
{
	iicPort.iic_Port->MODER&=~(3<<(iicPort.iicSDAPortNum*2));
	iicPort.iic_Port->MODER|=0<<iicPort.iicSDAPortNum*2;
}	
void SDA_OUT(iic iicPort)  
{
	iicPort.iic_Port->MODER&=~(3<<(iicPort.iicSDAPortNum*2));
	iicPort.iic_Port->MODER|=1<<iicPort.iicSDAPortNum*2;
}	

void IIC_SCL(iic iicPort,bool level)
{
	switch((uint32_t)iicPort.iic_Port)
	{
		case (uint32_t)GPIOA: PAout(iicPort.iicSCLPortNum)=level;
		case (uint32_t)GPIOB: PBout(iicPort.iicSCLPortNum)=level; 
		case (uint32_t)GPIOC: PCout(iicPort.iicSCLPortNum)=level;
		case (uint32_t)GPIOD: PDout(iicPort.iicSCLPortNum)=level; 
		case (uint32_t)GPIOE: PEout(iicPort.iicSCLPortNum)=level; 
		case (uint32_t)GPIOF: PFout(iicPort.iicSCLPortNum)=level; 
		case (uint32_t)GPIOG: PGout(iicPort.iicSCLPortNum)=level; 
	}
}

void IIC_SDA(iic iicPort,bool level)
{
	switch((uint32_t)iicPort.iic_Port)
	{
		case (uint32_t)GPIOA: PAout(iicPort.iicSDAPortNum)=level; 
		case (uint32_t)GPIOB: PBout(iicPort.iicSDAPortNum)=level; 
		case (uint32_t)GPIOC: PCout(iicPort.iicSDAPortNum)=level; 
		case (uint32_t)GPIOD: PDout(iicPort.iicSDAPortNum)=level;
		case (uint32_t)GPIOE: PEout(iicPort.iicSDAPortNum)=level;
		case (uint32_t)GPIOF: PFout(iicPort.iicSDAPortNum)=level; 
		case (uint32_t)GPIOG: PGout(iicPort.iicSDAPortNum)=level; 
	}
}
uint8_t IIC_READ_SDA(iic iicPort)
{
	uint8_t temp=0;
		switch((uint32_t)iicPort.iic_Port)
	{
		case (uint32_t)GPIOA: temp=PAin(iicPort.iicSDAPortNum);
		case (uint32_t)GPIOB: temp=PBin(iicPort.iicSDAPortNum);
		case (uint32_t)GPIOC: temp=PCin(iicPort.iicSDAPortNum);
		case (uint32_t)GPIOD: temp=PDin(iicPort.iicSDAPortNum);
		case (uint32_t)GPIOE: temp=PEin(iicPort.iicSDAPortNum); 
		case (uint32_t)GPIOF: temp=PFin(iicPort.iicSDAPortNum);
		case (uint32_t)GPIOG: temp=PGin(iicPort.iicSDAPortNum);
	}
	return temp;
}

void IIC_delay(void)
{
   u8 i=20; 
   while(i) 
   { 
     i--; 
   } 
	
}

int IIC_Start(iic iicPort)
{
	SDA_OUT(iicPort);     //sda�����
	IIC_SDA(iicPort,HIGH);	  	  
	IIC_SCL(iicPort,HIGH);
	IIC_delay();
 	IIC_SDA(iicPort,LOW);//START:when CLK is high,DATA change form high to low 
	IIC_delay();
	IIC_SCL(iicPort,LOW);//ǯסI2C���ߣ�׼�����ͻ�������� 
	return 1;
}	

int IIC_Stop(iic iicPort)
{
	SDA_OUT(iicPort);     //sda�����
	IIC_SCL(iicPort,LOW);
	IIC_SDA(iicPort,LOW);//STOP:when CLK is high DATA change form low to high
 	IIC_delay();
	IIC_SCL(iicPort,HIGH); 
	IIC_SDA(iicPort,HIGH);//����I2C���߽����ź�
	IIC_delay();		
	return 1;	
}

u8 IIC_Wait_Ack(iic iicPort)
{
	u8 ucErrTime=0;
	SDA_IN(iicPort);      //SDA����Ϊ����  
	IIC_SDA(iicPort,HIGH);
	IIC_delay();	   
	IIC_SCL(iicPort,HIGH);
	IIC_delay();	 
	while(IIC_READ_SDA(iicPort))
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			IIC_Stop(iicPort);
			return 1;
		}
	}
	IIC_SCL(iicPort,LOW);//ʱ�����0 	   
	return 0;  
} 

void IIC_Ack(iic iicPort)
{
	IIC_SCL(iicPort,LOW);
	SDA_OUT(iicPort);     //sda�����
	IIC_SDA(iicPort,LOW);
	IIC_delay();
	IIC_SCL(iicPort,HIGH);
	IIC_delay();
	IIC_SCL(iicPort,LOW);
}

void IIC_NAck(iic iicPort)
{
	IIC_SCL(iicPort,LOW);
	SDA_OUT(iicPort);     //sda�����
	IIC_SDA(iicPort,HIGH);
	IIC_delay();
	IIC_SCL(iicPort,HIGH);
	IIC_delay();
	IIC_SCL(iicPort,LOW);
}	

//IIC����һ���ֽ�
//���شӻ�����Ӧ��
//1����Ӧ��
//0����Ӧ��			  
void IIC_Send_Byte(iic iicPort,u8 txd)
{                        
	u8 t;   
	SDA_OUT(iicPort);     //sda����� 	    
	IIC_SCL(iicPort,LOW);//����ʱ�ӿ�ʼ���ݴ���
	for(t=0;t<8;t++)
	{              
		IIC_SDA(iicPort,(txd&0x80)>>7);
		txd<<=1; 	  
		IIC_delay();  
		IIC_SCL(iicPort,HIGH);
		IIC_delay(); 
		IIC_SCL(iicPort,LOW);	
		IIC_delay();
	}	 
} 
//��1���ֽڣ�ack=1ʱ������ACK��ack=0������nACK   
u8 IIC_Read_Byte(iic iicPort, unsigned char ack)
{
	unsigned char i,receive=0;
	SDA_IN(iicPort);      //SDA����Ϊ���� 
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
       IIC_NAck(iicPort);//����nACK
    else
       IIC_Ack(iicPort); //����ACK   
    return receive;
}

