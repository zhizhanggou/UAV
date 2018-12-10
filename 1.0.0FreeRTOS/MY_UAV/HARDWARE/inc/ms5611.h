#ifndef __MS5611_H
#define __MS5611_H

#include "sys.h"


#define DEV_ADDR 		0XEE
#define RESET_COM 	0X1E
#define COV_COM			0X48
#define ADC_READ		0X00
#define MS5611_PROM_BASE_ADDR 0XA0
#define MS5611_PROM_REG_SIZE 2 // size in bytes of a prom registry.
#define MS5611_PROM_REG_COUNT 6 // number of registers in the PROM 

// OSR (Over Sampling Ratio) constants  
#define MS561101BA_OSR_256 0x00  
#define MS561101BA_OSR_512 0x02  
#define MS561101BA_OSR_1024 0x04  
#define MS561101BA_OSR_2048 0x06  
#define MS561101BA_OSR_4096 0x08 
#define MS561101BA_D1 0x40
#define MS561101BA_D2 0x50
//#define  MS561101BA_D1_OSR_256 0x40   
//#define  MS561101BA_D1_OSR_512 0x42   
//#define  MS561101BA_D1_OSR_1024 0x44   
//#define  MS561101BA_D1_OSR_2048 0x46   
#define  MS561101BA_D1_OSR_4096 0x48   
  
//#define  MS561101BA_D2_OSR_256 0x50   
//#define  MS561101BA_D2_OSR_512 0x52   
//#define  MS561101BA_D2_OSR_1024 0x54   
//#define  MS561101BA_D2_OSR_2048 0x56   
#define  MS561101BA_D2_OSR_4096 0x58  

#define MS5611_SDA_IN()  {GPIOD->MODER&=~(3<<(1*2));GPIOD->MODER|=0<<1*2;}	//PB9输入模式
#define MS5611_SDA_OUT() {GPIOD->MODER&=~(3<<(1*2));GPIOD->MODER|=1<<1*2;} //PB9输出模式
//IO操作函数	 
#define MS5611_IIC_SCL    PDout(0) //SCL
#define MS5611_IIC_SDA    PDout(1) //SDA	 
#define MS5611_READ_SDA   PDin(1)  //输入SDA 

extern volatile float MS5611_Temperature,MS5611_Pressure,MS5611_Altitude,MS5611_VerticalSpeed;



#define MS5611_ADDR         0xEE// 0x77     // default I2C address


// registers of the device
#define MS561101BA_D1 0x40
#define MS561101BA_D2 0x50
#define MS561101BA_RESET 0x1E

// D1 and D2 result size (bytes)
#define MS561101BA_D1D2_SIZE 3

// OSR (Over Sampling Ratio) constants
#define MS561101BA_OSR_256 0x00  //Conversion time 0.6ms  Resolution 0.065mbar
#define MS561101BA_OSR_512 0x02  //Conversion time 1.2ms  Resolution 0.042mbar
#define MS561101BA_OSR_1024 0x04 //Conversion time 2.3ms  Resolution 0.027mbar
#define MS561101BA_OSR_2048 0x06 //Conversion time 4.6ms  Resolution 0.018mbar
#define MS561101BA_OSR_4096 0x08 //Conversion time 9.1ms  Resolution 0.012mbar

#define MS561101BA_PROM_BASE_ADDR 0xA2 // by adding ints from 0 to 6 we can read all the prom configuration values. 
// C1 will be at 0xA2 and all the subsequent are multiples of 2
#define MS561101BA_PROM_REG_COUNT 6 // number of registers in the PROM
#define MS561101BA_PROM_REG_SIZE 2 // size in bytes of a prom registry.

//Other
#define MSLP                    101325          // Mean Sea Level Pressure = 1013.25 hPA (1hPa = 100Pa = 1mbar)



extern volatile float MS5611_Temperature,MS5611_Pressure,MS5611_Altitude,MS5611_VerticalSpeed;

extern uint8_t Baro_ALT_Updated;
extern uint8_t paOffsetInited;





void MS5611_Init(void);
uint8_t  WaitBaroInitOffset(void);
void MS5611_ThreadNew(void) ;

void MS561101BA_reset(void) ;
void MS561101BA_readPROM(void) ;
void MS561101BA_startConversion(uint8_t command) ;
uint32_t MS561101BA_getConversion(void) ;


void MS5611_Init(void);
int MS5611_IIC_Start(void);				//发送IIC开始信号
int MS5611_IIC_Stop(void);	  			//发送IIC停止信号
void MS5611_IIC_Send_Byte(u8 txd);			//IIC发送一个字节
u8 MS5611_IIC_Read_Byte(unsigned char ack);//IIC读取一个字节
u8 MS5611_IIC_Wait_Ack(void); 				//IIC等待ACK信号
void MS5611_IIC_Ack(void);					//IIC发送ACK信号
void MS5611_IIC_NAck(void);				//IIC不发送ACK信号
u8 MS5611_IIC_Read(u8 Slaveaddress,u8 REG_Address);
void MS5611_IIC_Write(u8 Slaveaddress,u8 REG_Address,u8 REG_data);
void I2C_NoAddr_WriteByte(unsigned char DeviceAddr,unsigned char info);  
uint16_t I2C_Read_2Bytes(unsigned char DeviceAddr,unsigned char address);
uint32_t I2C_Read_3Bytes(unsigned char DeviceAddr,unsigned char address);   

#endif




