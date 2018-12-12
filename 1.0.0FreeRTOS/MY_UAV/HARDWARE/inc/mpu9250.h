#ifndef _MPU9250_H
#define _MPU9250_H

#include "sys.h"
#include "iic.h"
#include "includes.h"


//AK8963的内部寄存器
#define MAG_WIA					0x00	//AK8963的器件ID寄存器地址
#define MAG_ST1					0x02	//状态寄存器1
#define MAG_ST2					0x09	//状态寄存器2
#define MAG_CNTL1          	  	0X0A    
#define MAG_CNTL2            	0X0B

#define MAG_ASAX 0X10
#define MAG_ASAY 0X11
#define MAG_ASAZ 0X12

#define MAG_XOUT_L				0X03	
#define MAG_XOUT_H				0X04
#define MAG_YOUT_L				0X05
#define MAG_YOUT_H				0X06
#define MAG_ZOUT_L				0X07
#define MAG_ZOUT_H				0X08
//MPU6500的内部寄存器
#define	SMPLRT_DIV		0x19	//陀螺仪采样率，典型值：0x07(125Hz)
#define	CONFIG			0x1A	//低通滤波频率，典型值：0x06(5Hz)
#define	GYRO_CONFIG		0x1B	//陀螺仪自检及测量范围，典型值：0x18(不自检，2000deg/s)
#define	ACCEL_CONFIG	0x1C	//加速计自检、测量范围及高通滤波频率，典型值：0x01(不自检，2G，5Hz)
#define INT_PIN_CFG 0X37
#define I2C_SLV0_ADDR 0x25
#define I2C_SLV0_CTRL 0x27
#define USER_CTRL 0x6A
#define EXT_SENS_DATA_00 0x49
#define	ACCEL_XOUT_H	0x3B
#define	ACCEL_XOUT_L	0x3C
#define	ACCEL_YOUT_H	0x3D
#define	ACCEL_YOUT_L	0x3E
#define	ACCEL_ZOUT_H	0x3F
#define	ACCEL_ZOUT_L	0x40

#define	TEMP_OUT_H		0x41
#define	TEMP_OUT_L		0x42

#define	GYRO_XOUT_H		0x43
#define	GYRO_XOUT_L		0x44	
#define	GYRO_YOUT_H		0x45
#define	GYRO_YOUT_L		0x46
#define	GYRO_ZOUT_H		0x47
#define	GYRO_ZOUT_L		0x48
#define	PWR_MGMT_1		0x6B	//电源管理，典型值：0x00(正常启用)
#define	WHO_AM_I		  0x75	//IIC地址寄存器(默认数值0x68，只读)
#define	MPU9250_ADDRESS   0xD0	  //陀螺地址
#define MAG_ADDRESS    0x18   //磁场地址

typedef enum 
{
	X=0,
	Y=1,
	Z=2
}axis;

extern Axis3i16 accOriginalData,gyroOriginalData,magOriginalData;
u8 MPU9250_IIC_Read(iic iicPort,u8 Slaveaddress,u8 REG_Address);
void MPU9250_IIC_Write(iic iicPort,u8 Slaveaddress,u8 REG_Address,u8 REG_data);
void READ_MPU9250_GYRO(void);
void READ_MPU9250_ACCEL(void);
void READ_MPU9250_MAG(void);
void mpu9250_Init(void);

#endif
