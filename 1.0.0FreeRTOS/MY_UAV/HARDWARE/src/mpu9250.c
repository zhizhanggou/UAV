
#include "mpu9250.h"

iic mpu9250_iic,mpu9250_mag_iic;
bool isMagDataReady=false;


Axis3i16 accOriginalData,gyroOriginalData,magOriginalData;


unsigned int ASA[3]={0};

void mpu9250_Init()
{

	iicSingleInit(mpu9250_iic=iicPortDef(GPIOD,GPIO_PIN_3,GPIOD,GPIO_PIN_4,3));
  iicSingleInit(mpu9250_mag_iic=iicPortDef(GPIOD,GPIO_PIN_3,GPIOD,GPIO_PIN_4,20));
	MPU9250_IIC_Write(mpu9250_iic,MPU9250_ADDRESS,PWR_MGMT_1, 0x00);	//解除休眠状态
	MPU9250_IIC_Write(mpu9250_iic,MPU9250_ADDRESS,SMPLRT_DIV, 0x04);
	MPU9250_IIC_Write(mpu9250_iic,MPU9250_ADDRESS,CONFIG, 0x06);      
	MPU9250_IIC_Write(mpu9250_iic,MPU9250_ADDRESS,GYRO_CONFIG, 0x10);   //陀螺仪量程为1000dps
	MPU9250_IIC_Write(mpu9250_iic,MPU9250_ADDRESS,ACCEL_CONFIG, 0x10);  //加速度计量程是8g
	
	MPU9250_IIC_Write(mpu9250_iic,MPU9250_ADDRESS,USER_CTRL,0x00);		//关闭IIC主机模式
	MPU9250_IIC_Write(mpu9250_iic,MPU9250_ADDRESS,INT_PIN_CFG, 0X02);
	MPU9250_IIC_Write(mpu9250_iic,MAG_ADDRESS,MAG_CNTL1,0x00);	
	ASA[X]=MPU9250_IIC_Read(mpu9250_iic,MAG_ADDRESS,MAG_ASAX);
	ASA[Y]=MPU9250_IIC_Read(mpu9250_iic,MAG_ADDRESS,MAG_ASAY);
	ASA[Z]=MPU9250_IIC_Read(mpu9250_iic,MAG_ADDRESS,MAG_ASAZ);
	delay_ms(50);
	MPU9250_IIC_Write(mpu9250_iic,MAG_ADDRESS,MAG_CNTL1,0x01);		//设置AK8963为单次测量模式
}




u8 MPU9250_IIC_Read(iic iicPort,u8 Slaveaddress,u8 REG_Address)
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

void MPU9250_IIC_Write(iic iicPort,u8 Slaveaddress,u8 REG_Address,u8 REG_data)
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

void READ_MPU9250_ACCEL(void)
{   
	accOriginalData.x=(MPU9250_IIC_Read(mpu9250_iic,MPU9250_ADDRESS,ACCEL_XOUT_L) | (MPU9250_IIC_Read(mpu9250_iic,MPU9250_ADDRESS,ACCEL_XOUT_H)<<8)); 
	accOriginalData.y=(MPU9250_IIC_Read(mpu9250_iic,MPU9250_ADDRESS,ACCEL_YOUT_L) | (MPU9250_IIC_Read(mpu9250_iic,MPU9250_ADDRESS,ACCEL_YOUT_H)<<8)); 
	accOriginalData.z=(MPU9250_IIC_Read(mpu9250_iic,MPU9250_ADDRESS,ACCEL_ZOUT_L) | (MPU9250_IIC_Read(mpu9250_iic,MPU9250_ADDRESS,ACCEL_ZOUT_H)<<8)); 	
}
void READ_MPU9250_GYRO(void)
{ 
	gyroOriginalData.x=(MPU9250_IIC_Read(mpu9250_iic,MPU9250_ADDRESS,GYRO_XOUT_L) | (MPU9250_IIC_Read(mpu9250_iic,MPU9250_ADDRESS,GYRO_XOUT_H)<<8)); 
	gyroOriginalData.y=(MPU9250_IIC_Read(mpu9250_iic,MPU9250_ADDRESS,GYRO_YOUT_L) | (MPU9250_IIC_Read(mpu9250_iic,MPU9250_ADDRESS,GYRO_YOUT_H)<<8)); 
	gyroOriginalData.z=(MPU9250_IIC_Read(mpu9250_iic,MPU9250_ADDRESS,GYRO_ZOUT_L) | (MPU9250_IIC_Read(mpu9250_iic,MPU9250_ADDRESS,GYRO_ZOUT_H)<<8)); 
}
void READ_MPU9250_MAG(void)
{ 
	magOriginalData.x=(MPU9250_IIC_Read(mpu9250_mag_iic,MAG_ADDRESS,MAG_XOUT_L) | (MPU9250_IIC_Read(mpu9250_mag_iic,MAG_ADDRESS,MAG_XOUT_H)<<8)); 
	magOriginalData.y=(MPU9250_IIC_Read(mpu9250_mag_iic,MAG_ADDRESS,MAG_YOUT_L) | (MPU9250_IIC_Read(mpu9250_mag_iic,MAG_ADDRESS,MAG_YOUT_H)<<8)); 
	magOriginalData.z=(MPU9250_IIC_Read(mpu9250_mag_iic,MAG_ADDRESS,MAG_ZOUT_L) | (MPU9250_IIC_Read(mpu9250_mag_iic,MAG_ADDRESS,MAG_ZOUT_H)<<8));
	MPU9250_IIC_Write(mpu9250_iic,MAG_ADDRESS,MAG_CNTL1,0X01);		//设置AK8963为单次测量模式 14bit精度
	isMagDataReady=true;
}

