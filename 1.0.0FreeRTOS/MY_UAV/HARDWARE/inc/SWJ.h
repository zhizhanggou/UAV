#ifndef __SWJ_H
#define __SWJ_H		
#include "includes.h"
//#include "RC.h"
//#define RECEIVE_BUF_SIZE 100
#define USER_DATA 1
#define STATUS_DATA 0

typedef struct HMI
{
		int ACC_X;
		int ACC_Y;
		int ACC_Z;
		int GYRO_X;
		int GYRO_Y;
		int GYRO_Z;
		int MAG_X;
		int MAG_Y;
		int MAG_Z;
}HMI_data;

typedef struct USER
{
		float DATA1;
		float DATA2;
		float DATA3;
		float DATA4;
		float DATA5;
		float DATA6;
		float DATA7;
		float DATA8;
		float DATA9;
}USER;


extern u8 DataToSend; 
extern USER User_Data;
//void ANO_DT_Send_Status(float angle_rol, float angle_pit, float angle_yaw, s32 alt, u8 fly_model, u8 armed);
void Send_RCData(HMI_data data,float angle_rol, float angle_pit, float angle_yaw, s32 alt, u8 fly_model, u8 armed);
void Uart1_Put_Buf(unsigned char *DataToSend , u8 data_num);
void Send_USERDATA(USER data);
#endif
