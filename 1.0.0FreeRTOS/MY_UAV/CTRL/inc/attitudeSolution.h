#ifndef __ATTITUDESOLUTION_H
#define __ATTITUDESOLUTION_H

#include "includes.h"

//void NonlinearSO3AHRSupdate(DataProcessed data, float twoKp, float twoKi, float dt);

typedef struct 
{
   Axis3f attitude;
   float height;
}FlightStatus;



extern float Vertically_ACCEL;
extern float Vertically_Velocity,Vertically_Velocity_Last,Vertically_Altitude,Vertically_ACCEL_Dirft; //��ֱ�����ٶ�,�߶�
extern float accel_temp[3],gyro_temp[3];
extern u8 Vertically_Adjustment_Flag;

//float Yaw_Calc(float Bx,float By,float Bz,float Roll,float Pitch);
//extern DataProcessed data;

#endif
