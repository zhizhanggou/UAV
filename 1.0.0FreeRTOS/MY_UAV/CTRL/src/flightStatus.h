#ifndef _FLIGHTSTATUS_H
#define _FLIGHTSTATUS_H

#include "includes.h"

typedef struct 
{
   Axis3f attitude;
   float height;
}FlightStatus;
extern float Vertically_ACCEL;
extern float Vertically_Velocity,Vertically_Velocity_Last,Vertically_Altitude,Vertically_ACCEL_Dirft; //竖直方向速度,高度
extern float accel_temp[3],gyro_temp[3];
extern u8 Vertically_Adjustment_Flag;
void NonlinearSO3AHRSupdate(DataProcessed data, float twoKp, float twoKi, float dt);
#endif
