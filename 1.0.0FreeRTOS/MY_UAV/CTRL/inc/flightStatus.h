#ifndef __FLIGHTSTATUS_H
#define __FLIGHTSTATUS_H

//#include "includes.h"
#include "sys.h"
#include "sensorsType.h"
#include "sensors.h"
#define PI 3.14159f
#define STANDARD_ATMOSPHERIC_PRESSURE 101325.0f
typedef struct
{
  Axis3f attitude;
  float height;
}FlightStatus;


void NonlinearSO3AHRSupdate(DataProcessed data , float twoKp, float twoKi, float dt);
extern FlightStatus flightStatus;


#endif 
