#ifndef __ULTRASONIC_H
#define __ULTRASONIC_H
#include "includes.h"


#define GY_US42_START_CMD 0X51
#define GY_US42_ADDR 0XE0
#define GY_US42_DATA_READY 0
#define GY_US42_INT PAin(6)

void GY_US42_Init(void);
void GY_US42_StartMeasure(void);
void GY_US42_ReadData(void);

extern int16_t ulterDistance;

#endif
