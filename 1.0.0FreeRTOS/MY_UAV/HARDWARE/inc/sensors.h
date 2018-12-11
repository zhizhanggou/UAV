#ifndef _SENSORS_H
#define _SENSORS_H
#include "includes.h"

   
#define MAG_SENSOR_ENABLE 1                  //是否使用磁力计
#define GYRO_BIAS_SAMPLES_NUM 300             //陀螺仪的零偏采样样本个数
#define GYRO_BIAS_SAMPLES_VARIANCE_GATA 200     //陀螺仪零偏采样时允许的数据方差的最大值



typedef struct 
{
   float x;
   float y;
   float z;
}Axis3f;

typedef struct 
{
   Axis3f variance;
   Axis3f value;
}GyroBias;

typedef struct 
{
   int16_t x;
   int16_t y;
   int16_t z;
}Axis3i16;

void sensorsQueueInit(void);
void getGyroBiasMeanAndVar(Axis3i16 *buffer);   //计算平均值和方差
#endif
