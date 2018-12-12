#ifndef _SENSORS_H
#define _SENSORS_H
#include "includes.h"

   
#define MAG_SENSOR_ENABLE 1                  //是否使用磁力计
#define GYRO_BIAS_SAMPLES_NUM 300             //陀螺仪的零偏采样样本个数
#define GYRO_BIAS_SAMPLES_VARIANCE_GATA 200     //陀螺仪零偏采样时允许的数据方差的最大值
#define GYRO_GAIN 0.03052f   //陀螺仪转换系数   （1000/32767）
#define ACC_GAIN 0.002394f   //加速度计转换系数 （8*9.8/32767）





void sensorsQueueInit(void);
bool getGyroBias(Axis3i16 data);
void getGyroBiasMeanAndVar(Axis3i16 *buffer);   //计算平均值和方差


extern xQueueHandle accDataQueue;
extern xQueueHandle gyroDataQueue;
extern xQueueHandle magDataQueue;
extern xQueueHandle baroDataQueue;
extern bool isBufferFulled,isGetGyroBiasFinished;
extern GyroBias gyroBias;
#endif
