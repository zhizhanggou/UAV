#ifndef _SENSORS_H
#define _SENSORS_H
#include "includes.h"

   
#define MAG_SENSOR_ENABLE 1                  //�Ƿ�ʹ�ô�����
#define GYRO_BIAS_SAMPLES_NUM 300             //�����ǵ���ƫ������������
#define GYRO_BIAS_SAMPLES_VARIANCE_GATA 200     //��������ƫ����ʱ��������ݷ�������ֵ



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
void getGyroBiasMeanAndVar(Axis3i16 *buffer);   //����ƽ��ֵ�ͷ���
#endif
