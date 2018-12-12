#ifndef _SENSORS_H
#define _SENSORS_H
#include "includes.h"

   
#define MAG_SENSOR_ENABLE 1                  //�Ƿ�ʹ�ô�����
#define GYRO_BIAS_SAMPLES_NUM 300             //�����ǵ���ƫ������������
#define GYRO_BIAS_SAMPLES_VARIANCE_GATA 200     //��������ƫ����ʱ��������ݷ�������ֵ
#define GYRO_GAIN 0.03052f   //������ת��ϵ��   ��1000/32767��
#define ACC_GAIN 0.002394f   //���ٶȼ�ת��ϵ�� ��8*9.8/32767��





void sensorsQueueInit(void);
bool getGyroBias(Axis3i16 data);
void getGyroBiasMeanAndVar(Axis3i16 *buffer);   //����ƽ��ֵ�ͷ���


extern xQueueHandle accDataQueue;
extern xQueueHandle gyroDataQueue;
extern xQueueHandle magDataQueue;
extern xQueueHandle baroDataQueue;
extern bool isBufferFulled,isGetGyroBiasFinished;
extern GyroBias gyroBias;
#endif
