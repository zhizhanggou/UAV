#ifndef _TASK_H
#define _TASK_H
#include "includes.h"
#define ifSensorsReadFinish 1<<0
#define ifDataReadyUpLoad1 1<<0
#define ifDataReadyUpLoad2 1<<1
#define ifDataReadyUpLoad   (ifDataReadyUpLoad1|ifDataReadyUpLoad2)
#define MAIN_LOOP_TIME 2    //��ѭ��ʱ�䣬��λms
#define MAG_READ_LOOP_TIME 20  //�����ƶ�ȡ���ڣ���λms

extern EventGroupHandle_t xUploadEventGroup,xSensorEventGroup;

void vTaskDataUpload(void *pvParameters);
void vTaskAttitudeAlgorithm(void *pvParameters);
void vTaskReadSenser(void *pvParameters);
void vTaskIndicatorLED(void *pvParameters);
#endif
