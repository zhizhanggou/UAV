#ifndef _TASK_H
#define _TASK_H
#include "includes.h"
#define ifSensorsReadFinish 1<<0
#define MAIN_LOOP_TIME 5    //��ѭ��ʱ�䣬��λms

extern EventGroupHandle_t xCreatedEventGroup;
extern TaskHandle_t xTaskAttitudeAlgorithm;
void vTaskDataUpload(void *pvParameters);
void vTaskAttitudeAlgorithm(void *pvParameters);
void vTaskReadSenser(void *pvParameters);
void vTaskIndicatorLED(void *pvParameters);
#endif
