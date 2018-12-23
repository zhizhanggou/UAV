#ifndef _TASK_H
#define _TASK_H
#include "includes.h"
#define ifSensorsReadFinish 1<<0
#define ifDataReadyUpLoad1 1<<0
#define ifDataReadyUpLoad2 1<<1
#define ifDataReadyUpLoad   (ifDataReadyUpLoad1|ifDataReadyUpLoad2)
#define MAIN_LOOP_TIME 2    //主循环时间，单位ms
#define MAG_READ_LOOP_TIME 20  //磁力计读取周期，单位ms

extern EventGroupHandle_t xUploadEventGroup,xSensorEventGroup;

void vTaskDataUpload(void *pvParameters);
void vTaskAttitudeAlgorithm(void *pvParameters);
void vTaskReadSenser(void *pvParameters);
void vTaskIndicatorLED(void *pvParameters);
#endif
