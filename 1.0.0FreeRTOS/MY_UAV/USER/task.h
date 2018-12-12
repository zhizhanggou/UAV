#ifndef _TASK_H
#define _TASK_H
#include "includes.h"
#define BIT_0 1<<0

extern EventGroupHandle_t xCreatedEventGroup;

void vTaskDataUpload(void *pvParameters);
void vTaskAttitudeAlgorithm(void *pvParameters);
void vTaskReadSenser(void *pvParameters);
void vTaskIndicatorLED(void *pvParameters);
#endif
