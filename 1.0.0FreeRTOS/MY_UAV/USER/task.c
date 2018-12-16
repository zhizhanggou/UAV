#include "task.h"
#include "flightStatus.h"

EventGroupHandle_t xUploadEventGroup;
EventGroupHandle_t xSensorEventGroup;
TaskHandle_t xTaskAttitudeAlgorithm = NULL;
uint8_t isSensorFirstRead=0;
/******************************************
姿态解算任务
******************************************/

int deltaT;//两次传感器数据读取的时间差

void vTaskAttitudeAlgorithm(void *pvParameters)
{
  TickType_t lastWakeTime = xTaskGetTickCount();
  
	DataProcessed data;
   while(1)
   { 
      vTaskDelayUntil(&lastWakeTime,MAIN_LOOP_TIME);
      if(isGetGyroBiasFinished) 
      {
        
        if(isSensorFirstRead)
        {
          xQueueReceive(accDataQueue, data.accDataProcessed, 0);
          xQueueReceive(gyroDataQueue, data.gyroDataProcessed, 0);
          NonlinearSO3AHRSupdate(data, 1, 0.005, deltaT/1000000.0f);
        }
          
      }

     
     
      xEventGroupSetBits(xSensorEventGroup,ifSensorsReadFinish);  //设置事件标志组的BIT0
      xEventGroupSetBits(xUploadEventGroup,ifDataReadyUpLoad1);  //设置事件标志组的BIT0
      

   }
   
}




/******************************************
传感器读取任务
******************************************/
void vTaskReadSenser(void *pvParameters)
{
    EventBits_t uxBits;
   while(1)
   {	
      uxBits = xEventGroupWaitBits(xSensorEventGroup, /* 事件标志组句柄 */
                                 ifSensorsReadFinish, /* 等待 bit0 被设置 */
                                 pdTRUE, /* 退出前 bit0 和 bit1 被清除 */
                                 pdTRUE, /* 设置为 pdTRUE 表示等待 bit1 和 bit0 都被设置*/
                                 portMAX_DELAY); /* 等待延迟时间 */
     if((uxBits & ifSensorsReadFinish)==ifSensorsReadFinish)
     {
        deltaT=__HAL_TIM_GetCounter(&TIM6_Handler);//读取定时器6的时间
        __HAL_TIM_SetCounter(&TIM6_Handler,0x00);	
        READ_MPU9250_ACCEL();
        READ_MPU9250_GYRO();
        
          //READ_MPU9250_MAG();
        if(isGetGyroBiasFinished==false)
         getGyroBias(gyroOriginalData);
        else if(isGetGyroBiasFinished)
        {
          imuOriginalDataProcessing(accOriginalData,gyroOriginalData,magOriginalData);
  //        NonlinearSO3AHRSupdate(dataProcessed, 1, 0.005, 0.005);
  //        xEventGroupSetBits(xCreatedEventGroup,ifSensorsReadFinish);  //设置事件标志组的BIT0
          //NonlinearSO3AHRSupdate(dataProcessed, 1, 0.005, deltaT/1000000.0f);
          vTaskSuspendAll();	/*确保同一时刻把数据放入队列中*/
          xQueueOverwrite(accDataQueue,dataProcessed.accDataProcessed);
          xQueueOverwrite(gyroDataQueue,dataProcessed.gyroDataProcessed);
          if(MAG_SENSOR_ENABLE)
          {
            xQueueOverwrite(magDataQueue,&dataProcessed.magDataProcessed);
          }
          isSensorFirstRead=1;
          xTaskResumeAll();
          
        }
        xEventGroupSetBits(xUploadEventGroup,ifDataReadyUpLoad2);  //设置事件标志组的BIT0
      }
   
   }
}











/******************************************
参数上传上位机任务
******************************************/
HMI_data mpu9250_data;
void vTaskDataUpload(void *pvParameters)
{
   EventBits_t uxBits;

   while(1)
   {
      uxBits = xEventGroupWaitBits(xUploadEventGroup, /* 事件标志组句柄 */
                                 ifDataReadyUpLoad, /* 等待 bit0 被设置 */
                                 pdTRUE, /* 退出前 bit0 和 bit1 被清除 */
                                 pdTRUE, /* 设置为 pdTRUE 表示等待 bit1 和 bit0 都被设置*/
                                 portMAX_DELAY); /* 等待延迟时间 */

     if((uxBits & ifDataReadyUpLoad)==ifDataReadyUpLoad)
     {
        mpu9250_data.ACC_X=accOriginalData.x;
        mpu9250_data.ACC_Y=accOriginalData.y;
        mpu9250_data.ACC_Z=accOriginalData.z;
        if(isGetGyroBiasFinished)
        {
           mpu9250_data.GYRO_X=gyroOriginalData.x-gyroBias.value.x;
           mpu9250_data.GYRO_Y=gyroOriginalData.y-gyroBias.value.y;
           mpu9250_data.GYRO_Z=gyroOriginalData.z-gyroBias.value.z;
        }
        else
        {
           mpu9250_data.GYRO_X=gyroOriginalData.x;
           mpu9250_data.GYRO_Y=gyroOriginalData.y;
           mpu9250_data.GYRO_Z=gyroOriginalData.z;
        }
        mpu9250_data.MAG_X=magOriginalData.x;
        mpu9250_data.MAG_Y=magOriginalData.y;
        mpu9250_data.MAG_Z=magOriginalData.z;
     
        Send_RCData(mpu9250_data,flightStatus.attitude.x,flightStatus.attitude.y,flightStatus.attitude.z,0,0,0);
    }
     while(1)
    {
       if(__HAL_DMA_GET_FLAG(&UART1TxDMA_Handler,DMA_FLAG_TCIF3_7))  //判断是否传输完成
       {
          __HAL_DMA_CLEAR_FLAG(&UART1TxDMA_Handler,DMA_FLAG_TCIF3_7);//清除DMA2_Steam7传输完成标志
          HAL_UART_DMAStop(&UART1_Handler);      //传输完成以后关闭串口DMA
          break;
       }    
    }
      
      
   }
}

/******************************************
指示灯任务
******************************************/
void vTaskIndicatorLED(void *pvParameters)
{
   while(1)
   {
       if(isGetGyroBiasFinished==false)
          ledFlash(200,200);
   }
}

