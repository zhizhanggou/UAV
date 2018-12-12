#include "task.h"
EventGroupHandle_t xCreatedEventGroup;

void vTaskAttitudeAlgorithm(void *pvParameters)
{
   while(1)
   {
      //READ_MPU9250_MAG();
      //xEventGroupSetBits(xCreatedEventGroup,BIT_0);  //设置事件标志组的BIT0
      vTaskDelay(20);
   }
   
}




/******************************************
传感器读取任务
******************************************/
void vTaskReadSenser(void *pvParameters)
{
   while(1)
   {
      READ_MPU9250_ACCEL();
      READ_MPU9250_GYRO();
      
      if(isGetGyroBiasFinished==false)
         getGyroBias(gyroOriginalData);
      else if(isGetGyroBiasFinished)
      {
         
      }
      xEventGroupSetBits(xCreatedEventGroup,BIT_0);  //设置事件标志组的BIT0
      
      vTaskSuspendAll();	/*确保同一时刻把数据放入队列中*/
      
   //   xQueueOverwrite(imuDataQueue,&mpu9250_OriginalData);
      
      xTaskResumeAll();
      vTaskDelay(1);
      
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
      uxBits = xEventGroupWaitBits(xCreatedEventGroup, /* 事件标志组句柄 */
                                 BIT_0, /* 等待 bit0 被设置 */
                                 pdTRUE, /* 退出前 bit0 和 bit1 被清除 */
                                 pdTRUE, /* 设置为 pdTRUE 表示等待 bit1 和 bit0 都被设置*/
                                 portMAX_DELAY); /* 等待延迟时间 */
      if((uxBits & BIT_0)==BIT_0)
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
      
         Send_RCData(mpu9250_data,1,0,0,0,0,0);
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

