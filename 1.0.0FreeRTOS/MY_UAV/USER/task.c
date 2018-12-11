#include "task.h"
EventGroupHandle_t xCreatedEventGroup;

void vTaskAttitudeAlgorithm(void *pvParameters)
{
   while(1)
   {
      READ_MPU9250_MAG();
      xEventGroupSetBits(xCreatedEventGroup,BIT_0);  //设置事件标志组的BIT0
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
      
   }
}










/*******************参数上传上位机任务******************/
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
         mpu9250_data.ACC_X=mpu9250_OriginalData.ACC_X;
         mpu9250_data.ACC_Y=mpu9250_OriginalData.ACC_Y;
         mpu9250_data.ACC_Z=mpu9250_OriginalData.ACC_Z;
         mpu9250_data.GYRO_X=mpu9250_OriginalData.GYRO_X;
         mpu9250_data.GYRO_Y=mpu9250_OriginalData.GYRO_Y;
         mpu9250_data.GYRO_Z=mpu9250_OriginalData.GYRO_Z;
         mpu9250_data.MAG_X=mpu9250_OriginalData.MAG_X;
         mpu9250_data.MAG_Y=mpu9250_OriginalData.MAG_Y;
         mpu9250_data.MAG_Z=mpu9250_OriginalData.MAG_Z;
      
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