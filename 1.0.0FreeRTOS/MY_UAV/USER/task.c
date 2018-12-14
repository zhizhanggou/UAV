#include "task.h"
#include "flightStatus.h"

EventGroupHandle_t xCreatedEventGroup;
TaskHandle_t xTaskAttitudeAlgorithm = NULL;
/******************************************
姿态解算任务
******************************************/

int deltaT;//两次传感器数据读取的时间差

void vTaskAttitudeAlgorithm(void *pvParameters)
{
   EventBits_t uxBits;
	DataProcessed data;
   while(1)
   {
//      uxBits = xEventGroupWaitBits(xCreatedEventGroup, /* 事件标志组句柄 */
//                                 ifSensorsReadFinish, /* 等待 bit0 被设置 */
//                                 pdTRUE, /* 退出前 bit0 和 bit1 被清除 */
//                                 pdTRUE, /* 设置为 pdTRUE 表示等待 bit1 和 bit0 都被设置*/
//                                 portMAX_DELAY); /* 等待延迟时间 */
//      if((uxBits & ifSensorsReadFinish)==ifSensorsReadFinish)
//      {
//				
//			
//				
//			
//      }
      NonlinearSO3AHRSupdate(data, 1, 0.005, deltaT/1000000.0f);
     
      xQueueReceive(gyroDataQueue, data.gyroDataProcessed, 0);
     
      xQueueReceive(accDataQueue, data.accDataProcessed, 0);
      //xQueueReceive(gyroDataQueue, data.gyroDataProcessed, 0);
      
      
      vTaskSuspend(xTaskAttitudeAlgorithm);
      //xEventGroupSetBits(xCreatedEventGroup,BIT_0);  //设置事件标志组的BIT0
//      vTaskDelay(20);
   }
   
}




/******************************************
传感器读取任务
******************************************/
void vTaskReadSenser(void *pvParameters)
{
	u32 lastWakeTime = xTaskGetTickCount();
   while(1)
   {
      vTaskDelayUntil(&lastWakeTime,MAIN_LOOP_TIME);
			deltaT=__HAL_TIM_GetCounter(&TIM6_Handler);//读取定时器6的时间
			READ_MPU9250_ACCEL();
			READ_MPU9250_GYRO();
			__HAL_TIM_SetCounter(&TIM6_Handler,0x00);	
				//READ_MPU9250_MAG();
			if(isGetGyroBiasFinished==false)
			 getGyroBias(gyroOriginalData);
			else if(isGetGyroBiasFinished)
			{
        imuOriginalDataProcessing(accOriginalData,gyroOriginalData,magOriginalData);

        vTaskSuspendAll();	/*确保同一时刻把数据放入队列中*/
        xQueueOverwrite(accDataQueue,dataProcessed.accDataProcessed);
        xQueueOverwrite(gyroDataQueue,dataProcessed.gyroDataProcessed);
        xTaskResumeAll();
        vTaskResume(xTaskAttitudeAlgorithm);
			}
			//xEventGroupSetBits(xCreatedEventGroup,ifSensorsReadFinish);  //设置事件标志组的BIT0
				
				
				
		 //   xQueueOverwrite(imuDataQueue,&mpu9250_OriginalData);
				
      
      vTaskDelay(1);
      
   }
}











/******************************************
参数上传上位机任务
******************************************/
HMI_data mpu9250_data;
void vTaskDataUpload(void *pvParameters)
{

   while(1)
   {
//      uxBits = xEventGroupWaitBits(xCreatedEventGroup, /* 事件标志组句柄 */
//                                 ifSensorsReadFinish, /* 等待 bit0 被设置 */
//                                 pdTRUE, /* 退出前 bit0 和 bit1 被清除 */
//                                 pdTRUE, /* 设置为 pdTRUE 表示等待 bit1 和 bit0 都被设置*/
//                                 portMAX_DELAY); /* 等待延迟时间 */

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

