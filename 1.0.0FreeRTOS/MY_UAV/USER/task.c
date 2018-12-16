#include "task.h"
#include "flightStatus.h"

EventGroupHandle_t xUploadEventGroup;
EventGroupHandle_t xSensorEventGroup;
TaskHandle_t xTaskAttitudeAlgorithm = NULL;
uint8_t isSensorFirstRead=0;
/******************************************
��̬��������
******************************************/

int deltaT;//���δ��������ݶ�ȡ��ʱ���

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

     
     
      xEventGroupSetBits(xSensorEventGroup,ifSensorsReadFinish);  //�����¼���־���BIT0
      xEventGroupSetBits(xUploadEventGroup,ifDataReadyUpLoad1);  //�����¼���־���BIT0
      

   }
   
}




/******************************************
��������ȡ����
******************************************/
void vTaskReadSenser(void *pvParameters)
{
    EventBits_t uxBits;
   while(1)
   {	
      uxBits = xEventGroupWaitBits(xSensorEventGroup, /* �¼���־���� */
                                 ifSensorsReadFinish, /* �ȴ� bit0 ������ */
                                 pdTRUE, /* �˳�ǰ bit0 �� bit1 ����� */
                                 pdTRUE, /* ����Ϊ pdTRUE ��ʾ�ȴ� bit1 �� bit0 ��������*/
                                 portMAX_DELAY); /* �ȴ��ӳ�ʱ�� */
     if((uxBits & ifSensorsReadFinish)==ifSensorsReadFinish)
     {
        deltaT=__HAL_TIM_GetCounter(&TIM6_Handler);//��ȡ��ʱ��6��ʱ��
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
  //        xEventGroupSetBits(xCreatedEventGroup,ifSensorsReadFinish);  //�����¼���־���BIT0
          //NonlinearSO3AHRSupdate(dataProcessed, 1, 0.005, deltaT/1000000.0f);
          vTaskSuspendAll();	/*ȷ��ͬһʱ�̰����ݷ��������*/
          xQueueOverwrite(accDataQueue,dataProcessed.accDataProcessed);
          xQueueOverwrite(gyroDataQueue,dataProcessed.gyroDataProcessed);
          if(MAG_SENSOR_ENABLE)
          {
            xQueueOverwrite(magDataQueue,&dataProcessed.magDataProcessed);
          }
          isSensorFirstRead=1;
          xTaskResumeAll();
          
        }
        xEventGroupSetBits(xUploadEventGroup,ifDataReadyUpLoad2);  //�����¼���־���BIT0
      }
   
   }
}











/******************************************
�����ϴ���λ������
******************************************/
HMI_data mpu9250_data;
void vTaskDataUpload(void *pvParameters)
{
   EventBits_t uxBits;

   while(1)
   {
      uxBits = xEventGroupWaitBits(xUploadEventGroup, /* �¼���־���� */
                                 ifDataReadyUpLoad, /* �ȴ� bit0 ������ */
                                 pdTRUE, /* �˳�ǰ bit0 �� bit1 ����� */
                                 pdTRUE, /* ����Ϊ pdTRUE ��ʾ�ȴ� bit1 �� bit0 ��������*/
                                 portMAX_DELAY); /* �ȴ��ӳ�ʱ�� */

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
       if(__HAL_DMA_GET_FLAG(&UART1TxDMA_Handler,DMA_FLAG_TCIF3_7))  //�ж��Ƿ������
       {
          __HAL_DMA_CLEAR_FLAG(&UART1TxDMA_Handler,DMA_FLAG_TCIF3_7);//���DMA2_Steam7������ɱ�־
          HAL_UART_DMAStop(&UART1_Handler);      //��������Ժ�رմ���DMA
          break;
       }    
    }
      
      
   }
}

/******************************************
ָʾ������
******************************************/
void vTaskIndicatorLED(void *pvParameters)
{
   while(1)
   {
       if(isGetGyroBiasFinished==false)
          ledFlash(200,200);
   }
}

