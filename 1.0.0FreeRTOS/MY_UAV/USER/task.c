#include "task.h"
#include "flightStatus.h"

EventGroupHandle_t xUploadEventGroup;
EventGroupHandle_t xSensorEventGroup;
/******************************************
��̬��������
******************************************/

uint16_t deltaT;//���δ��������ݶ�ȡ��ʱ���

void vTaskAttitudeAlgorithm(void *pvParameters)
{
  TickType_t lastWakeTime = xTaskGetTickCount();
  
	DataProcessed data;
   while(1)
   { 
      vTaskDelayUntil(&lastWakeTime,MAIN_LOOP_TIME);
      if(isGetGyroBiasFinished) 
      {
        
        if(xQueueReceive(accDataQueue, data.accDataProcessed, 0)==pdTRUE)
        {
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
int16_t runingCount=0;
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
       
        runingCount++;
        if(runingCount>100)   //�����ȡ����ļ���
        {
          runingCount=0;
        }
        
        READ_MPU9250_ACCEL();
        READ_MPU9250_GYRO();
        if(runingCount%(MAG_READ_LOOP_TIME/MAIN_LOOP_TIME)==0 && MAG_SENSOR_ENABLE)
          READ_MPU9250_MAG();
        MS5611_ThreadNew();
      // deltaT=__HAL_TIM_GetCounter(&TIM6_Handler);//��ȡ��ʱ��6��ʱ��

        if(isGetGyroBiasFinished==false)
         getGyroBias(gyroOriginalData);
        else if(isGetGyroBiasFinished)
        {
          imuOriginalDataProcessing(accOriginalData,gyroOriginalData,magOriginalData);
          vTaskSuspendAll();	/*ȷ��ͬһʱ�̰����ݷ��������*/
          xQueueOverwrite(accDataQueue,dataProcessed.accDataProcessed);
          xQueueOverwrite(gyroDataQueue,dataProcessed.gyroDataProcessed);
          if(MAG_SENSOR_ENABLE)
          {
          //  xQueueOverwrite(magDataQueue,&dataProcessed.magDataProcessed);
          }
          xTaskResumeAll();
          
        }
        xEventGroupSetBits(xUploadEventGroup,ifDataReadyUpLoad2);  //�����¼���־���BIT0
      }
   
   }
}











/******************************************
�����ϴ���λ������
******************************************/

void vTaskDataUpload(void *pvParameters)
{
		EventBits_t uxBits;
		HMI_data mpu9250_data;
		USER data;
		while(1)
		{
			uxBits = xEventGroupWaitBits(xUploadEventGroup, /* �¼���־���� */
                                 ifDataReadyUpLoad, /* �ȴ� bit0 ������ */
                                 pdTRUE, /* �˳�ǰ bit0 �� bit1 ����� */
                                 pdTRUE, /* ����Ϊ pdTRUE ��ʾ�ȴ� bit1 �� bit0 ��������*/
                                 portMAX_DELAY); /* �ȴ��ӳ�ʱ�� */
			if((uxBits & ifDataReadyUpLoad)==ifDataReadyUpLoad)
			{
        if(DataToSend==STATUS_DATA)
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
         
            Send_RCData(mpu9250_data,flightStatus.attitude.x,flightStatus.attitude.y,flightStatus.attitude.z,MS5611_Altitude,0,0);
        }
        else if(DataToSend==USER_DATA)
        {
            data.DATA1=MS5611_Altitude;
            data.DATA2=0.0f;
            data.DATA3=0.0f;
            data.DATA4=0.0f;
            data.DATA5=0.0f;
            data.DATA6=0.0f;
            data.DATA7=0.0f;
            data.DATA8=0.0f;
            data.DATA9=0.0f;
            Send_USERDATA(data);
        }

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

