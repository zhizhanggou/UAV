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
      if(isSensorAllReady()) 
      {
        
        if(xQueueReceive(accDataQueue, data.accDataProcessed, 0)==pdTRUE && xQueueReceive(gyroDataQueue, data.gyroDataProcessed, 0)==pdTRUE)
        {
          NonlinearSO3AHRSupdate(data, 1, 0.005, deltaT/1000000.0f);
        }
				if(xQueueReceive(atltitudeDataQueue, &data.baroAltitude, 0)==pdTRUE)
				{
					
				}
				if(xQueueReceive(magDataQueue, &data.magDataProcessed, 0)==pdTRUE)
				{
					
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
        if(runingCount%(MAG_READ_LOOP_TIME/MAIN_LOOP_TIME)==0 && MAG_SENSOR_ENABLE) //��һ������ִ�д����ƶ�ȡ
						READ_MPU9250_MAG();
        MS5611_ThreadNew();
      // deltaT=__HAL_TIM_GetCounter(&TIM6_Handler);//��ȡ��ʱ��6��ʱ��
				if(isGetGyroBiasFinished==false)
						getGyroBias(gyroOriginalData);
        else if(isSensorAllReady())
        {
						imuOriginalDataProcessing(accOriginalData,gyroOriginalData,magOriginalData);
						vTaskSuspendAll();	/*ȷ��ͬһʱ�̰����ݷ��������*/
						xQueueOverwrite(accDataQueue,dataProcessed.accDataProcessed);
						xQueueOverwrite(gyroDataQueue,dataProcessed.gyroDataProcessed);
						if(isAltitudeDataReady)
						{
								xQueueOverwrite(atltitudeDataQueue,&dataProcessed.baroAltitude);
								isAltitudeDataReady=false;
						}
						if(MAG_SENSOR_ENABLE&&isMagDataReady==true)
						{
								xQueueOverwrite(magDataQueue,&dataProcessed.magDataProcessed);
								isMagDataReady=false;
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
USER userData;
void vTaskDataUpload(void *pvParameters)
{
		EventBits_t uxBits;
		HMI_data mpu9250_data;

		while(1)
		{
			uxBits = xEventGroupWaitBits(xUploadEventGroup, /* �¼���־���� */
                                 ifDataReadyUpLoad, /* �ȴ� bit0 ������ */
                                 pdTRUE, /* �˳�ǰ bit0 �� bit1 ����� */
                                 pdTRUE, /* ����Ϊ pdTRUE ��ʾ�ȴ� bit1 �� bit0 ��������*/
                                 portMAX_DELAY); /* �ȴ��ӳ�ʱ�� */
			if((uxBits & ifDataReadyUpLoad)==ifDataReadyUpLoad)
			{
        if(dataType==STATUS_DATA)
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
        else if(dataType==USER_DATA)
        {
            userData.DATA1=MS5611_Altitude;
            userData.DATA2=0.0f;
            userData.DATA3=0.0f;
            userData.DATA4=0.0f;
            Send_USERDATA(userData);
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
       if(isSensorAllReady()==false)
          ledFlash(200,200);
   }
}

