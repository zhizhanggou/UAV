#include "task.h"
EventGroupHandle_t xCreatedEventGroup;

/******************************************
��̬��������
******************************************/
void vTaskAttitudeSolution(void *pvParameters)
{
   EventBits_t uxBits;
   while(1)
   {
      uxBits = xEventGroupWaitBits(xCreatedEventGroup, /* �¼���־���� */
                                 ifSensorsReadFinish, /* �ȴ� bit0 ������ */
                                 pdTRUE, /* �˳�ǰ bit0 �� bit1 ����� */
                                 pdTRUE, /* ����Ϊ pdTRUE ��ʾ�ȴ� bit1 �� bit0 ��������*/
                                 portMAX_DELAY); /* �ȴ��ӳ�ʱ�� */
      if((uxBits & ifSensorsReadFinish)==ifSensorsReadFinish)
      {
         
      }
      //xEventGroupSetBits(xCreatedEventGroup,BIT_0);  //�����¼���־���BIT0
//      vTaskDelay(20);
   }
   
}




/******************************************
��������ȡ����
******************************************/
void vTaskReadSenser(void *pvParameters)
{
   while(1)
   {
      READ_MPU9250_ACCEL();
      READ_MPU9250_GYRO();
      //READ_MPU9250_MAG();
      if(isGetGyroBiasFinished==false)
         getGyroBias(gyroOriginalData);
      else if(isGetGyroBiasFinished)
      {
         imuOriginalDataProcessing(accOriginalData,gyroOriginalData,magOriginalData);
         
         vTaskSuspendAll();	/*ȷ��ͬһʱ�̰����ݷ��������*/
         xQueueOverwrite(accDataQueue,dataProcessed.accDataProcessed);
         xQueueOverwrite(gyroDataQueue,dataProcessed.gyroDataProcessed);
         xTaskResumeAll();
         
      }
      xEventGroupSetBits(xCreatedEventGroup,ifSensorsReadFinish);  //�����¼���־���BIT0
      
      
      
   //   xQueueOverwrite(imuDataQueue,&mpu9250_OriginalData);
      
      
      vTaskDelay(1);
      
   }
}











/******************************************
�����ϴ���λ������
******************************************/
HMI_data mpu9250_data;
void vTaskDataUpload(void *pvParameters)
{

   while(1)
   {
//      uxBits = xEventGroupWaitBits(xCreatedEventGroup, /* �¼���־���� */
//                                 ifSensorsReadFinish, /* �ȴ� bit0 ������ */
//                                 pdTRUE, /* �˳�ǰ bit0 �� bit1 ����� */
//                                 pdTRUE, /* ����Ϊ pdTRUE ��ʾ�ȴ� bit1 �� bit0 ��������*/
//                                 portMAX_DELAY); /* �ȴ��ӳ�ʱ�� */

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

