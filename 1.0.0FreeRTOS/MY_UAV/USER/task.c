#include "task.h"
EventGroupHandle_t xCreatedEventGroup;

void vTaskAttitudeAlgorithm(void *pvParameters)
{
   while(1)
   {
      READ_MPU9250_MAG();
      xEventGroupSetBits(xCreatedEventGroup,BIT_0);  //�����¼���־���BIT0
      vTaskDelay(20);
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
      
   }
}










/*******************�����ϴ���λ������******************/
HMI_data mpu9250_data;
void vTaskDataUpload(void *pvParameters)
{
   EventBits_t uxBits;
   while(1)
   {
      uxBits = xEventGroupWaitBits(xCreatedEventGroup, /* �¼���־���� */
                                 BIT_0, /* �ȴ� bit0 ������ */
                                 pdTRUE, /* �˳�ǰ bit0 �� bit1 ����� */
                                 pdTRUE, /* ����Ϊ pdTRUE ��ʾ�ȴ� bit1 �� bit0 ��������*/
                                 portMAX_DELAY); /* �ȴ��ӳ�ʱ�� */
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
            if(__HAL_DMA_GET_FLAG(&UART1TxDMA_Handler,DMA_FLAG_TCIF3_7))  //�ж��Ƿ������
            {
               __HAL_DMA_CLEAR_FLAG(&UART1TxDMA_Handler,DMA_FLAG_TCIF3_7);//���DMA2_Steam7������ɱ�־
               HAL_UART_DMAStop(&UART1_Handler);      //��������Ժ�رմ���DMA
               break;
            }    
         }
      }
      
   }
}