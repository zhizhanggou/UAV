#include "includes.h"
#if SYSTEM_SUPPORT_OS
#include "SWJ.h"					//ucos 使用	  
#endif
u8 TxBuffer[256];
#define BYTE0(dwTemp)       (*(char *)(&dwTemp))
#define BYTE1(dwTemp)       (*((char *)(&dwTemp) + 1))
#define BYTE2(dwTemp)       ( *( (char *)(&dwTemp) + 2) )
#define BYTE3(dwTemp)       ( *( (char *)(&dwTemp) + 3) )


u8 DataToSend=STATUS_DATA;
u8 count=0;
//void Uart1_Put_Buf(unsigned char *DataToSend , u8 data_num)
//{
//	for(i=0;i<data_num;i++)
//		TxBuffer[count++] = *(DataToSend+i);
//	if(!(USART1->CR1 & USART_CR1_TXEIE))
//		HAL_UART_Transmit_IT(&UART1_Handler, USART_IT_TXE, ENABLE); 
//}
void Send_RCData(HMI_data data,float angle_rol, float angle_pit, float angle_yaw, s32 alt, u8 fly_model, u8 armed)
{
	u8 data_to_send[100];
	vs16 _temp;
	vs32 _temp2 = alt;
	u8 _cnt=0;
	u8 sum = 0;
/***************************传感器数据****************************/
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x02;
	data_to_send[_cnt++]=0;
	
	data_to_send[_cnt++]=BYTE1(data.ACC_X);
	data_to_send[_cnt++]=BYTE0(data.ACC_X);
	
	data_to_send[_cnt++]=BYTE1(data.ACC_Y);
	data_to_send[_cnt++]=BYTE0(data.ACC_Y);
	
	data_to_send[_cnt++]=BYTE1(data.ACC_Z);
	data_to_send[_cnt++]=BYTE0(data.ACC_Z);
	
	data_to_send[_cnt++]=BYTE1(data.GYRO_X);
	data_to_send[_cnt++]=BYTE0(data.GYRO_X);
	
	data_to_send[_cnt++]=BYTE1(data.GYRO_Y);
	data_to_send[_cnt++]=BYTE0(data.GYRO_Y);
	
	data_to_send[_cnt++]=BYTE1(data.GYRO_Z);
	data_to_send[_cnt++]=BYTE0(data.GYRO_Z);
	
	data_to_send[_cnt++]=BYTE1(data.MAG_X);
	data_to_send[_cnt++]=BYTE0(data.MAG_X);
	
	data_to_send[_cnt++]=BYTE1(data.MAG_Y);
	data_to_send[_cnt++]=BYTE0(data.MAG_Y);
	
	data_to_send[_cnt++]=BYTE1(data.MAG_Z);
	data_to_send[_cnt++]=BYTE0(data.MAG_Z);


	data_to_send[3] = _cnt-4;
	
	
	for(int i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;
	
	
	/********************姿态数据************************/
	unsigned int state_count1=0;
	state_count1=_cnt;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x01;
	data_to_send[_cnt++]=0;
	
	_temp = (int)(angle_rol*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int)(angle_pit*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int)(angle_yaw*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	data_to_send[_cnt++]=BYTE3(alt);
	data_to_send[_cnt++]=BYTE2(alt);
	data_to_send[_cnt++]=BYTE1(alt);
	data_to_send[_cnt++]=BYTE0(alt);
	
	data_to_send[_cnt++] = fly_model;
	
	data_to_send[_cnt++] = armed;
	
	data_to_send[state_count1+3] = _cnt-state_count1-4;
	
	sum = 0;
	for(u8 i=state_count1;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++]=sum;
	
	
//	/*******************遥控器数据***********************/
//	
//	state_count1=_cnt;
//	
//	data_to_send[_cnt++]=0xAA;
//	data_to_send[_cnt++]=0xAA;
//	data_to_send[_cnt++]=0x03;
//	data_to_send[_cnt++]=0;
//	data_to_send[_cnt++]=BYTE1(RC.CH1);
//	data_to_send[_cnt++]=BYTE0(RC.CH1);
//	data_to_send[_cnt++]=BYTE1(RC.CH2);
//	data_to_send[_cnt++]=BYTE0(RC.CH2);
//	data_to_send[_cnt++]=BYTE1(RC.CH3);
//	data_to_send[_cnt++]=BYTE0(RC.CH3);
//	data_to_send[_cnt++]=BYTE1(RC.CH4);
//	data_to_send[_cnt++]=BYTE0(RC.CH4);
//	data_to_send[_cnt++]=BYTE1(RC.CH5);
//	data_to_send[_cnt++]=BYTE0(RC.CH5);
//	data_to_send[_cnt++]=BYTE1(RC.CH6);
//	data_to_send[_cnt++]=BYTE0(RC.CH6);
//	data_to_send[_cnt++]=BYTE1(RC.CH7);
//	data_to_send[_cnt++]=BYTE0(RC.CH7);
//	data_to_send[_cnt++]=BYTE1(RC.CH8);
//	data_to_send[_cnt++]=BYTE0(RC.CH8);
//	data_to_send[_cnt++]=BYTE1(RC.CH9);
//	data_to_send[_cnt++]=BYTE0(RC.CH9);
////	data_to_send[_cnt++]=BYTE1(aux6);
////	data_to_send[_cnt++]=BYTE0(aux6);
//	data_to_send[_cnt++]=0;
//	data_to_send[_cnt++]=0;
//	data_to_send[state_count1+3] = _cnt-state_count1-4;
//	
//	sum = 0;
//	for(u8 i=state_count1;i<_cnt;i++)
//		sum += data_to_send[i];
//	
//	data_to_send[_cnt++]=sum;

	
   UART1DMA_USART_Transmit(&UART1_Handler,(uint8_t *)data_to_send,_cnt);
//      HAL_UART_Transmit(&UART1_Handler,(uint8_t*)data_to_send,_cnt,1000);	//发送接收到的数据
//      while(__HAL_UART_GET_FLAG(&UART1_Handler,UART_FLAG_TC)!=SET);		//等待发送结束

	
}


/************************自定义数据***************************/

USER User_Data;
void Send_USERDATA(USER data)
{
	u8 data_to_send[80];
	vs16 _temp;
	u8 _cnt=0;
	u8 sum = 0;
/***************************传感器数据****************************/
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xF1;
	data_to_send[_cnt++]=0;
   
	data_to_send[_cnt++]=BYTE3(data.DATA1);
	data_to_send[_cnt++]=BYTE2(data.DATA1);
	data_to_send[_cnt++]=BYTE1(data.DATA1);
	data_to_send[_cnt++]=BYTE0(data.DATA1);

	data_to_send[_cnt++]=BYTE3(data.DATA2);
	data_to_send[_cnt++]=BYTE2(data.DATA2);
	data_to_send[_cnt++]=BYTE1(data.DATA2);
	data_to_send[_cnt++]=BYTE0(data.DATA2);

//	data_to_send[_cnt++]=BYTE3(data.DATA3);
//	data_to_send[_cnt++]=BYTE2(data.DATA3);
//	data_to_send[_cnt++]=BYTE1(data.DATA3);
//	data_to_send[_cnt++]=BYTE0(data.DATA3);

//	data_to_send[_cnt++]=BYTE3(data.DATA4);
//	data_to_send[_cnt++]=BYTE2(data.DATA4);
//	data_to_send[_cnt++]=BYTE1(data.DATA4);
//	data_to_send[_cnt++]=BYTE0(data.DATA4);

//	data_to_send[_cnt++]=BYTE3(data.DATA5);
//	data_to_send[_cnt++]=BYTE2(data.DATA5);
//	data_to_send[_cnt++]=BYTE1(data.DATA5);
//	data_to_send[_cnt++]=BYTE0(data.DATA5);

//	data_to_send[_cnt++]=BYTE3(data.DATA6);
//	data_to_send[_cnt++]=BYTE2(data.DATA6);
//	data_to_send[_cnt++]=BYTE1(data.DATA6);
//	data_to_send[_cnt++]=BYTE0(data.DATA6);

//	data_to_send[_cnt++]=BYTE3(data.DATA7);
//	data_to_send[_cnt++]=BYTE2(data.DATA7);
//	data_to_send[_cnt++]=BYTE1(data.DATA7);
//	data_to_send[_cnt++]=BYTE0(data.DATA7);


//	data_to_send[_cnt++]=BYTE3(data.DATA8);
//	data_to_send[_cnt++]=BYTE2(data.DATA8);
//	data_to_send[_cnt++]=BYTE1(data.DATA8);
//	data_to_send[_cnt++]=BYTE0(data.DATA8);


//	data_to_send[_cnt++]=BYTE3(data.DATA9);
//	data_to_send[_cnt++]=BYTE2(data.DATA9);
//	data_to_send[_cnt++]=BYTE1(data.DATA9);
//	data_to_send[_cnt++]=BYTE0(data.DATA9);
	data_to_send[3] = _cnt-4;
	for(int i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;
	UART1DMA_USART_Transmit(&UART1_Handler,(uint8_t *)data_to_send,_cnt);
}
///*******************************************/


//u8  ReceiveBuff[RECEIVE_BUF_SIZE];   //接收缓冲  
//void USART1_IRQHandler(void)
//{
//	#if SYSTEM_SUPPORT_OS  //使用UCOS操作系统
//		OSIntEnter();    
//	#endif
//	
//		u16 data;  
//    if(USART_GetITStatus(USART1,USART_IT_IDLE) != RESET)  
//    {  
//        DMA_Cmd(DMA2_Stream5, DISABLE); //关闭DMA,防止处理其间有数据  
//  
//        data = USART1->SR;  
//        data = USART1->DR;  
//          
//        UART1_ReceiveSize =RECEIVE_BUF_SIZE - DMA_GetCurrDataCounter(DMA2_Stream5);  
////        if(UART1_ReceiveSize !=0)  
////        {  
////            OSSemPost(DMAReceiveSize_Sem);  
////        }  
//        DMA_ClearFlag(DMA2_Stream5,DMA_FLAG_TCIF5 | DMA_FLAG_FEIF5 | DMA_FLAG_DMEIF5 | DMA_FLAG_TEIF5 | DMA_FLAG_HTIF5);//清除DMA2_Steam7传输完成标志  
//        DMA_SetCurrDataCounter(DMA2_Stream5, RECEIVE_BUF_SIZE);  
//        DMA_Cmd(DMA2_Stream5, ENABLE);     //打开DMA,  
//  
//    }  
//			
//	#if SYSTEM_SUPPORT_OS  
//		OSIntExit();    	//退出中断
//	#endif			
//}

