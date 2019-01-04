#include "ms5611.h"
#include "stm32f4xx_it.h"
#include "math.h"



#define MS5611Press_OSR  MS561101BA_OSR_4096  //��ѹ��������
#define MS5611Temp_OSR   MS561101BA_OSR_4096  //�¶Ȳ�������
// ��ѹ��״̬��
#define SCTemperature  0x01	  //��ʼ �¶�ת��
#define CTemperatureing  0x02  //����ת���¶�
#define SCPressure  0x03	  //��ʼת�� ��ѹ
#define SCPressureing  0x04	  //����ת����ѹֵ

#define MOVAVG_SIZE  10	   //�������10������  5
#define Alt_offset_num 200 //��ѹ��ƫ�ü����ۼ���

iic ms5611_iic;
int Alt_offset_count=0,Altitude_Offset_count=0; //��ѹ�Ƽ���ƫ�ü���
long long int Alt_offset_temp=0; //��ʱ����ۼ�ֵ
static uint8_t  Now_doing = SCTemperature;	//��ǰת��״̬
static uint16_t PROM_C[MS561101BA_PROM_REG_COUNT]; //�궨ֵ���
static uint32_t Current_delay=0;	    //ת����ʱʱ�� us 
static uint32_t Start_Convert_Time; //����ת��ʱ�� ʱ�� us 
static int32_t  tempCache;


bool isGetAtitudeOffsetFinished=false;
bool isAltitudeDataReady=false;
static float Alt_Offset_m = 0;

//
#define ALT_OFFSET_INIT_NUM 100	

static float Alt_offset_Pa=0; //�����0��(���������ƽ��)ʱ ��Ӧ����ѹֵ  ���ֵ����ϵ�ʱ����ѹֵ 
uint16_t  AltGetOffsetCnt=0;

//uint8_t paOffsetInited=0;

//interface for outside 
uint8_t Baro_ALT_Updated = 0; //��ѹ�Ƹ߶ȸ�����ɱ�־��
//units (Celsius degrees*100, mbar*100  ).
//��λ [�¶� ��] [��ѹ ��]  [�߶� ��] 
volatile float MS5611_Temperature,MS5611_Pressure,MS5611_Altitude,MS5611_VerticalSpeed,MS5611_Altitude_Offset=0;   //volatile �ӵ�ַ��ȡ��������׼ȷ

// ��ʱ��λ us 	  ��ͬ�Ĳ������ȶ�Ӧ��ͬ����ʱֵ
uint32_t MS5611_Delay_us[9] = {
	1500,//MS561101BA_OSR_256 0.9ms  0x00
	1500,//MS561101BA_OSR_256 0.9ms  
	2000,//MS561101BA_OSR_512 1.2ms  0x02
	2000,//MS561101BA_OSR_512 1.2ms
	3000,//MS561101BA_OSR_1024 2.3ms 0x04
	3000,//MS561101BA_OSR_1024 2.3ms
	5000,//MS561101BA_OSR_2048 4.6ms 0x06
	5000,//MS561101BA_OSR_2048 4.6ms
	11000,//MS561101BA_OSR_4096 9.1ms 0x08
};

// FIFO ����					
static float Temp_buffer[MOVAVG_SIZE],Press_buffer[MOVAVG_SIZE],Alt_buffer[MOVAVG_SIZE];
static uint8_t temp_index=0,press_index=0; //����ָ��

void MS5611_Init(void)
{    	 
  cycleCounterInit();
  iicSingleInit(ms5611_iic=iicPortDef(GPIOD,GPIO_PIN_0,GPIOD,GPIO_PIN_1,3));
	MS561101BA_reset(ms5611_iic);
	delay_ms(50);
	MS561101BA_readPROM(ms5611_iic);
	delay_ms(50);
	
}
//���һ���µ�ֵ�� �¶ȶ��� �����˲�
void MS561101BA_NewTemp(float val) 
{
	Temp_buffer[temp_index] = val;
	temp_index = (temp_index + 1) % MOVAVG_SIZE;
}

//���һ���µ�ֵ�� ��ѹ���� �����˲�
void MS561101BA_NewPress(float val)
{
	Press_buffer[press_index] = val;
	press_index = (press_index + 1) % MOVAVG_SIZE;
}

//���һ���µ�ֵ�� �߶ȶ��� �����˲�
void MS561101BA_NewAlt(float val) 
{
	int16_t i;
	for(i=1;i<MOVAVG_SIZE;i++)
		Alt_buffer[i-1] = Alt_buffer[i];
	Alt_buffer[MOVAVG_SIZE-1] = val;
}

//��ȡ���е�ƽ��ֵ
float MS561101BA_getAvg(float * buff, int size) 
{
	float sum = 0.0;
	int i;
	for(i=0; i<size; i++) 
	{
		sum += buff[i];
	}
	return sum / size;
}


/**************************ʵ�ֺ���********************************************
*����ԭ��:		void MS561101BA_GetTemperature(void)
*��������:	    ��ȡ �¶�ת�����	 
*******************************************************************************/
void MS561101BA_GetTemperature(void)
{	
	tempCache = MS561101BA_getConversion(ms5611_iic);	
}


/**************************ʵ�ֺ���********************************************
*����ԭ��:		float MS561101BA_get_altitude(void)
*��������:	    ����ǰ����ѹֵת�� �߶ȡ�	 
*******************************************************************************/
float MS561101BA_get_altitude(void)
{
	static float Altitude;
	


	//����������ϵ�ʱ��λ�õĸ߶�ֵ ����λΪm
	Altitude = 4433000.0 * (1 - pow((MS5611_Pressure / STANDARD_ATMOSPHERIC_PRESSURE), 0.190295f))*0.01f;
	if(isGetAtitudeOffsetFinished==false)
	{
		getAltitudeOffset(Altitude);
		return Altitude;
	}
	// �Ƿ��ʼ�������λ�ø߶ȣ�

//	if(Alt_Offset_m == 0)
//	{ 
//		if(paInitCnt > PA_OFFSET_INIT_NUM)
//		{
//			temp+=Altitude;
//			paOffsetInited=1;
//		}
//		else
//			paOffsetNum += MS5611_Pressure;
//		
//		paInitCnt++;
//		
//		Altitude = 0; //�߶� Ϊ 0
//		
//		return Altitude;
//	}
	Altitude = Altitude - Alt_Offset_m ;  //��ƫ��


	
	return Altitude; 
}


void getAltitudeOffset(float attitude)
{
	static float attitudeTemp;
	if(AltGetOffsetCnt<ALT_OFFSET_INIT_NUM)	
	{
		attitudeTemp+=attitude;
		AltGetOffsetCnt++;
	}
	else if(AltGetOffsetCnt==ALT_OFFSET_INIT_NUM)
	{
		Alt_Offset_m=attitudeTemp/ALT_OFFSET_INIT_NUM;
		AltGetOffsetCnt++;
		isGetAtitudeOffsetFinished=true;
	}
}



/**************************ʵ�ֺ���********************************************
*����ԭ��:		void MS561101BA_getPressure(void)
*��������:	    ��ȡ ��ѹת����� ������������	 
*******************************************************************************/
//static float lastPress=0,newPress=0;
//static float press_limit_coe = 1;
void MS561101BA_getPressure(void) 
{
	int64_t off,sens;
	int64_t TEMP,T2,Aux_64,OFF2,SENS2;  // 64 bits
	int32_t rawPress = MS561101BA_getConversion(ms5611_iic);
	int64_t dT  = tempCache - (((int32_t)PROM_C[4]) << 8);       //dT = D2 - TREF = D2 - C5 * 2^8
	
	TEMP = 2000 + (dT * (int64_t)PROM_C[5])/8388608;       //TEMP = 20��C + dT * TEMPSENS = 2000 + dT * C6 / 2^23
	off  = (((int64_t)PROM_C[1]) << 16) + ((((int64_t)PROM_C[3]) * dT) >> 7);  //OFF = OFFT1 + TCO * dT = C2 * 2^16+ (C4 * dT ) / 2^7
	sens = (((int64_t)PROM_C[0]) << 15) + (((int64_t)(PROM_C[2]) * dT) >> 8);  //SENS = SENST1 + TCS * dT = C1 * 2 ^15+ (C3 * dT ) / 2^8
	
	if (TEMP < 2000)
	{   // second order temperature compensation
		T2 = (((int64_t)dT)*dT) >> 31;
		Aux_64 = (TEMP-2000)*(TEMP-2000);
		OFF2 = (5*Aux_64)>>1;
		SENS2 = (5*Aux_64)>>2;
		if( TEMP<-1500)
		{
			Aux_64= (TEMP+1500)*(TEMP+1500);
			OFF2=OFF2+7*Aux_64;
			SENS2=SENS2+((11*Aux_64)>>1);
		}
		TEMP = TEMP - T2;
		off = off - OFF2;
		sens = sens - SENS2;
	}

	//------------- ��ѹ���� ----------------
//	newPress=(((((int64_t)rawPress) * sens) >> 21) - off) / 32768;
//	
//	press_limit_coe = 1.0f; 
//	
//	//����ʱ������ѹֵ���ͣ���ѹ�߶����ߣ�
//	if(ALT_LOCK_FLAG == 0xff && (Math_abs(IMU_Pitch)>15 || Math_abs(IMU_Roll)>15))
//	{		
//		press_limit_coe = 0.01f;   //0.005
//		if(newPress<lastPress)
//			newPress = (1 - press_limit_coe) * lastPress + press_limit_coe * newPress; 
//	}

//	lastPress = newPress;
//	
//	MS5611_Pressure = newPress;
	
	//ԭʼ�ķ���
	
	MS5611_Pressure = (((((int64_t)rawPress) * sens) >> 21) - off) >>15; //P = D1 * SENS - OFF = (D1 * SENS / 2^21 - OFF) / 2^15

	//�¶ȶ��д���
	MS561101BA_NewTemp(TEMP*0.01f);
	
	MS5611_Temperature = MS561101BA_getAvg(Temp_buffer,MOVAVG_SIZE); //0.01c
	
  MS5611_Altitude = MS561101BA_get_altitude(); // ��λ��cm 
	dataProcessed.baroAltitude=MS5611_Altitude;
	isAltitudeDataReady=true;

}


/**************************ʵ�ֺ���********************************************
*����ԭ��:		void MS561101BA_reset(void)
*��������:	    ���͸�λ��� MS561101B 
*******************************************************************************/
void MS561101BA_reset(iic iicPort) 
{
	IIC_Start(iicPort);
  IIC_Send_Byte(iicPort,MS5611_ADDR); //д��ַ
	IIC_Wait_Ack(iicPort); 
  IIC_Send_Byte(iicPort,MS561101BA_RESET);//���͸�λ����
	IIC_Wait_Ack(iicPort); 	
  IIC_Stop(iicPort); 
}
/**************************ʵ�ֺ���********************************************
*����ԭ��:		void MS561101BA_readPROM(void)
*��������:	    ��ȡ MS561101B �Ĺ����궨ֵ
��ȡ ��ѹ�Ƶı궨ֵ  ���������¶Ⱥ���ѹ�Ķ���
*******************************************************************************/
void MS561101BA_readPROM(iic iicPort) 
{
	u8  inth,intl;
	int i;
	for (i=0;i<MS561101BA_PROM_REG_COUNT;i++) 
	{
			IIC_Start(iicPort);
			IIC_Send_Byte(iicPort,MS5611_ADDR);
			IIC_Wait_Ack(iicPort); 
			IIC_Send_Byte(iicPort,MS561101BA_PROM_BASE_ADDR + (i * MS561101BA_PROM_REG_SIZE));   //MS561101BA_PROM_REG_SIZE �����ֽ� 
			IIC_Wait_Ack(iicPort); 	
			IIC_Stop(iicPort); 
			delay_us(5);
			IIC_Start(iicPort);
			IIC_Send_Byte(iicPort,MS5611_ADDR+1);  //�������ģʽ	
			delay_us(1);
			IIC_Wait_Ack(iicPort); 
			inth = IIC_Read_Byte(iicPort,1);  //��ACK�Ķ�����
			delay_us(1);
			intl = IIC_Read_Byte(iicPort,0);	 //���һ���ֽ�NACK
			IIC_Stop(iicPort); 
			PROM_C[i] = (((uint16_t)inth << 8) | intl);

	}
}
/*                           PROM������

                                                           |               |size  |Typical| my
C1 Pressure sensitivity | SENST1                           |unsignedint 16 |16    |40127  |48856      
C2 Pressure offset | OFFT1                                 |unsignedint 16 |16    |36924  |46810          
C3 Temperature coefficient of pressure sensitivity | TCS   |unsignedint 16 |16    |23317  |30901       
C4 Temperature coefficient of pressure offset | TCO        |unsignedint 16 |16    |23282  |28353        
C5 Reference temperature | TREF                            |unsignedint 16 |16    |33464  |32634        
C6 Temperature coefficient of the temperature | TEMPSENS   |unsignedint 16 |16    |28312  |28052        
*/

/**************************ʵ�ֺ���********************************************
*����ԭ��:		void MS561101BA_startConversion(uint8_t command)
*��������:	    ��������ת����� MS561101B
��ѡ�� ת������Ϊ MS561101BA_D1  ת����ѹ
				  MS561101BA_D2  ת���¶�	 
*******************************************************************************/
void MS561101BA_startConversion(iic iicPort,uint8_t command) 
{
	// initialize pressure conversion
	IIC_Start(iicPort);
	IIC_Send_Byte(iicPort,MS5611_ADDR); //д��ַ
	IIC_Wait_Ack(iicPort); 
	IIC_Send_Byte(iicPort,command); //дת������
	IIC_Wait_Ack(iicPort); 	
	IIC_Stop(iicPort); 

}
#define CMD_ADC_READ            0x00 // ADC read command
/**************************ʵ�ֺ���********************************************
*����ԭ��:		unsigned long MS561101BA_getConversion(void)
*��������:	    ��ȡ MS561101B ��ת�����	 
*******************************************************************************/
uint32_t MS561101BA_getConversion(iic iicPort) 
{
	uint32_t conversion = 0;
	u8 temp[3];
	IIC_Start(iicPort);
	IIC_Send_Byte(iicPort,MS5611_ADDR); //д��ַ
	IIC_Wait_Ack(iicPort); 
	IIC_Send_Byte(iicPort,0);// start read sequence
	IIC_Wait_Ack(iicPort); 	
	IIC_Stop(iicPort); 
	
	IIC_Start(iicPort);
	IIC_Send_Byte(iicPort,MS5611_ADDR+1);  //�������ģʽ	
	IIC_Wait_Ack(iicPort); 
	temp[0] = IIC_Read_Byte(iicPort,1);  //��ACK�Ķ�����  bit 23-16
	temp[1] = IIC_Read_Byte(iicPort,1);  //��ACK�Ķ�����  bit 8-15
	temp[2] = IIC_Read_Byte(iicPort,0);  //��NACK�Ķ����� bit 0-7
	IIC_Stop(iicPort); 
	conversion = (unsigned long)temp[0] * 65536 + (unsigned long)temp[1] * 256 + (unsigned long)temp[2];

	return conversion;
}

void MS5611_ThreadNew(void) 
{
	switch(Now_doing)
	{ //��ѯ״̬ ������������ ����Щʲô��
 		case SCTemperature:  //�����¶�ת��
			//�����¶�ת��
				MS561101BA_startConversion(ms5611_iic,MS561101BA_D2 + MS5611Temp_OSR);
				Current_delay = MS5611_Delay_us[MS5611Temp_OSR] ;//ת��ʱ��
				Start_Convert_Time = micros(); //��ʱ��ʼ
				Now_doing = CTemperatureing;//��һ��״̬
 		break;
		
		case CTemperatureing:  //����ת���� 
			if((micros()-Start_Convert_Time) > Current_delay)
			{ //��ʱʱ�䵽����
				MS561101BA_GetTemperature(); //ȡ�¶�	
				//������ѹת��
				MS561101BA_startConversion(ms5611_iic,MS561101BA_D1 + MS5611Press_OSR);
				Current_delay = MS5611_Delay_us[MS5611Press_OSR];//ת��ʱ��
				Start_Convert_Time = micros();//��ʱ��ʼ
				Now_doing = SCPressureing;//��һ��״̬
			}
			break;
 
		case SCPressureing:	 //����ת����ѹֵ
			if((micros()-Start_Convert_Time) > Current_delay)
			{ //��ʱʱ�䵽����
				MS561101BA_getPressure();   //���� ����	
				Baro_ALT_Updated = 0xff; 	//�߶ȸ��� ��ɡ�
			//	Now_doing = SCTemperature;  //��ͷ����
				//�����¶�ת��
				MS561101BA_startConversion(ms5611_iic,MS561101BA_D2 + MS5611Temp_OSR);
				Current_delay = MS5611_Delay_us[MS5611Temp_OSR] ;//ת��ʱ��
				Start_Convert_Time = micros(); //��ʱ��ʼ
				Now_doing = CTemperatureing;//��һ��״̬
			}
			break;
		default: 
			Now_doing = CTemperatureing;
			break;
	}
}
//ע�⣬ʹ��ǰȷ��
//uint8_t  WaitBaroInitOffset(void)
//{
//	uint32_t startTime=0;
//	uint32_t now=0;
//	
//	startTime=micros();	//us
//  while(!paOffsetInited)
//	{
//			MS5611_ThreadNew();
//			now=micros();
//			if((now-startTime)/1000 >= PA_OFFSET_INIT_NUM * 50)	//��ʱ
//			{
//				return 0;
//			}
//	}
//	
//	return 1;
//}


/**********************IICЭ��**********************/
void ms5611_I2C_delay()
{
    uint16_t i=20;
    while(i)
    {
      i--;
    }
}

u8 ms5611_IIC_Read(iic iicPort,u8 Slaveaddress,u8 REG_Address)
{
  u8 REG_data;
	IIC_Start(iicPort); 
	IIC_Send_Byte(iicPort,Slaveaddress); 
	REG_data=IIC_Wait_Ack(iicPort);	
	IIC_Send_Byte(iicPort,REG_Address);
	REG_data=IIC_Wait_Ack(iicPort);	
	IIC_Start(iicPort); 
	IIC_Send_Byte(iicPort,Slaveaddress|1);
	REG_data=IIC_Wait_Ack(iicPort);	
	REG_data=IIC_Read_Byte(iicPort,0);
	IIC_Stop(iicPort);	
	return REG_data;
}
void ms5611_IIC_Write(iic iicPort,u8 Slaveaddress,u8 REG_Address,u8 REG_data)
{
	IIC_Start(iicPort); 
	IIC_Send_Byte(iicPort,Slaveaddress); 
	IIC_Wait_Ack(iicPort);	
	IIC_Send_Byte(iicPort,REG_Address); 
	IIC_Wait_Ack(iicPort); 
	IIC_Send_Byte(iicPort,REG_data);
	IIC_Wait_Ack(iicPort); 
	IIC_Stop(iicPort);
}

void IIC_NoAddr_WriteByte(iic iicPort,u8 Slaveaddress,u8 REG_data)  
{    
  
    IIC_Start(iicPort);   
    IIC_Send_Byte(iicPort,Slaveaddress); 
    IIC_Wait_Ack(iicPort); 
    IIC_Send_Byte(iicPort,REG_data);   
    IIC_Wait_Ack(iicPort);  
    IIC_Stop(iicPort); 
   //delay2(50);  
    ms5611_I2C_delay();
  
}  
uint16_t IIC_Read_2Bytes(iic iicPort,u8 Slaveaddress,u8 REG_Address)  
{  
   unsigned char data_temp1,data_temp2;  
     uint16_t data16;  
   IIC_Start(iicPort);  
   IIC_Send_Byte(iicPort,Slaveaddress); 
   IIC_Wait_Ack(iicPort); 
   IIC_Send_Byte(iicPort,REG_Address);  
   IIC_Wait_Ack(iicPort);  
   IIC_Start(iicPort);  
   IIC_Send_Byte(iicPort,Slaveaddress|1);  
   IIC_Wait_Ack(iicPort);   
   data_temp1=IIC_Read_Byte(iicPort,0);  
   IIC_Wait_Ack(iicPort);  
   data_temp2=IIC_Read_Byte(iicPort,0);;    
   IIC_NAck(iicPort);//���һ���ֽ�����Ӧ��  
   IIC_Stop(iicPort);  
   ms5611_I2C_delay();  
   data16=(data_temp1<<8)|data_temp2;  
    return data16;
}  
uint32_t IIC_Read_3Bytes(iic iicPort,u8 Slaveaddress,u8 REG_Address)  
{  
   unsigned char data_temp1,data_temp2,data_temp3;  
     uint32_t data32;  
   IIC_Start(iicPort);  
   IIC_Send_Byte(iicPort,Slaveaddress);  
   IIC_Wait_Ack(iicPort);   
   IIC_Send_Byte(iicPort,REG_Address);  
   IIC_Wait_Ack(iicPort);   
   IIC_Start(iicPort);  
   IIC_Send_Byte(iicPort,Slaveaddress+1);  
   IIC_Wait_Ack(iicPort);   
       
   data_temp1=IIC_Read_Byte(iicPort,0);  
   IIC_Wait_Ack(iicPort);  
   data_temp2=IIC_Read_Byte(iicPort,0);    
   IIC_Wait_Ack(iicPort);  
   data_temp3=IIC_Read_Byte(iicPort,0);  
   IIC_NAck(iicPort);//���һ���ֽ�����Ӧ��  
   IIC_Stop(iicPort);  
   //delay2(10);  
   ms5611_I2C_delay();  
   data32=data_temp1*65535+data_temp2*256+data_temp3;  
   return data32;
}


