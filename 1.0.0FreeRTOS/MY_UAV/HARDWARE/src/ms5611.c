#include "ms5611.h"
#include "math.h"
#include "delay.h"
#include "stm32f4xx_it.h"



#define MS5611Press_OSR  MS561101BA_OSR_4096  //��ѹ��������
#define MS5611Temp_OSR   MS561101BA_OSR_4096  //�¶Ȳ�������
// ��ѹ��״̬��
#define SCTemperature  0x01	  //��ʼ �¶�ת��
#define CTemperatureing  0x02  //����ת���¶�
#define SCPressure  0x03	  //��ʼת�� ��ѹ
#define SCPressureing  0x04	  //����ת����ѹֵ

#define MOVAVG_SIZE  10	   //�������10������  5
#define Alt_offset_num 200 //��ѹ��ƫ�ü����ۼ���
int Alt_offset_count=0,Altitude_Offset_count=0; //��ѹ�Ƽ���ƫ�ü���
long long int Alt_offset_temp=0; //��ʱ����ۼ�ֵ
static uint8_t  Now_doing = SCTemperature;	//��ǰת��״̬
static uint16_t PROM_C[MS561101BA_PROM_REG_COUNT]; //�궨ֵ���
static uint32_t Current_delay=0;	    //ת����ʱʱ�� us 
static uint32_t Start_Convert_Time; //����ת��ʱ�� ʱ�� us 
static int32_t  tempCache;

static float Alt_Offset_m = 0;

//
#define PA_OFFSET_INIT_NUM 50	

static float Alt_offset_Pa=0; //�����0��(���������ƽ��)ʱ ��Ӧ����ѹֵ  ���ֵ����ϵ�ʱ����ѹֵ 
double paOffsetNum = 0; 
uint16_t  paInitCnt=0;

uint8_t paOffsetInited=0;

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
  GPIO_InitTypeDef  GPIO_InitStructure;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);//ʹ��GPIODʱ��

  //GPIOF9,F10��ʼ������
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_0;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//��ͨ���ģʽ
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
  GPIO_Init(GPIOD, &GPIO_InitStructure);//��ʼ��
	
	MS5611_IIC_SCL=1;
	MS5611_IIC_SDA=1;
	MS561101BA_reset();
	delay_ms(50);
	MS561101BA_readPROM();
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
	tempCache = MS561101BA_getConversion();	
}


/**************************ʵ�ֺ���********************************************
*����ԭ��:		float MS561101BA_get_altitude(void)
*��������:	    ����ǰ����ѹֵת�� �߶ȡ�	 
*******************************************************************************/
float MS561101BA_get_altitude(void)
{
	static float Altitude;

	// �Ƿ��ʼ����0����ѹֵ��
	if(Alt_offset_Pa == 0)
	{ 
		if(paInitCnt > PA_OFFSET_INIT_NUM)
		{
			Alt_offset_Pa = paOffsetNum / paInitCnt;
			paOffsetInited=1;
		}
		else
			paOffsetNum += MS5611_Pressure;
		
		paInitCnt++;
		
		Altitude = 0; //�߶� Ϊ 0
		
		return Altitude;
	}
	//����������ϵ�ʱ��λ�õĸ߶�ֵ ����λΪm
	Altitude = 4433000.0 * (1 - pow((MS5611_Pressure / Alt_offset_Pa), 0.190295f))*0.01f;
	Altitude = Altitude + Alt_Offset_m ;  //��ƫ��


	
	return Altitude; 
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
	int32_t rawPress = MS561101BA_getConversion();
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
	
	if(Alt_offset_count<Alt_offset_num && MS5611_Pressure>0 )   //������ѹ��ƫ��
	{
		Alt_offset_count++;
		if(Alt_offset_count>=100)
		{
			Alt_offset_temp+=MS5611_Pressure;
			Alt_offset_Pa=Alt_offset_temp/((Alt_offset_count-100)*1.0);
		}
	}
	else if(MS5611_Pressure>0)
	{   

   MS5611_Altitude = MS561101BA_get_altitude(); // ��λ��cm 
		
	}
	
	
}


/**************************ʵ�ֺ���********************************************
*����ԭ��:		void MS561101BA_reset(void)
*��������:	    ���͸�λ��� MS561101B 
*******************************************************************************/
void MS561101BA_reset(void) 
{
	MS5611_IIC_Start();
  MS5611_IIC_Send_Byte(MS5611_ADDR); //д��ַ
	MS5611_IIC_Wait_Ack();
  MS5611_IIC_Send_Byte(MS561101BA_RESET);//���͸�λ����
	MS5611_IIC_Wait_Ack();	
  MS5611_IIC_Stop();
}
/**************************ʵ�ֺ���********************************************
*����ԭ��:		void MS561101BA_readPROM(void)
*��������:	    ��ȡ MS561101B �Ĺ����궨ֵ
��ȡ ��ѹ�Ƶı궨ֵ  ���������¶Ⱥ���ѹ�Ķ���
*******************************************************************************/
void MS561101BA_readPROM(void) 
{
	u8  inth,intl;
	int i;
	for (i=0;i<MS561101BA_PROM_REG_COUNT;i++) 
	{
			MS5611_IIC_Start();
			MS5611_IIC_Send_Byte(MS5611_ADDR);
			MS5611_IIC_Wait_Ack();
			MS5611_IIC_Send_Byte(MS561101BA_PROM_BASE_ADDR + (i * MS561101BA_PROM_REG_SIZE));   //MS561101BA_PROM_REG_SIZE �����ֽ� 
			MS5611_IIC_Wait_Ack();	
			MS5611_IIC_Stop();
			delay_us(5);
			MS5611_IIC_Start();
			MS5611_IIC_Send_Byte(MS5611_ADDR+1);  //�������ģʽ	
			delay_us(1);
			MS5611_IIC_Wait_Ack();
			inth = MS5611_IIC_Read_Byte(1);  //��ACK�Ķ�����
			delay_us(1);
			intl = MS5611_IIC_Read_Byte(0);	 //���һ���ֽ�NACK
			MS5611_IIC_Stop();
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
void MS561101BA_startConversion(uint8_t command) 
{
	// initialize pressure conversion
	MS5611_IIC_Start();
	MS5611_IIC_Send_Byte(MS5611_ADDR); //д��ַ
	MS5611_IIC_Wait_Ack();
	MS5611_IIC_Send_Byte(command); //дת������
	MS5611_IIC_Wait_Ack();	
	MS5611_IIC_Stop();

}
#define CMD_ADC_READ            0x00 // ADC read command
/**************************ʵ�ֺ���********************************************
*����ԭ��:		unsigned long MS561101BA_getConversion(void)
*��������:	    ��ȡ MS561101B ��ת�����	 
*******************************************************************************/
uint32_t MS561101BA_getConversion(void) 
{
	uint32_t conversion = 0;
	u8 temp[3];
	MS5611_IIC_Start();
	MS5611_IIC_Send_Byte(MS5611_ADDR); //д��ַ
	MS5611_IIC_Wait_Ack();
	MS5611_IIC_Send_Byte(0);// start read sequence
	MS5611_IIC_Wait_Ack();	
	MS5611_IIC_Stop();
	
	MS5611_IIC_Start();
	MS5611_IIC_Send_Byte(MS5611_ADDR+1);  //�������ģʽ	
	MS5611_IIC_Wait_Ack();
	temp[0] = MS5611_IIC_Read_Byte(1);  //��ACK�Ķ�����  bit 23-16
	temp[1] = MS5611_IIC_Read_Byte(1);  //��ACK�Ķ�����  bit 8-15
	temp[2] = MS5611_IIC_Read_Byte(0);  //��NACK�Ķ����� bit 0-7
	MS5611_IIC_Stop();
	conversion = (unsigned long)temp[0] * 65536 + (unsigned long)temp[1] * 256 + (unsigned long)temp[2];

	return conversion;
}

void MS5611_ThreadNew(void) 
{
	switch(Now_doing)
	{ //��ѯ״̬ ������������ ����Щʲô��
 		case SCTemperature:  //�����¶�ת��
			//�����¶�ת��
				MS561101BA_startConversion(MS561101BA_D2 + MS5611Temp_OSR);
				Current_delay = MS5611_Delay_us[MS5611Temp_OSR] ;//ת��ʱ��
				Start_Convert_Time = micros(); //��ʱ��ʼ
				Now_doing = CTemperatureing;//��һ��״̬
 		break;
		
		case CTemperatureing:  //����ת���� 
			if((micros()-Start_Convert_Time) > Current_delay)
			{ //��ʱʱ�䵽����
				MS561101BA_GetTemperature(); //ȡ�¶�	
				//������ѹת��
				MS561101BA_startConversion(MS561101BA_D1 + MS5611Press_OSR);
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
				MS561101BA_startConversion(MS561101BA_D2 + MS5611Temp_OSR);
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
uint8_t  WaitBaroInitOffset(void)
{
	uint32_t startTime=0;
	uint32_t now=0;
	
	startTime=micros();	//us
  while(!paOffsetInited)
	{
			MS5611_ThreadNew();
			now=micros();
			if((now-startTime)/1000 >= PA_OFFSET_INIT_NUM * 50)	//��ʱ
			{
				return 0;
			}
	}
	
	return 1;
}


/**********************IICЭ��**********************/
void MS5611_I2C_delay(void)
{
   u8 i=100; 
   while(i) 
   { 
     i--; 
   } 
	
}
int MS5611_IIC_Start(void)
{
	MS5611_SDA_OUT();     //sda�����
	MS5611_IIC_SDA=1;	  	  
	MS5611_IIC_SCL=1;
	MS5611_I2C_delay();
 	MS5611_IIC_SDA=0;//START:when CLK is high,DATA change form high to low 
	MS5611_I2C_delay();
	MS5611_IIC_SCL=0;//ǯסI2C���ߣ�׼�����ͻ�������� 
	return 1;
}	  
//����IICֹͣ�ź�
int MS5611_IIC_Stop(void)
{
	MS5611_SDA_OUT();//sda�����
	MS5611_IIC_SCL=0;
	MS5611_IIC_SDA=0;//STOP:when CLK is high DATA change form low to high
 	MS5611_I2C_delay();
	MS5611_IIC_SCL=1; 
	MS5611_IIC_SDA=1;//����I2C���߽����ź�
	MS5611_I2C_delay();		
	return 1;	
}
//�ȴ�Ӧ���źŵ���
//����ֵ��1������Ӧ��ʧ��
//        0������Ӧ��ɹ�
u8 MS5611_IIC_Wait_Ack(void)
{
	u8 ucErrTime=0;
	MS5611_SDA_IN();      //SDA����Ϊ����  
	MS5611_IIC_SDA=1;MS5611_I2C_delay();	   
	MS5611_IIC_SCL=1;MS5611_I2C_delay();	 
	while(MS5611_READ_SDA)
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			MS5611_IIC_Stop();
			return 1;
		}
	}
	MS5611_IIC_SCL=0;//ʱ�����0 	   
	return 0;  
} 
//����ACKӦ��
void MS5611_IIC_Ack(void)
{
	MS5611_IIC_SCL=0;
	MS5611_SDA_OUT();
	MS5611_IIC_SDA=0;
	MS5611_I2C_delay();
	MS5611_IIC_SCL=1;
	MS5611_I2C_delay();
	MS5611_IIC_SCL=0;
}
//������ACKӦ��		    
void MS5611_IIC_NAck(void)
{
	MS5611_IIC_SCL=0;
	MS5611_SDA_OUT();
	MS5611_IIC_SDA=1;
	MS5611_I2C_delay();
	MS5611_IIC_SCL=1;
	MS5611_I2C_delay();
	MS5611_IIC_SCL=0;
}					 				     
//IIC����һ���ֽ�
//���شӻ�����Ӧ��
//1����Ӧ��
//0����Ӧ��			  
void MS5611_IIC_Send_Byte(u8 txd)
{                        
    u8 t;   
	MS5611_SDA_OUT(); 	    
    MS5611_IIC_SCL=0;//����ʱ�ӿ�ʼ���ݴ���
    for(t=0;t<8;t++)
    {              
        MS5611_IIC_SDA=(txd&0x80)>>7;
        txd<<=1; 	  
		MS5611_I2C_delay();  //��TEA5767��������ʱ���Ǳ����
		MS5611_IIC_SCL=1;
		MS5611_I2C_delay(); 
		MS5611_IIC_SCL=0;	
		MS5611_I2C_delay();
    }	 
} 	    
//��1���ֽڣ�ack=1ʱ������ACK��ack=0������nACK   
u8 MS5611_IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	MS5611_SDA_IN();//SDA����Ϊ����
    for(i=0;i<8;i++ )
	{
        MS5611_IIC_SCL=0; 
        MS5611_I2C_delay();
		MS5611_IIC_SCL=1;
        receive<<=1;
        if(MS5611_READ_SDA)receive++;   
		MS5611_I2C_delay(); 
    }					 
    if (!ack)
        MS5611_IIC_NAck();//����nACK
    else
       MS5611_IIC_Ack(); //����ACK   
    return receive;
}


u8 MS5611_IIC_Read(u8 Slaveaddress,u8 REG_Address)
{
	u8 REG_data;
	MS5611_IIC_Start(); 
	MS5611_IIC_Send_Byte(Slaveaddress); 
	REG_data=MS5611_IIC_Wait_Ack();	
	MS5611_IIC_Send_Byte(REG_Address);
	REG_data=MS5611_IIC_Wait_Ack();	
	MS5611_IIC_Start(); 
	MS5611_IIC_Send_Byte(Slaveaddress|1);
	REG_data=MS5611_IIC_Wait_Ack();	
	REG_data=MS5611_IIC_Read_Byte(0);
	MS5611_IIC_Stop();	
	return REG_data;
}
void MS5611_IIC_Write(u8 Slaveaddress,u8 REG_Address,u8 REG_data)
{
	MS5611_IIC_Start(); 
	MS5611_IIC_Send_Byte(Slaveaddress); 
	MS5611_IIC_Wait_Ack();	
	MS5611_IIC_Send_Byte(REG_Address); 
	MS5611_IIC_Wait_Ack(); 
	MS5611_IIC_Send_Byte(REG_data);
	MS5611_IIC_Wait_Ack(); 
	MS5611_IIC_Stop();
}

void I2C_NoAddr_WriteByte(unsigned char DeviceAddr,unsigned char info)  
{  
  
   MS5611_IIC_Start();  
   MS5611_IIC_Send_Byte(DeviceAddr);  
   MS5611_IIC_Wait_Ack();  
   MS5611_IIC_Send_Byte(info);  
   MS5611_IIC_Wait_Ack();  
   MS5611_IIC_Stop();  
   //delay2(50);  
   MS5611_I2C_delay();  
  
}  
uint16_t I2C_Read_2Bytes(unsigned char DeviceAddr,unsigned char address)  
{  
   unsigned char data_temp1,data_temp2;  
     uint16_t data16;  
   MS5611_IIC_Start();  
   MS5611_IIC_Send_Byte(DeviceAddr);  
   MS5611_IIC_Wait_Ack();  
   MS5611_IIC_Send_Byte(address);  
   MS5611_IIC_Wait_Ack();  
   MS5611_IIC_Start();  
   MS5611_IIC_Send_Byte(DeviceAddr+1);  
   MS5611_IIC_Wait_Ack();    
   data_temp1=MS5611_IIC_Read_Byte(0);  
   MS5611_IIC_Ack();  
   data_temp2=MS5611_IIC_Read_Byte(0);    
    MS5611_IIC_NAck();//���һ���ֽ�����Ӧ��  
   MS5611_IIC_Stop();  
   //delay2(10);  
   MS5611_I2C_delay();  
   data16=(data_temp1<<8)|data_temp2;  
    return data16;
}  
uint32_t I2C_Read_3Bytes(unsigned char DeviceAddr,unsigned char address)  
{  
   unsigned char data_temp1,data_temp2,data_temp3;  
     uint32_t data32;  
   MS5611_IIC_Start();  
   MS5611_IIC_Send_Byte(DeviceAddr);  
   MS5611_IIC_Wait_Ack();  
   MS5611_IIC_Send_Byte(address);  
   MS5611_IIC_Wait_Ack();  
   MS5611_IIC_Start();  
   MS5611_IIC_Send_Byte(DeviceAddr+1);  
   MS5611_IIC_Wait_Ack();  
       
   data_temp1=MS5611_IIC_Read_Byte(0);  
   MS5611_IIC_Ack();  
   data_temp2=MS5611_IIC_Read_Byte(0);    
   MS5611_IIC_Ack();  
   data_temp3=MS5611_IIC_Read_Byte(0);  
   MS5611_IIC_NAck();//���һ���ֽ�����Ӧ��  
   MS5611_IIC_Stop();  
   //delay2(10);  
   MS5611_I2C_delay();  
   data32=data_temp1*65535+data_temp2*256+data_temp3;  
   return data32;
}


