#include "ms5611.h"
#include "math.h"
#include "delay.h"
#include "stm32f4xx_it.h"



#define MS5611Press_OSR  MS561101BA_OSR_4096  //气压采样精度
#define MS5611Temp_OSR   MS561101BA_OSR_4096  //温度采样精度
// 气压计状态机
#define SCTemperature  0x01	  //开始 温度转换
#define CTemperatureing  0x02  //正在转换温度
#define SCPressure  0x03	  //开始转换 气压
#define SCPressureing  0x04	  //正在转换气压值

#define MOVAVG_SIZE  10	   //保存最近10组数据  5
#define Alt_offset_num 200 //气压计偏置计算累加数
int Alt_offset_count=0,Altitude_Offset_count=0; //气压计计算偏置计数
long long int Alt_offset_temp=0; //暂时存放累加值
static uint8_t  Now_doing = SCTemperature;	//当前转换状态
static uint16_t PROM_C[MS561101BA_PROM_REG_COUNT]; //标定值存放
static uint32_t Current_delay=0;	    //转换延时时间 us 
static uint32_t Start_Convert_Time; //启动转换时的 时间 us 
static int32_t  tempCache;

static float Alt_Offset_m = 0;

//
#define PA_OFFSET_INIT_NUM 50	

static float Alt_offset_Pa=0; //存放着0米(离起飞所在平面)时 对应的气压值  这个值存放上电时的气压值 
double paOffsetNum = 0; 
uint16_t  paInitCnt=0;

uint8_t paOffsetInited=0;

//interface for outside 
uint8_t Baro_ALT_Updated = 0; //气压计高度更新完成标志。
//units (Celsius degrees*100, mbar*100  ).
//单位 [温度 度] [气压 帕]  [高度 米] 
volatile float MS5611_Temperature,MS5611_Pressure,MS5611_Altitude,MS5611_VerticalSpeed,MS5611_Altitude_Offset=0;   //volatile 从地址读取，慢但是准确

// 延时表单位 us 	  不同的采样精度对应不同的延时值
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

// FIFO 队列					
static float Temp_buffer[MOVAVG_SIZE],Press_buffer[MOVAVG_SIZE],Alt_buffer[MOVAVG_SIZE];
static uint8_t temp_index=0,press_index=0; //队列指针

void MS5611_Init(void)
{    	 
  GPIO_InitTypeDef  GPIO_InitStructure;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);//使能GPIOD时钟

  //GPIOF9,F10初始化设置
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_0;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
  GPIO_Init(GPIOD, &GPIO_InitStructure);//初始化
	
	MS5611_IIC_SCL=1;
	MS5611_IIC_SDA=1;
	MS561101BA_reset();
	delay_ms(50);
	MS561101BA_readPROM();
	delay_ms(50);
	
}
//添加一个新的值到 温度队列 进行滤波
void MS561101BA_NewTemp(float val) 
{
	Temp_buffer[temp_index] = val;
	temp_index = (temp_index + 1) % MOVAVG_SIZE;
}

//添加一个新的值到 气压队列 进行滤波
void MS561101BA_NewPress(float val)
{
	Press_buffer[press_index] = val;
	press_index = (press_index + 1) % MOVAVG_SIZE;
}

//添加一个新的值到 高度队列 进行滤波
void MS561101BA_NewAlt(float val) 
{
	int16_t i;
	for(i=1;i<MOVAVG_SIZE;i++)
		Alt_buffer[i-1] = Alt_buffer[i];
	Alt_buffer[MOVAVG_SIZE-1] = val;
}

//读取队列的平均值
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


/**************************实现函数********************************************
*函数原型:		void MS561101BA_GetTemperature(void)
*功　　能:	    读取 温度转换结果	 
*******************************************************************************/
void MS561101BA_GetTemperature(void)
{	
	tempCache = MS561101BA_getConversion();	
}


/**************************实现函数********************************************
*函数原型:		float MS561101BA_get_altitude(void)
*功　　能:	    将当前的气压值转成 高度。	 
*******************************************************************************/
float MS561101BA_get_altitude(void)
{
	static float Altitude;

	// 是否初始化过0米气压值？
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
		
		Altitude = 0; //高度 为 0
		
		return Altitude;
	}
	//计算相对于上电时的位置的高度值 。单位为m
	Altitude = 4433000.0 * (1 - pow((MS5611_Pressure / Alt_offset_Pa), 0.190295f))*0.01f;
	Altitude = Altitude + Alt_Offset_m ;  //加偏置


	
	return Altitude; 
}

/**************************实现函数********************************************
*函数原型:		void MS561101BA_getPressure(void)
*功　　能:	    读取 气压转换结果 并做补偿修正	 
*******************************************************************************/
//static float lastPress=0,newPress=0;
//static float press_limit_coe = 1;
void MS561101BA_getPressure(void) 
{
	int64_t off,sens;
	int64_t TEMP,T2,Aux_64,OFF2,SENS2;  // 64 bits
	int32_t rawPress = MS561101BA_getConversion();
	int64_t dT  = tempCache - (((int32_t)PROM_C[4]) << 8);       //dT = D2 - TREF = D2 - C5 * 2^8
	
	TEMP = 2000 + (dT * (int64_t)PROM_C[5])/8388608;       //TEMP = 20°C + dT * TEMPSENS = 2000 + dT * C6 / 2^23
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

	//------------- 气压修正 ----------------
//	newPress=(((((int64_t)rawPress) * sens) >> 21) - off) / 32768;
//	
//	press_limit_coe = 1.0f; 
//	
//	//机动时限制气压值降低（气压高度增高）
//	if(ALT_LOCK_FLAG == 0xff && (Math_abs(IMU_Pitch)>15 || Math_abs(IMU_Roll)>15))
//	{		
//		press_limit_coe = 0.01f;   //0.005
//		if(newPress<lastPress)
//			newPress = (1 - press_limit_coe) * lastPress + press_limit_coe * newPress; 
//	}

//	lastPress = newPress;
//	
//	MS5611_Pressure = newPress;
	
	//原始的方法
	
	MS5611_Pressure = (((((int64_t)rawPress) * sens) >> 21) - off) >>15; //P = D1 * SENS - OFF = (D1 * SENS / 2^21 - OFF) / 2^15

	//温度队列处理
	MS561101BA_NewTemp(TEMP*0.01f);
	
	MS5611_Temperature = MS561101BA_getAvg(Temp_buffer,MOVAVG_SIZE); //0.01c
	
	if(Alt_offset_count<Alt_offset_num && MS5611_Pressure>0 )   //计算气压计偏置
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

   MS5611_Altitude = MS561101BA_get_altitude(); // 单位：cm 
		
	}
	
	
}


/**************************实现函数********************************************
*函数原型:		void MS561101BA_reset(void)
*功　　能:	    发送复位命令到 MS561101B 
*******************************************************************************/
void MS561101BA_reset(void) 
{
	MS5611_IIC_Start();
  MS5611_IIC_Send_Byte(MS5611_ADDR); //写地址
	MS5611_IIC_Wait_Ack();
  MS5611_IIC_Send_Byte(MS561101BA_RESET);//发送复位命令
	MS5611_IIC_Wait_Ack();	
  MS5611_IIC_Stop();
}
/**************************实现函数********************************************
*函数原型:		void MS561101BA_readPROM(void)
*功　　能:	    读取 MS561101B 的工厂标定值
读取 气压计的标定值  用于修正温度和气压的读数
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
			MS5611_IIC_Send_Byte(MS561101BA_PROM_BASE_ADDR + (i * MS561101BA_PROM_REG_SIZE));   //MS561101BA_PROM_REG_SIZE 俩个字节 
			MS5611_IIC_Wait_Ack();	
			MS5611_IIC_Stop();
			delay_us(5);
			MS5611_IIC_Start();
			MS5611_IIC_Send_Byte(MS5611_ADDR+1);  //进入接收模式	
			delay_us(1);
			MS5611_IIC_Wait_Ack();
			inth = MS5611_IIC_Read_Byte(1);  //带ACK的读数据
			delay_us(1);
			intl = MS5611_IIC_Read_Byte(0);	 //最后一个字节NACK
			MS5611_IIC_Stop();
			PROM_C[i] = (((uint16_t)inth << 8) | intl);

	}
}
/*                           PROM参数表

                                                           |               |size  |Typical| my
C1 Pressure sensitivity | SENST1                           |unsignedint 16 |16    |40127  |48856      
C2 Pressure offset | OFFT1                                 |unsignedint 16 |16    |36924  |46810          
C3 Temperature coefficient of pressure sensitivity | TCS   |unsignedint 16 |16    |23317  |30901       
C4 Temperature coefficient of pressure offset | TCO        |unsignedint 16 |16    |23282  |28353        
C5 Reference temperature | TREF                            |unsignedint 16 |16    |33464  |32634        
C6 Temperature coefficient of the temperature | TEMPSENS   |unsignedint 16 |16    |28312  |28052        
*/

/**************************实现函数********************************************
*函数原型:		void MS561101BA_startConversion(uint8_t command)
*功　　能:	    发送启动转换命令到 MS561101B
可选的 转换命令为 MS561101BA_D1  转换气压
				  MS561101BA_D2  转换温度	 
*******************************************************************************/
void MS561101BA_startConversion(uint8_t command) 
{
	// initialize pressure conversion
	MS5611_IIC_Start();
	MS5611_IIC_Send_Byte(MS5611_ADDR); //写地址
	MS5611_IIC_Wait_Ack();
	MS5611_IIC_Send_Byte(command); //写转换命令
	MS5611_IIC_Wait_Ack();	
	MS5611_IIC_Stop();

}
#define CMD_ADC_READ            0x00 // ADC read command
/**************************实现函数********************************************
*函数原型:		unsigned long MS561101BA_getConversion(void)
*功　　能:	    读取 MS561101B 的转换结果	 
*******************************************************************************/
uint32_t MS561101BA_getConversion(void) 
{
	uint32_t conversion = 0;
	u8 temp[3];
	MS5611_IIC_Start();
	MS5611_IIC_Send_Byte(MS5611_ADDR); //写地址
	MS5611_IIC_Wait_Ack();
	MS5611_IIC_Send_Byte(0);// start read sequence
	MS5611_IIC_Wait_Ack();	
	MS5611_IIC_Stop();
	
	MS5611_IIC_Start();
	MS5611_IIC_Send_Byte(MS5611_ADDR+1);  //进入接收模式	
	MS5611_IIC_Wait_Ack();
	temp[0] = MS5611_IIC_Read_Byte(1);  //带ACK的读数据  bit 23-16
	temp[1] = MS5611_IIC_Read_Byte(1);  //带ACK的读数据  bit 8-15
	temp[2] = MS5611_IIC_Read_Byte(0);  //带NACK的读数据 bit 0-7
	MS5611_IIC_Stop();
	conversion = (unsigned long)temp[0] * 65536 + (unsigned long)temp[1] * 256 + (unsigned long)temp[2];

	return conversion;
}

void MS5611_ThreadNew(void) 
{
	switch(Now_doing)
	{ //查询状态 看看我们现在 该做些什么？
 		case SCTemperature:  //启动温度转换
			//开启温度转换
				MS561101BA_startConversion(MS561101BA_D2 + MS5611Temp_OSR);
				Current_delay = MS5611_Delay_us[MS5611Temp_OSR] ;//转换时间
				Start_Convert_Time = micros(); //计时开始
				Now_doing = CTemperatureing;//下一个状态
 		break;
		
		case CTemperatureing:  //正在转换中 
			if((micros()-Start_Convert_Time) > Current_delay)
			{ //延时时间到了吗？
				MS561101BA_GetTemperature(); //取温度	
				//启动气压转换
				MS561101BA_startConversion(MS561101BA_D1 + MS5611Press_OSR);
				Current_delay = MS5611_Delay_us[MS5611Press_OSR];//转换时间
				Start_Convert_Time = micros();//计时开始
				Now_doing = SCPressureing;//下一个状态
			}
			break;
 
		case SCPressureing:	 //正在转换气压值
			if((micros()-Start_Convert_Time) > Current_delay)
			{ //延时时间到了吗？
				MS561101BA_getPressure();   //更新 计算	
				Baro_ALT_Updated = 0xff; 	//高度更新 完成。
			//	Now_doing = SCTemperature;  //从头再来
				//开启温度转换
				MS561101BA_startConversion(MS561101BA_D2 + MS5611Temp_OSR);
				Current_delay = MS5611_Delay_us[MS5611Temp_OSR] ;//转换时间
				Start_Convert_Time = micros(); //计时开始
				Now_doing = CTemperatureing;//下一个状态
			}
			break;
		default: 
			Now_doing = CTemperatureing;
			break;
	}
}
//注意，使用前确保
uint8_t  WaitBaroInitOffset(void)
{
	uint32_t startTime=0;
	uint32_t now=0;
	
	startTime=micros();	//us
  while(!paOffsetInited)
	{
			MS5611_ThreadNew();
			now=micros();
			if((now-startTime)/1000 >= PA_OFFSET_INIT_NUM * 50)	//超时
			{
				return 0;
			}
	}
	
	return 1;
}


/**********************IIC协议**********************/
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
	MS5611_SDA_OUT();     //sda线输出
	MS5611_IIC_SDA=1;	  	  
	MS5611_IIC_SCL=1;
	MS5611_I2C_delay();
 	MS5611_IIC_SDA=0;//START:when CLK is high,DATA change form high to low 
	MS5611_I2C_delay();
	MS5611_IIC_SCL=0;//钳住I2C总线，准备发送或接收数据 
	return 1;
}	  
//产生IIC停止信号
int MS5611_IIC_Stop(void)
{
	MS5611_SDA_OUT();//sda线输出
	MS5611_IIC_SCL=0;
	MS5611_IIC_SDA=0;//STOP:when CLK is high DATA change form low to high
 	MS5611_I2C_delay();
	MS5611_IIC_SCL=1; 
	MS5611_IIC_SDA=1;//发送I2C总线结束信号
	MS5611_I2C_delay();		
	return 1;	
}
//等待应答信号到来
//返回值：1，接收应答失败
//        0，接收应答成功
u8 MS5611_IIC_Wait_Ack(void)
{
	u8 ucErrTime=0;
	MS5611_SDA_IN();      //SDA设置为输入  
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
	MS5611_IIC_SCL=0;//时钟输出0 	   
	return 0;  
} 
//产生ACK应答
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
//不产生ACK应答		    
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
//IIC发送一个字节
//返回从机有无应答
//1，有应答
//0，无应答			  
void MS5611_IIC_Send_Byte(u8 txd)
{                        
    u8 t;   
	MS5611_SDA_OUT(); 	    
    MS5611_IIC_SCL=0;//拉低时钟开始数据传输
    for(t=0;t<8;t++)
    {              
        MS5611_IIC_SDA=(txd&0x80)>>7;
        txd<<=1; 	  
		MS5611_I2C_delay();  //对TEA5767这三个延时都是必须的
		MS5611_IIC_SCL=1;
		MS5611_I2C_delay(); 
		MS5611_IIC_SCL=0;	
		MS5611_I2C_delay();
    }	 
} 	    
//读1个字节，ack=1时，发送ACK，ack=0，发送nACK   
u8 MS5611_IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	MS5611_SDA_IN();//SDA设置为输入
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
        MS5611_IIC_NAck();//发送nACK
    else
       MS5611_IIC_Ack(); //发送ACK   
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
    MS5611_IIC_NAck();//最后一个字节无需应答  
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
   MS5611_IIC_NAck();//最后一个字节无需应答  
   MS5611_IIC_Stop();  
   //delay2(10);  
   MS5611_I2C_delay();  
   data32=data_temp1*65535+data_temp2*256+data_temp3;  
   return data32;
}


