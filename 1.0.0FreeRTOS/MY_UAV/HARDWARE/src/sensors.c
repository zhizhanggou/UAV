#include "sensors.h"


static xQueueHandle gyroDataQueue;
static xQueueHandle accDataQueue;
static xQueueHandle magDataQueue;
static xQueueHandle baroDataQueue;
GyroBias gyroBias;
Axis3f imuBias; //9250��ƫ
Axis3i16 gyroBiasSampleBuffer[GYRO_BIAS_SAMPLES_NUM];
Axis3i16 *bufHead=gyroBiasSampleBuffer;
bool isBufferFulled=false,isGetGyroBiasFinished=false;
void sensorsQueueInit()
{
   gyroDataQueue = xQueueCreate(1,sizeof(Axis3f));
   //baroDataQueue = xQueueCreate(1,sizeof(motionProcessing));
   
}
bool getGyroBias(Axis3i16 data) 
{
   bufHead->x=data.x;
   bufHead->y=data.y;
   bufHead->z=data.z;
	
   bufHead++;
	if(bufHead>=&gyroBiasSampleBuffer[GYRO_BIAS_SAMPLES_NUM])
	{
		isBufferFulled=true;
		bufHead=gyroBiasSampleBuffer;
	}
   
	if(isBufferFulled)
	{
		getGyroBiasMeanAndVar(gyroBiasSampleBuffer);
		//��鷽���Ƿ񳬱꣬����������������ƫ���ݣ����²���
		if(gyroBias.variance.x>GYRO_BIAS_SAMPLES_VARIANCE_GATA||gyroBias.variance.x<-GYRO_BIAS_SAMPLES_VARIANCE_GATA)
		{
			isBufferFulled=false;
			bufHead=gyroBiasSampleBuffer;
		}
		else if(gyroBias.variance.y>GYRO_BIAS_SAMPLES_VARIANCE_GATA||gyroBias.variance.y<-GYRO_BIAS_SAMPLES_VARIANCE_GATA)
		{
			isBufferFulled=false;
			bufHead=gyroBiasSampleBuffer;
		}
		else if(gyroBias.variance.z>GYRO_BIAS_SAMPLES_VARIANCE_GATA||gyroBias.variance.z<-GYRO_BIAS_SAMPLES_VARIANCE_GATA)
		{
			isBufferFulled=false;
			bufHead=gyroBiasSampleBuffer;
		}
      else
         return isGetGyroBiasFinished=true;
	}
   return isGetGyroBiasFinished=false;
}

void getGyroBiasMeanAndVar(Axis3i16 *buffer)   //����ƽ��ֵ�ͷ���
{
	u32 i;
	int64_t sum[3] = {0};
	int64_t sumsq[3] = {0};
	
	for(i=0;i<GYRO_BIAS_SAMPLES_NUM;i++)
	{
		sum[0]+=buffer[i].x;
		sum[1]+=buffer[i].y;
		sum[2]+=buffer[i].z;
		sumsq[0] += buffer[i].x * buffer[i].x;
		sumsq[1] += buffer[i].y * buffer[i].y;
		sumsq[2] += buffer[i].z * buffer[i].z;
	}
	
	gyroBias.variance.x=(sumsq[0] - ((int64_t)sum[0] * sum[0])/GYRO_BIAS_SAMPLES_NUM)/GYRO_BIAS_SAMPLES_NUM;
	gyroBias.variance.y=(sumsq[1] - ((int64_t)sum[1] * sum[1])/GYRO_BIAS_SAMPLES_NUM)/GYRO_BIAS_SAMPLES_NUM;
	gyroBias.variance.z=(sumsq[2] - ((int64_t)sum[2] * sum[2])/GYRO_BIAS_SAMPLES_NUM)/GYRO_BIAS_SAMPLES_NUM;
	
	gyroBias.value.x=(float)sum[0]/GYRO_BIAS_SAMPLES_NUM;
	gyroBias.value.y=(float)sum[1]/GYRO_BIAS_SAMPLES_NUM;
	gyroBias.value.z=(float)sum[2]/GYRO_BIAS_SAMPLES_NUM;
   
}


Axis3f gyroDataProcessed,accDataProcessed,magDataProcessed;

void imuOriginalDataProcessing(Axis3i16 accData,Axis3i16 gyroData,Axis3i16 magData) //�Ӽƣ������ǡ������Ƶ�ԭʼ���ݴ���  
{
   gyroDataProcessed.x=(float)(gyroData.x-gyroBias.value.x)*0.0174f*GYRO_GAIN;
   gyroDataProcessed.y=(float)(gyroData.x-gyroBias.value.y)*0.0174f*GYRO_GAIN;
   gyroDataProcessed.z=(float)(gyroData.x-gyroBias.value.z)*0.0174f*GYRO_GAIN;
}

