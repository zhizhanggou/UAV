#include "sensors.h"
#include "math.h"
xSemaphoreHandle sensorsDataReady;
xQueueHandle gyroDataQueue;
xQueueHandle accDataQueue;
xQueueHandle magDataQueue;
xQueueHandle atltitudeDataQueue;
GyroBias gyroBias;
Axis3f accBias,accT; //�Ӽ���ƫ������ϵ��
Axis3i16 gyroBiasSampleBuffer[GYRO_BIAS_SAMPLES_NUM];
Axis3i16 *bufHead=gyroBiasSampleBuffer;
bool isBufferFulled=false,isGetGyroBiasFinished=false;
void sensorsInit()
{
	flightStatus.attitude.x=0;
	flightStatus.attitude.y=0;
	flightStatus.attitude.z=0;
	flightStatus.height=0;
   accBias.x=-0.127453;
   accBias.y=0.158471;
   accBias.z=-0.461189;
   accT.x=1.003888;
   accT.y=1.000822;
   accT.z=0.994147;
	sensorsDataReady = xSemaphoreCreateBinary();/*�������������ݾ�����ֵ�ź���*/
   
}
void sensorsQueueInit()
{
    gyroDataQueue = xQueueCreate(1,sizeof(Axis3f));
    accDataQueue = xQueueCreate(1,sizeof(Axis3f));
    magDataQueue = xQueueCreate(1,sizeof(Axis3f));
    atltitudeDataQueue = xQueueCreate(1,sizeof(float));
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


DataProcessed dataProcessed;

void imuOriginalDataProcessing(Axis3i16 accData,Axis3i16 gyroData,Axis3i16 magData) //�Ӽƣ������ǡ������Ƶ�ԭʼ���ݴ���  
{
   dataProcessed.gyroDataProcessed[0]=(float)((float)gyroData.x-gyroBias.value.x)*0.0174f*GYRO_GAIN;
   dataProcessed.gyroDataProcessed[1]=(float)((float)gyroData.y-gyroBias.value.y)*0.0174f*GYRO_GAIN;
   dataProcessed.gyroDataProcessed[2]=(float)((float)gyroData.z-gyroBias.value.z)*0.0174f*GYRO_GAIN;
   
//   dataProcessed.accDataProcessed[0]=(accData.x*ACC_GAIN-accBias.x)*accT.x;
//   dataProcessed.accDataProcessed[1]=(accData.y*ACC_GAIN-accBias.y)*accT.y;
//   dataProcessed.accDataProcessed[2]=(accData.z*ACC_GAIN-accBias.z)*accT.z;
	
	 dataProcessed.accDataProcessed[0]=accData.x*ACC_GAIN;
   dataProcessed.accDataProcessed[1]=accData.y*ACC_GAIN;
   dataProcessed.accDataProcessed[2]=accData.z*ACC_GAIN;
}







bool isSensorAllReady()
{
	if(isGetGyroBiasFinished && isGetAtitudeOffsetFinished)
		return true;
	else 
		return false;
}
