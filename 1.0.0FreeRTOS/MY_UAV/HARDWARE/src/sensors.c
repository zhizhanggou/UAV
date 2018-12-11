#include "sensors.h"


static xQueueHandle imuDataQueue;
static xQueueHandle baroDataQueue;
GyroBias gyroBias;
Axis3f imuBias; //9250零偏
Axis3i16 gyroBiasSampleBuffer[GYRO_BIAS_SAMPLES_NUM];
Axis3i16 *bufHead=gyroBiasSampleBuffer;
bool isBufferFulled=false;
void sensorsQueueInit()
{
   imuDataQueue = xQueueCreate(1,sizeof(motionProcessing));
   //baroDataQueue = xQueueCreate(1,sizeof(motionProcessing));
   
}
Axis3f getGyroBias(motionProcessing data)
{
   bufHead->x=data.GYRO_X;
   bufHead->y=data.GYRO_Y;
   bufHead->z=data.GYRO_Z;
	
   bufHead++;
	if(bufHead>=&gyroBiasSampleBuffer[GYRO_BIAS_SAMPLES_NUM])
	{
		isBufferFulled=true;
		bufHead=gyroBiasSampleBuffer;
	}
   
	if(isBufferFulled)
	{
		getGyroBiasMeanAndVar(gyroBiasSampleBuffer);
		//检查方差是否超标，如果超标放弃这组零偏数据，重新测量
		if(gyroBias.variance.x>GYRO_BIAS_SAMPLES_VARIANCE_GATA||gyroBias.variance.x<GYRO_BIAS_SAMPLES_VARIANCE_GATA)
		{
			isBufferFulled=false;
			bufHead=gyroBiasSampleBuffer;
		}
		else if(gyroBias.variance.y>GYRO_BIAS_SAMPLES_VARIANCE_GATA||gyroBias.variance.y<GYRO_BIAS_SAMPLES_VARIANCE_GATA)
		{
			isBufferFulled=false;
			bufHead=gyroBiasSampleBuffer;
		}
		else if(gyroBias.variance.z>GYRO_BIAS_SAMPLES_VARIANCE_GATA||gyroBias.variance.z<GYRO_BIAS_SAMPLES_VARIANCE_GATA)
		{
			isBufferFulled=false;
			bufHead=gyroBiasSampleBuffer;
		}
		
	}
   
}

void getGyroBiasMeanAndVar(Axis3i16 *buffer)   //计算平均值和方差
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
	
	gyroBias.variance.x=(sumsq[0] - ((int64_t)sum[0] * sum[0]/GYRO_BIAS_SAMPLES_NUM));
	gyroBias.variance.y=(sumsq[1] - ((int64_t)sum[1] * sum[1]/GYRO_BIAS_SAMPLES_NUM));
	gyroBias.variance.z=(sumsq[2] - ((int64_t)sum[2] * sum[2]/GYRO_BIAS_SAMPLES_NUM));
	
	gyroBias.value.x=(float)sum[0]/GYRO_BIAS_SAMPLES_NUM;
	gyroBias.value.y=(float)sum[1]/GYRO_BIAS_SAMPLES_NUM;
	gyroBias.value.z=(float)sum[2]/GYRO_BIAS_SAMPLES_NUM;
	
}
