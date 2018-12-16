#include "flightStatus.h"
#include "math.h"
//#include "sys.h"



uint8_t  bFilterInit;
//加速度值一阶低通滤波系数（高度解算使用）
float ACCEL_LOWPASS_Kp=0.2; 
FlightStatus flightStatus;
static float gyro_bias[3] = {0.0f, 0.0f, 0.0f}; /** bias estimation */
//重力方向加速度分量,上一次的值,去除偏置暂存数组，偏置,计数值
float Vertically_ACCEL=0,Vertically_ACCEL_Last,Vertically_ACCEL_Bias,Vertically_ACCEL_Count;
//竖直方向速度,上一次的值,加速度漂移,高度
float Vertically_Velocity,Vertically_Velocity_Last,Vertically_ACCEL_Dirft=0,Vertically_Altitude; 
u8 Vertically_Adjustment_Flag=0;
 
static float dq0 = 0.0f, dq1 = 0.0f, dq2 = 0.0f, dq3 = 0.0f;	/** quaternion of sensor frame relative to auxiliary frame */
float halfT=0.0025f;
float q0 = 1, q1 = 0, q2 = 0, q3 = 0;   
float exInt = 0, eyInt = 0, ezInt = 0;
float q0q0; 
float q0q1 ;
float q0q2 ;
float q0q3 ;
float q1q1 ;
float q1q2 ;
float q1q3;
float q2q2;
float q2q3;
float q3q3;
float Rot_matrix[9] = {1.f,  0.0f,  0.0f, 0.0f,  1.f,  0.0f, 0.0f,  0.0f,  1.f };		/**< init: identity matrix */



void NonlinearSO3AHRSinit(DataProcessed data)
{
    float initialRoll, initialPitch;
    float cosRoll, sinRoll, cosPitch, sinPitch;
    float magX, magY;
    float initialHdg, cosHeading, sinHeading;

    initialRoll = atan2(data.accDataProcessed[1], data.accDataProcessed[2]);     //此处对准完成
    initialPitch = atan2(-data.accDataProcessed[0], data.accDataProcessed[2]);

    cosRoll = cosf(initialRoll);
    sinRoll = sinf(initialRoll);
    cosPitch = cosf(initialPitch);
    sinPitch = sinf(initialPitch);

    magX = data.magDataProcessed[0] * cosPitch + data.magDataProcessed[1] * sinRoll * sinPitch + data.magDataProcessed[2] * cosRoll * sinPitch;

    magY = data.magDataProcessed[1] * cosRoll - data.magDataProcessed[2] * sinRoll;

    initialHdg = atan2f(-magY, magX);    //此处对准完成

    cosRoll = cosf(initialRoll * 0.5f);
    sinRoll = sinf(initialRoll * 0.5f);

    cosPitch = cosf(initialPitch * 0.5f);
    sinPitch = sinf(initialPitch * 0.5f);

    cosHeading = cosf(initialHdg * 0.5f);
    sinHeading = sinf(initialHdg * 0.5f);

    q0 = cosRoll * cosPitch * cosHeading + sinRoll * sinPitch * sinHeading;
    q1 = sinRoll * cosPitch * cosHeading - cosRoll * sinPitch * sinHeading;
    q2 = cosRoll * sinPitch * cosHeading + sinRoll * cosPitch * sinHeading;
    q3 = cosRoll * cosPitch * sinHeading - sinRoll * sinPitch * cosHeading;

    // auxillary variables to reduce number of repeated operations, for 1st pass
    q0q0 = q0 * q0;
    q0q1 = q0 * q1;
    q0q2 = q0 * q2;
    q0q3 = q0 * q3;
    q1q1 = q1 * q1;
    q1q2 = q1 * q2;
    q1q3 = q1 * q3;
    q2q2 = q2 * q2;
    q2q3 = q2 * q3;
    q3q3 = q3 * q3;
}
float accel_temp[3];
float gyro_temp[3];

void NonlinearSO3AHRSupdate(DataProcessed data, float twoKp, float twoKi, float dt)
{
	float euler[3] = {0.0f, 0.0f, 0.0f};	//rad
	float recipNorm;
	float halfex = 0.0f, halfey = 0.0f, halfez = 0.0f;
	
	

	// Make filter converge to initial solution faster
	// This function assumes you are in static position.
	// WARNING : in case air reboot, this can cause problem. But this is very unlikely happen.
	if(bFilterInit == 0) {
		NonlinearSO3AHRSinit(data);
		bFilterInit = 1;
	}
   
   

   
   
	//! If magnetometer measurement is available, use it.
//	if((data.magDataProcessed[0] != 0.0f) && (data.magDataProcessed[1] != 0.0f) && (data.magDataProcessed[2] != 0.0f)) {
//		float hx, hy, hz, bx, bz;
//		float halfwx, halfwy, halfwz;
//		// Normalise magnetometer measurement
//		// Will sqrt work better? PX4 system is powerful enough?
//    	recipNorm = 1.0/sqrt(data.magDataProcessed[0] * data.magDataProcessed[0] + data.magDataProcessed[1] * data.magDataProcessed[1] + data.magDataProcessed[2] * data.magDataProcessed[2]);
//    	data.magDataProcessed[0] *= recipNorm;
//    	data.magDataProcessed[1] *= recipNorm;
//    	data.magDataProcessed[2] *= recipNorm;
//    
//    	// Reference direction of Earth's magnetic field
//    	hx = 2.0f * (data.magDataProcessed[0] * (q0q0 + q1q1 - q2q2 - q3q3) + data.magDataProcessed[1] * (q1q2 - q0q3) + data.magDataProcessed[2] * (q1q3 + q0q2));
//    	hy = 2.0f * (data.magDataProcessed[0] * (q1q2 + q0q3) + data.magDataProcessed[1] * (q0q0 + q1q1 - q1q1 - q3q3) + data.magDataProcessed[2] * (q2q3 - q0q1));
//			hz = 2.0f * data.magDataProcessed[0] * (q1q3 - q0q2) + 2.0f * data.magDataProcessed[1] * (q2q3 + q0q1) + 2.0f * data.magDataProcessed[2] * (q0q0 + q1q1 - q1q1 - q2q2);
//			
////			mag_err=-atan2f(hy,hx);

////    	halfex += (q1q3-q0q2)*mag_err;
////    	halfey += (q2q3+q0q1)*mag_err;
////    	halfez += 0.5*(q0q0-q1q1-q2q2+q3q3)*mag_err;
//			
//    	bx = sqrt(hx * hx + hy * hy);
//    	bz = hz;
//    
//    	// Estimated direction of magnetic field
//    	halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
//    	halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
//    	halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);
//    
//    	// Error is sum of cross product between estimated direction and measured direction of field vectors
//    	halfex += (data.magDataProcessed[1] * halfwz - data.magDataProcessed[2] * halfwy);
//    	halfey += (data.magDataProcessed[2] * halfwx - data.magDataProcessed[0] * halfwz);
//    	halfez += (data.magDataProcessed[0] * halfwy - data.magDataProcessed[1] * halfwx);
//	}

	//增加一个条件：  加速度的模量与G相差不远时。 0.75*G < normAcc < 1.25*G
	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((data.accDataProcessed[0] == 0.0f) && (data.accDataProcessed[1] == 0.0f) && (data.accDataProcessed[2] == 0.0f))) 	
	{
		float halfvx, halfvy, halfvz;
	
		// Normalise accelerometer measurement
		recipNorm =1.0/sqrt(data.accDataProcessed[0] * data.accDataProcessed[0] + data.accDataProcessed[1] * data.accDataProcessed[1] + data.accDataProcessed[2] * data.accDataProcessed[2]);
		//--added!!!
//		accNorm=1.0f/recipNorm;
//		if(accNorm > 0.75 * CONSTANTS_ONE_G && accNorm < 1.25 * CONSTANTS_ONE_G )  //加速度过大时
//		{
			data.accDataProcessed[0] *= recipNorm;
			data.accDataProcessed[1] *= recipNorm;
			data.accDataProcessed[2] *= recipNorm;

			// Estimated direction of gravity and magnetic field
			halfvx = q1q3 - q0q2;     //计算载体坐标系上各轴的重力分量
			halfvy = q0q1 + q2q3;
			halfvz = q0q0 - 0.5f + q3q3;   //vz = q0q0 - q1q1 - q2q2 + q3q3 ;???
		
		if(Vertically_ACCEL_Count<50)   //计算Z轴加速度偏置
		{
			Vertically_ACCEL_Last=Vertically_ACCEL;
			Vertically_ACCEL=accel_temp[0]*2*halfvx+accel_temp[1]*2*halfvy+accel_temp[2]*2*halfvz-9.8f;//竖直方向总加速度减去重力加速度得到纯运动加速度
			Vertically_ACCEL_Bias+=Vertically_ACCEL;
			Vertically_ACCEL_Count++;
		}
		else
		{
			Vertically_Adjustment_Flag=1;
			Vertically_ACCEL_Last=Vertically_ACCEL-Vertically_ACCEL_Dirft;		
			//计算竖直方向加速度，使用一阶低通滤波
			Vertically_ACCEL=(accel_temp[0]*2*halfvx+accel_temp[1]*2*halfvy+accel_temp[2]*2*halfvz-9.8f-Vertically_ACCEL_Bias/Vertically_ACCEL_Count-Vertically_ACCEL_Dirft)*ACCEL_LOWPASS_Kp+(1.0f-ACCEL_LOWPASS_Kp)*Vertically_ACCEL_Last;//竖直方向总加速度减去重力加速度得到纯运动加速度
			
			
			//二次积分得到高度
			//Vertically_Altitude+=0.5*(Vertically_Velocity+Vertically_Velocity_Last)*0.005;
			//Vertically_Altitude+=(Vertically_Velocity)*dt;
			//Vertically_Altitude+=Vertically_ACCEL*0.005*0.005/2;
		}
			
			
			// Error is sum of cross product between estimated direction and measured direction of field vectors
			halfex += data.accDataProcessed[1] * halfvz - data.accDataProcessed[2] * halfvy;
			halfey += data.accDataProcessed[2] * halfvx - data.accDataProcessed[0] * halfvz;
			halfez += data.accDataProcessed[0] * halfvy - data.accDataProcessed[1] * halfvx;
//		}
	}

	// Apply feedback only when valid data has been gathered from the accelerometer or magnetometer
	if(halfex != 0.0f && halfey != 0.0f && halfez != 0.0f) {
		// Compute and apply integral feedback if enabled
		if(twoKi > 0.0f) {
			gyro_bias[0] += twoKi * halfex * dt;	// integral error scaled by Ki
			gyro_bias[1] += twoKi * halfey * dt;
			gyro_bias[2] += twoKi * halfez * dt;
			
			// apply integral feedback
			data.gyroDataProcessed[0] += gyro_bias[0];
			data.gyroDataProcessed[1] += gyro_bias[1];
			data.gyroDataProcessed[2] += gyro_bias[2];
		}
		else {
			gyro_bias[0] = 0.0f;	// prevent integral windup
			gyro_bias[1] = 0.0f;
			gyro_bias[2] = 0.0f;
		}

		// Apply proportional feedback
		data.gyroDataProcessed[0] += twoKp * halfex;
		data.gyroDataProcessed[1] += twoKp * halfey;
		data.gyroDataProcessed[2] += twoKp * halfez;
		
	// Time derivative of quaternion. q_dot = 0.5*q\otimes omega.
	//! q_k = q_{k-1} + dt*\dot{q}
	//! \dot{q} = 0.5*q \otimes P(\omega)
	dq0 = 0.5f*(-q1 * data.gyroDataProcessed[0] - q2 * data.gyroDataProcessed[1] - q3 * data.gyroDataProcessed[2]);
	dq1 = 0.5f*(q0 * data.gyroDataProcessed[0] + q2 * data.gyroDataProcessed[2] - q3 * data.gyroDataProcessed[1]);
	dq2 = 0.5f*(q0 * data.gyroDataProcessed[1] - q1 * data.gyroDataProcessed[2] + q3 * data.gyroDataProcessed[0]);
	dq3 = 0.5f*(q0 * data.gyroDataProcessed[2] + q1 * data.gyroDataProcessed[1] - q2 * data.gyroDataProcessed[0]); 

	q0 += dt*dq0;
	q1 += dt*dq1;
	q2 += dt*dq2;
	q3 += dt*dq3;
	
	// Normalise quaternion
	recipNorm = 1/sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;

	// Auxiliary variables to avoid repeated arithmetic
    q0q0 = q0 * q0;
    q0q1 = q0 * q1;
    q0q2 = q0 * q2;
    q0q3 = q0 * q3;
    q1q1 = q1 * q1;
    q1q2 = q1 * q2;
   	q1q3 = q1 * q3;
    q2q2 = q2 * q2;
    q2q3 = q2 * q3;
    q3q3 = q3 * q3; 
	}
	
	// Convert q->R, This R converts inertial frame to body frame.
		Rot_matrix[0] = q0q0 + q1q1 - q2q2 - q3q3;// 11
		Rot_matrix[1] = 2.f * (q1*q2 + q0*q3);	// 12
		Rot_matrix[2] = 2.f * (q1*q3 - q0*q2);	// 13
		Rot_matrix[3] = 2.f * (q1*q2 - q0*q3);	// 21
		Rot_matrix[4] = q0q0 - q1q1 + q2q2 - q3q3;// 22
		Rot_matrix[5] = 2.f * (q2*q3 + q0*q1);	// 23
		Rot_matrix[6] = 2.f * (q1*q3 + q0*q2);	// 31
		Rot_matrix[7] = 2.f * (q2*q3 - q0*q1);	// 32
		Rot_matrix[8] = q0q0 - q1q1 - q2q2 + q3q3;// 33

		//1-2-3 Representation.
		//Equation (290) 
		//Representing Attitude: Euler Angles, Unit Quaternions, and Rotation Vectors, James Diebel.
		// Existing PX4 EKF code was generated by MATLAB which uses coloum major order matrix.
		euler[0] = atan2f(Rot_matrix[5], Rot_matrix[8]);	//! Roll
		euler[1] = -asinf(Rot_matrix[2]);									//! Pitch
		euler[2] = atan2f(Rot_matrix[1], Rot_matrix[0]);	 
		
		//DCM . ground to body
		
		//imu.accNED[2]=-imu.accNED[2];
		
		
		flightStatus.attitude.x=euler[0]* 180.0f / PI;
		flightStatus.attitude.y=euler[1]* 180.0f / PI;
		flightStatus.attitude.z=euler[2]* 180.0f / PI;
      //Attitude.Yaw_BY=Yaw_Calc(Mag_Data[X],Mag_Data[Y],Mag_Data[Z],euler[0],euler[1])* 180.0f / PI;
      
}

