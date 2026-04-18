#ifndef		__MPU6050_H__
#define		__MPU6050_H__

#include "stm32f10x.h"                  // Device header
#include "MPU6050_Reg.h"
#include "math.h"

typedef struct 
{	//欧拉角
	float Pitch;
	float Roll;
	float Yaw;
	//温度
	float temperature;
	//原始值
	int16_t  Accel_Original[3];
	int16_t Gyro_Original[3];
	//校准值
	float Accel_Offset[3];
	float Gyro_Offset[3];
	//校准后的值
	float Accel_Calulate[3];
	float Gyro_Calulate[3]; 
	//滤波后的值
	float Accel_Average[3];
	float Gyro_Average[3];   
}MPU6050_DEF;



//卡尔曼滤波参数结构体
struct KalmanFilter{
	float LastP;			//上一次协方差
	float NewP;				//最新的协方差
	float Out;				//卡尔曼输出
	float Kg;				//卡尔曼增益
	float Q;				//过程噪声的协方差
	float R;				//观测噪声的协方差
};

extern MPU6050_DEF mpu6050;
extern float Gyro_Z_Measeure;

void MPU6050_Init(void);
void AHRS_Geteuler(void);
uint8_t MPU6050_Get_ID(void);



float LPF_1st(float oldData, float newData, float lpf_factor);
void kalmanfiter(struct KalmanFilter *EKF,float input);

#endif

