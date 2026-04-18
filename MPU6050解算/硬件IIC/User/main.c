#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "MPU6050.h"
#include "Serial.h"



uint8_t ID;


int main()
{
	MPU6050_Init();
	UART1_Init();
	
	ID=MPU6050_Get_ID();
	
	while(1)
	{
		
		AHRS_Geteuler();
		
//		printf("%d,%d,%d\n",mpu6050.Accel_Original[0],mpu6050.Accel_Original[1],mpu6050.Accel_Original[2]);
//		printf("%d,%d,%d\n",mpu6050.Gyro_Original[0],mpu6050.Gyro_Original[1],mpu6050.Gyro_Original[2]);
		printf("%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n",mpu6050.Pitch,mpu6050.Roll,mpu6050.Yaw,
		mpu6050.Accel_Offset[0],mpu6050.Accel_Offset[1],mpu6050.Accel_Offset[2],
		mpu6050.Gyro_Offset[0],mpu6050.Gyro_Offset[0],mpu6050.Gyro_Offset[0]);

		
		Delay_ms(10);
	}
}

