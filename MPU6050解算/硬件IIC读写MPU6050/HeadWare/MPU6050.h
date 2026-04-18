#include "stm32f10x.h"                  // Device header

#ifndef		__MPU6050_H__
#define		__MPU6050_H__

typedef struct {
	int16_t AccX;int16_t AccY;int16_t AccZ;
	int16_t GyroX;int16_t GyroY;int16_t GyroZ;
}MPU_6050_Data;
extern MPU_6050_Data mpu_data;


void MPU6050_Init(void);
void MPU6050_WriteReg(uint8_t RegAddress,uint8_t Data);
uint8_t MPU6050_ReadReg(uint8_t RegAddress);
void MUP6050_GetData();
uint8_t MPU6050_Get_ID(void);

#endif

