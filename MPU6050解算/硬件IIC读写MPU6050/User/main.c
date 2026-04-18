#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "OLED.h"
#include "MPU6050.h"

int16_t AX,AY,AZ,GX,GY,GZ;
uint8_t ID;

int main()
{
	
	OLED_Init();
	MPU6050_Init();
	OLED_ShowString(1,1,"ID:");
	ID=MPU6050_Get_ID();
	OLED_ShowHexNum(1,4,ID,2);
	while(1)
	{
		AX=mpu_data.AccX;AY=mpu_data.AccY;AZ=mpu_data.AccZ;
		GX=mpu_data.GyroX;GY=mpu_data.GyroY;GZ=mpu_data.GyroZ;
		
		MUP6050_GetData();
		
		OLED_ShowSignedNum(2,1,AX,5);
		OLED_ShowSignedNum(3,1,AY,5);
		OLED_ShowSignedNum(4,1,AZ,5);
		
		OLED_ShowSignedNum(2,8,GX,5);
		OLED_ShowSignedNum(3,8,GY,5);
		OLED_ShowSignedNum(4,8,GZ,5);
	}
}

