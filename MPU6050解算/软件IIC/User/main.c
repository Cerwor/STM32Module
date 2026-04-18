#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "OLED.h"
#include "MPU6050.h"



int main()
{
	OLED_Init();
	mpu6050_init();
	
	while(1)
	{
		MPU6050_ReadDatas_Proc();
		AHRS_Geteuler();
		OLED_ShowFloatNum(0,16,mpu6050.Pitch,2,1,OLED_8X16);
		OLED_ShowFloatNum(0,32,mpu6050.Roll,2,1,OLED_8X16);
		OLED_ShowFloatNum(0,48,mpu6050.Yaw,2,1,OLED_8X16);
		OLED_Update();
		Delay_ms(10);
	}
}

