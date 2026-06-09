#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "OLED.h"
#include "MPU6050.h"
#include "Timer.h"



int main()
{
	OLED_Init();
	mpu6050_init();
	Timer_Init();
	
	while(1)
	{
		
		OLED_ShowFloatNum(0,16,mpu6050.Pitch,2,1,OLED_8X16);
		OLED_ShowFloatNum(0,32,mpu6050.Roll,2,1,OLED_8X16);
		OLED_ShowFloatNum(0,48,mpu6050.Yaw,3,1,OLED_8X16);
		OLED_Update();

	}
}

void TIM2_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) == SET)
	{
		AHRS_Geteuler();
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
	}
}

