#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "OLED.h"
#include "bsp_uart.h"
#include "stdio.h"
#include "bsp_mpu6050.h"
#include "inv_mpu.h"



uint32_t Temp;
float pitch=0,roll=0,yaw=0;   //≈∑¿≠Ω«

int main(void)
{
	OLED_Init();
	uart1_init(115200);
	MPU6050_Init();
	//DMP≥ı ºªØ
	mpu_dmp_init();
	
	while (1)
	{
		if( mpu_dmp_get_data(&pitch,&roll,&yaw) == 0 )
		{ 
			OLED_Printf(0,0,OLED_8X16,"Pitch;%.2f",pitch);
			OLED_Printf(0,16,OLED_8X16,"Roll;%.2f",roll);
			OLED_Printf(0,32,OLED_8X16,"Yaw;%.2f",yaw);
			printf("%.2f,%.2f,%.2f\r\n", pitch, roll, yaw);
		}      
		OLED_Update();
	}
}
