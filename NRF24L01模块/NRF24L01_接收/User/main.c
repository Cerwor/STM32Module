#include "stm32f10x.h"                  // Device header
#include "NRF24L01.h"
#include "Delay.h"
#include "OLED.h"





int main(void)
{
	OLED_Init();
	NRF24L01_Init();
	NRF_RX_Mode();
	
	while (1)
	{
		OLED_Printf(0,0,OLED_8X16,"%.2f",Temp1.A1);
		OLED_Printf(0,16,OLED_8X16,"%.2f",Temp1.A1);
		OLED_Update();
	}
}
