#include "stm32f10x.h"                  // Device header
#include "NRF24L01.h"
#include "Delay.h"






int main(void)
{
	NRF24L01_Init();
	NRF_TX_Mode();
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC,&GPIO_InitStructure);
	GPIO_SetBits(GPIOC,GPIO_Pin_13);
	
	
	
	
	while (1)
	{
		Temp1.A1 += 0.1f;
		Temp1.B2 += 0.2f;
		Delay_ms(100);
		
		if(NRF_SendPacket((uint8_t *)&Temp1,sizeof(Temp1)) == 0)
		{
			GPIOC->ODR ^= GPIO_Pin_13;
		}
		
	}
}
