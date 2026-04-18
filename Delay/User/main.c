#include "stm32f10x.h"                  // Device header
#include "Delay.h"


void LED_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
}






int main(void)
{
	LED_Init();
	Delay_Init();
	while (1)
	{
		GPIO_SetBits(GPIOA,GPIO_Pin_8);
		Delay_ms(1000);
		GPIO_ResetBits(GPIOA,GPIO_Pin_8);
		Delay_ms(1000);
	}
}
