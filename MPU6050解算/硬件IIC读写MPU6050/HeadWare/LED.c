#include "stm32f10x.h"                  // Device header

void LED_Init()
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_0 | GPIO_Pin_1;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_SetBits(GPIOA,GPIO_Pin_0);
	GPIO_SetBits(GPIOA,GPIO_Pin_1);
}

void LED1_ON(void)
{
	GPIO_WriteBit(GPIOA,GPIO_Pin_0,Bit_RESET);
}

void LED2_ON(void)
{
	GPIO_WriteBit(GPIOA,GPIO_Pin_1,Bit_RESET);
}

void LED1_OFF(void)
{
	GPIO_WriteBit(GPIOA,GPIO_Pin_0,Bit_SET);
}

void LED2_OFF(void)
{
	GPIO_WriteBit(GPIOA,GPIO_Pin_1,Bit_SET);
}
