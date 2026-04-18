#include "Delay.h"


void Delay_Init(void)
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
    
	TIM4->PSC = 72 - 1;      // 预分频，使计数器 1us 计数一次
	TIM4->ARR = 0xFFFF;      // 最大重装载值
	TIM4->EGR = TIM_EGR_UG;  // 手动产生更新事件，使 PSC 立即生效
}


/**
  * @brief  微秒级延时
  * @param  xus 延时时长，范围：0~65535
  * @retval 无
  */
void Delay_us(uint32_t xus)
{
	if (xus == 0) return;

	TIM4->CNT = 0;           // 清空计数值
	TIM4->CR1 |= TIM_CR1_CEN; // 开启定时器
	
	while(TIM4->CNT < xus);
	
	TIM4->CR1 &= ~TIM_CR1_CEN; // 关闭定时器
}

/**
  * @brief  毫秒级延时
  * @param  xms 延时时长，范围：0~4294967295
  * @retval 无
  */

void Delay_ms(uint32_t xms)
{
	while(xms--)
	{
		Delay_us(1000);
	}
}

/**
  * @brief  秒级延时
  * @param  xs 延时时长，范围：0~4294967295
  * @retval 无
  */

void Delay_s(uint32_t xs)
{
	while(xs--)
	{
		Delay_ms(1000);
	}
} 

