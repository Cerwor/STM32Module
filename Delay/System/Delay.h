#ifndef __DELAY_H_
#define __DELAY_H_

#include "stm32f10x.h"                  // Device header

void Delay_Init(void);
void Delay_us(uint32_t xus);
void Delay_ms(uint32_t xms);
void Delay_s(uint32_t xs);


#endif


