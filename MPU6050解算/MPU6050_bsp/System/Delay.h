#ifndef __DELAY_H
#define __DELAY_H

#include "stm32f10x.h"                  // Device header

#define delay_us(X)		Delay_us(X)
#define delay_ms(X)		Delay_ms(X)
#define delay_s(X)		Delay_s(X)

void Delay_us(uint32_t us);
void Delay_ms(uint32_t ms);
void Delay_s(uint32_t s);

#endif
