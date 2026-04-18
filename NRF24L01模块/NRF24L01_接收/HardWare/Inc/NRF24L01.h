#ifndef _NRF24L01_H_
#define _NRF24L01_H_

#include "stm32f10x.h"                  // Device header
#include "NRF24L01_Reg.h"
#include "Delay.h"
#include <string.h>


//引脚定义
#define NRF_CSN_Port			GPIOA
#define NRF_CSN_Pin				GPIO_Pin_4

#define NRF_CE_Port				GPIOA
#define NRF_CE_Pin				GPIO_Pin_3

#define NRF_IRQ_Port			GPIOA
#define NRF_IRQ_Pin				GPIO_Pin_2 
#define NRF_IRQ_PortSource		GPIO_PortSourceGPIOA
#define NRF_IRQ_PinSource		GPIO_PinSource2
#define NRF_IRQ_EXTI_Line		EXTI_Line2
#define NRF_IRQ_NVIC_Channel	EXTI2_IRQn

// 引脚控制宏
#define NRF_CSN_HIGH			(NRF_CSN_Port->BSRR = NRF_CSN_Pin)
#define NRF_CSN_LOW				(NRF_CSN_Port->BRR  = NRF_CSN_Pin)

#define NRF_CE_HIGH				(NRF_CE_Port->BSRR  = NRF_CE_Pin)
#define NRF_CE_LOW				(NRF_CE_Port->BRR   = NRF_CE_Pin)


#define DYNAMIC_PLOAD_LENGTH	1			// [0 = 固定有效载荷] [1 = 动态有效载荷]
#define STATIC_PLOAD_LENGTH		32		    // 静态有效载荷长度（当不使用动态有效载荷时）
#define ADDRESS_WIDTH					5			// 通信地址宽度（字节）



typedef struct {
	
	float A1;
	float B2;
	
}TempStruct;


extern TempStruct Temp1;






void NRF24L01_Init(void);
void NRF_TX_Mode(void);
void NRF_RX_Mode(void);
uint8_t NRF_SendPacket(uint8_t* Tx_BUFF,uint8_t Len);





#endif
