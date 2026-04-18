#ifndef _NRF24L01_REG_H_
#define _NRF24L01_REG_H_

#include "stm32f10x.h"                  // Device header

//========================= NRF24L01 命令 ====================================
#define R_REGISTER       	0x00	// 读取寄存器命令
#define W_REGISTER       	0x20	// 写寄存器命令
#define RD_RX_PLOAD      	0x61	// 读取接收有效载荷命令 (PTX/PRX)
#define WR_TX_PLOAD      	0xA0	// 写 TX 有效载荷命令 (PTX)
#define FLUSH_TX         	0xE1	// 清除 TX FIFO 命令 (PTX/PRX)
#define FLUSH_RX         	0xE2	// 清除 RX FIFO 命令 (PTX/PRX)
#define REUSE_TX_PL      	0xE3	// 重用最后一次载荷命令 (PTX)
#define R_RX_PL_WID      	0x60	// 读取接收有效载荷宽度命令
#define NOP              	0xFF	// 空操作

//======================== NRF24L01 寄存器地址 ================================
#define CONFIG           	0x00	// 配置寄存器：启动/关闭 CRC、接收/发送模式等
#define EN_AA            	0x01	// 自动应答使能
#define EN_RXADDR        	0x02	// 启用接收地址
#define SETUP_AW         	0x03	// 地址宽度设置
#define SETUP_RETR       	0x04	// 自动重发设置
#define RF_CH            	0x05	// 频率通道
#define RF_SETUP         	0x06	// 发射参数设置（速率、功率等）
#define STATUS           	0x07	// 状态寄存器
#define OBSERVE_TX       	0x08	// 发送观察寄存器（丢包/重发统计）
#define CD               	0x09	// 载波检测 / 信道占用指示
#define RX_ADDR_P0       	0x0A	// 接收管道0地址
#define RX_ADDR_P1       	0x0B	// 接收管道1地址
#define RX_ADDR_P2       	0x0C	// 接收管道2地址
#define RX_ADDR_P3       	0x0D	// 接收管道3地址
#define RX_ADDR_P4       	0x0E	// 接收管道4地址
#define RX_ADDR_P5       	0x0F	// 接收管道5地址
#define TX_ADDR          	0x10	// 发送地址寄存器
#define RX_PW_P0         	0x11	// 接收管道0的有效载荷宽度
#define RX_PW_P1         	0x12	// 接收管道1的有效载荷宽度
#define RX_PW_P2         	0x13	// 接收管道2的有效载荷宽度
#define RX_PW_P3         	0x14	// 接收管道3的有效载荷宽度
#define RX_PW_P4         	0x15	// 接收管道4的有效载荷宽度
#define RX_PW_P5         	0x16	// 接收管道5的有效载荷宽度
#define FIFO_STATUS      	0x17	// FIFO 状态寄存器
#define DYNPD            	0x1C	// 动态有效载荷使能寄存器
#define FEATURE          	0x1D	// 特性寄存器（动态有效载荷、ACK 等）

//============================= RF24L01 状态位 ================================
// CONFIG 寄存器位
#define PWR_UP           	0x02	// 上电（Power Up）
#define PWR_DOWN         	0x00	// 断电（Power Down）
#define PRIM_RX          	0x01	// 接收模式位（PRX）
#define PRIM_TX          	0x00	// 发送模式位（PTX）

// 中断相关：通过 STATUS 寄存器的位判断中断来源
#define IRQ_STATUS       	0x70	// 中断位掩码
#define IRQ_CLEAR        	0x7E	// 清除中断标志位
#define RX_DR            	0x40	// 数据到达中断 (RX_DR)
#define TX_DS            	0x20	// 数据发送完成中断 (TX_DS)
#define MAX_RT           	0x10	// 达到最大重发次数中断 (MAX_RT)

#endif


