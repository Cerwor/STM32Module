#include "stm32f10x.h"                  // Device header
#include "NRF24L01.h"

uint8_t RX_ADDRESS[5] = {0xa5,0xa5,0xa5,0xa5,0xa5};
uint8_t TX_ADDRESS[5] = {0xa5,0xa5,0xa5,0xa5,0xa5};
uint8_t RX_DATA[STATIC_PLOAD_LENGTH];


TempStruct Temp1;


void SIP1_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);

	GPIO_InitTypeDef GPIO_InitStructure;
	//SCK
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	//MISO
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	//MOSI
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	//CSN
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin = NRF_CSN_Pin;
	GPIO_Init(NRF_CSN_Port,&GPIO_InitStructure);
	//CE
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin = NRF_CE_Pin;
	GPIO_Init(NRF_CE_Port,&GPIO_InitStructure);
	//IRQ
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Pin = NRF_IRQ_Pin;
	GPIO_Init(NRF_IRQ_Port,&GPIO_InitStructure);

	SPI_InitTypeDef SPI_InitStructure;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_Init(SPI1,&SPI_InitStructure);
	SPI_Cmd(SPI1,ENABLE);

	GPIO_EXTILineConfig(NRF_IRQ_PortSource,NRF_IRQ_PinSource);
	EXTI_InitTypeDef EXTI_InitStructure;
	EXTI_InitStructure.EXTI_Line = NRF_IRQ_EXTI_Line;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_Init(&EXTI_InitStructure);

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = NRF_IRQ_NVIC_Channel;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_Init(&NVIC_InitStructure);
}


uint8_t SPI1_ReadWriteByte(uint8_t data)
{
	uint8_t temp = 0;
	// 等待发送缓冲区空，使用 temp 防止死循环
	while(!(SPI1->SR & SPI_I2S_FLAG_TXE))
	{
			temp++;
			if (temp > 200){return 0xFF;}
	}
	// 写入数据到 SPI 数据寄存器
	SPI1->DR = data;
	temp = 0;

	// 等待接收完成，使用 temp 防止死循环
	while(!(SPI1->SR & SPI_I2S_FLAG_RXNE))
	{
			temp++;
			if (temp > 200){return 0xFF;}
	}
	return SPI1->DR;
}

// 读取寄存器的值
uint8_t NRF_SPI_ReadReg(uint8_t Reg)
{
	uint8_t Reg_val;

	NRF_CSN_LOW;                // 片选
	SPI1_ReadWriteByte(R_REGISTER | Reg);
	Reg_val = SPI1_ReadWriteByte(NOP);
	NRF_CSN_HIGH;               // 取消片选

	return Reg_val;
}

// 读取缓冲区数据
uint8_t NRF_SPI_ReadBuf(uint8_t Reg,uint8_t* Buf,uint8_t Len)
{
	uint8_t Status;

	NRF_CSN_LOW;                // 片选
	Status = SPI1_ReadWriteByte(R_REGISTER | Reg);
	for(uint8_t i=0;i<Len;i++)
	{
			Buf[i] = SPI1_ReadWriteByte(NOP);
	}
	NRF_CSN_HIGH;               // 取消片选

	return Status;
}



// 写寄存器
uint8_t NRF_SPI_WriteReg(uint8_t Reg,uint8_t Data)
{
	uint8_t Status;

	NRF_CSN_LOW;                // 片选
	Status = SPI1_ReadWriteByte(W_REGISTER | Reg);
	SPI1_ReadWriteByte(Data);
	NRF_CSN_HIGH;               // 取消片选

	return Status;
}

// 向寄存器写入缓冲区数据
uint8_t NRF_SPI_WriteBuf(uint8_t Reg ,uint8_t* Buf,uint8_t Len)
{
	uint8_t Status;

	NRF_CSN_LOW;                // 片选
	Status = SPI1_ReadWriteByte(W_REGISTER | Reg);
	for(uint8_t i=0;i<Len;i++)
	{
			SPI1_ReadWriteByte(*Buf++);
	}
	NRF_CSN_HIGH;               // 取消片选

	return Status;
}

// 发送单字节命令并返回状态
uint8_t NRF_SPI_SendCmd(uint8_t Cmd)
{
	uint8_t Status;

	NRF_CSN_LOW;
	Status = SPI1_ReadWriteByte(Cmd);
	NRF_CSN_HIGH;

	return Status;
}



void NRF24L01_Init(void)
{
	SIP1_Init();
	// 复位 CE，进入配置模式
	NRF_CE_LOW;
	Delay_ms(15);
	// 配置寄存器
	NRF_SPI_WriteReg(EN_AA,0x01);                       // 使能通道0自动应答
	NRF_SPI_WriteReg(EN_RXADDR,0x01);                   // 启用通道0接收地址
	NRF_SPI_WriteReg(SETUP_AW,0x03);                    // 地址宽度为5字节
	NRF_SPI_WriteReg(SETUP_RETR,0x35);                  // 自动重发间隔和次数（示例：750us，重发5次）
	NRF_SPI_WriteReg(RF_CH,0x02);                       // 频道设置为2402 MHz
	NRF_SPI_WriteReg(RF_SETUP,0x07);                    // RF 参数：1Mbps，0dBm
	NRF_SPI_WriteBuf(RX_ADDR_P0,RX_ADDRESS,ADDRESS_WIDTH);          // 写入接收通道0地址
	NRF_SPI_WriteBuf(TX_ADDR,TX_ADDRESS,ADDRESS_WIDTH);             // 写入发送地址
#if DYNAMIC_PLOAD_LENGTH
	NRF_SPI_WriteReg(DYNPD, 0x01);                      // 使能通道0动态有效载荷
	NRF_SPI_WriteReg(FEATURE,0x07);                     // 使能动态有效载荷、ack 和相关功能
#else
	NRF_SPI_WriteReg(RX_PW_P0,STATIC_PLOAD_LENGTH);     // 通道0固定有效载荷长度
	NRF_SPI_WriteReg(DYNPD, 0x00);                      // 关闭动态有效载荷
	NRF_SPI_WriteReg(FEATURE, 0x00);                    // 关闭额外功能
#endif
	NRF_SPI_WriteReg(STATUS, IRQ_CLEAR);       	        // 清除中断标志位
	// 清空 FIFO
	NRF_SPI_SendCmd(FLUSH_TX);
	NRF_SPI_SendCmd(FLUSH_RX);
	// 配置 CONFIG 寄存器为接收模式并启用中断
	NRF_SPI_WriteReg(CONFIG,0x3B);                      // 配置为 PRX 模式，开启相关中断
	Delay_ms(10);
	NRF_CE_HIGH;
}


void NRF_TX_Mode(void)
{
	NRF_SPI_WriteReg(STATUS,IRQ_CLEAR);         // 清除中断
	NRF_SPI_SendCmd(FLUSH_RX);                  // 清空 FIFO
	NRF_SPI_SendCmd(FLUSH_TX);
	NRF_SPI_WriteReg(CONFIG,0x7A);              // 关闭部分中断并切换到 PTX（发射）模式
	Delay_ms(2);
}

void NRF_RX_Mode(void)
{
	NRF_SPI_WriteReg(STATUS,IRQ_CLEAR);         // 清除中断
	NRF_SPI_SendCmd(FLUSH_RX);                  // 清空 FIFO
	NRF_SPI_SendCmd(FLUSH_TX);
	NRF_SPI_WriteReg(CONFIG,0x3B);              // 配置为 PRX（接收）模式并启用中断
	Delay_ms(2);
}


//返回值：0 代表发送成功并收到应答
//1 代表达到最大重发次数（未收到应答）
//0xFF 代表硬件故障
uint8_t NRF_SendPacket(uint8_t* Tx_BUFF,uint8_t Len)
{
	uint8_t Status;
	
	NRF_CE_LOW;             // 拉低 CE，准备发送
	NRF_SPI_WriteBuf(WR_TX_PLOAD,Tx_BUFF,Len);        // 写入待发送的数据
	NRF_CE_HIGH;            // 拉高 CE，开始发送
	Delay_us(20);
	NRF_CE_LOW;             // 拉低 CE，结束发送周期
	
	while(1)
	{
		Status = NRF_SPI_ReadReg(STATUS); // 读取状态
		if(Status & TX_DS) // 收到应答，发送成功
		{
			NRF_SPI_WriteReg(STATUS, TX_DS); // 清除标志位
			return 0; 
		}
		if(Status & MAX_RT) // 达到重发上限，没收到应答
		{
			NRF_SPI_WriteReg(STATUS, MAX_RT); // 清除标志位
			NRF_SPI_SendCmd(FLUSH_TX);        // 清空TX FIFO
			return 1;
		}
	}
}

void NRF_ReceivePacket(void)
{
	uint8_t Len;

	NRF_CSN_LOW;
	SPI1_ReadWriteByte(R_RX_PL_WID);
	Len = SPI1_ReadWriteByte(NOP);
	NRF_CSN_HIGH;
	// 判断接收到的长度是否正确
	if(Len>32)
	{
			NRF_SPI_SendCmd(FLUSH_RX);
			NRF_SPI_WriteReg(STATUS,IRQ_CLEAR);        // 清除中断
			return;
	}
	NRF_SPI_ReadBuf(RD_RX_PLOAD,(uint8_t*)RX_DATA,Len);
	if(Len == sizeof(Temp1))
	{
			memcpy(&Temp1, RX_DATA, sizeof(Temp1));
	}
	NRF_SPI_WriteReg(STATUS,IRQ_CLEAR);            // 清除中断
}


void EXTI2_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line2) != RESET)
	{
		EXTI_ClearITPendingBit(EXTI_Line2);

		NRF_ReceivePacket();
	}
}

