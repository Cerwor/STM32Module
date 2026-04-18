#include "stm32f10x.h"                  // Device header
#include "MPU6050.h"
#include "MPU6050_Reg.h"

#define MPU6050_ADDRESS		0xD0


/*参数*/
uint8_t read_imu[5];
MPU6050_DEF mpu6050;

void MPU_6050_WaitEvent(I2C_TypeDef* I2Cx, uint32_t I2C_EVENT)
{
	uint32_t TimeOut;
	TimeOut=10000;
	while(I2C_CheckEvent(I2Cx,I2C_EVENT)!=SUCCESS)
	{
		TimeOut--;
		if(TimeOut==0)
		{
			
			I2C_GenerateSTOP(I2Cx, ENABLE);
			break;
		}
	}
}



void MPU6050_WriteReg(uint8_t RegAddress,uint8_t Data)
{
	I2C_GenerateSTART(I2C1,ENABLE);
	MPU_6050_WaitEvent(I2C1,I2C_EVENT_MASTER_MODE_SELECT);
	
	I2C_Send7bitAddress(I2C1,MPU6050_ADDRESS,I2C_Direction_Transmitter);
	MPU_6050_WaitEvent(I2C1,I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED);
	
	I2C_SendData(I2C1,RegAddress);
	MPU_6050_WaitEvent(I2C1,I2C_EVENT_MASTER_BYTE_TRANSMITTING);
	
	I2C_SendData(I2C1,Data);
	MPU_6050_WaitEvent(I2C1,I2C_EVENT_MASTER_BYTE_TRANSMITTED);
	
	I2C_GenerateSTOP(I2C1,ENABLE);
}

uint8_t MPU6050_ReadReg(uint8_t RegAddress)
{
	uint8_t Data;
	
	I2C_GenerateSTART(I2C1,ENABLE);
	MPU_6050_WaitEvent(I2C1,I2C_EVENT_MASTER_MODE_SELECT);
	
	I2C_Send7bitAddress(I2C1,MPU6050_ADDRESS,I2C_Direction_Transmitter);
	MPU_6050_WaitEvent(I2C1,I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED);
	
	I2C_SendData(I2C1,RegAddress);
	MPU_6050_WaitEvent(I2C1,I2C_EVENT_MASTER_BYTE_TRANSMITTED);
	
	I2C_GenerateSTART(I2C1,ENABLE);
	MPU_6050_WaitEvent(I2C1,I2C_EVENT_MASTER_MODE_SELECT);
	
	I2C_Send7bitAddress(I2C1,MPU6050_ADDRESS,I2C_Direction_Receiver);
	MPU_6050_WaitEvent(I2C1,I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED);
	
	I2C_AcknowledgeConfig(I2C1,DISABLE);
	I2C_GenerateSTOP(I2C1,ENABLE);
	
	MPU_6050_WaitEvent(I2C1,I2C_EVENT_MASTER_BYTE_RECEIVED);
	Data=I2C_ReceiveData(I2C1);
	
	I2C_AcknowledgeConfig(I2C1,ENABLE);
	
	return Data;
}

void MPU6050_Init(void)
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_OD;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	I2C_InitTypeDef I2C_InitStructure;
	I2C_InitStructure.I2C_Ack=I2C_Ack_Enable;
	I2C_InitStructure.I2C_AcknowledgedAddress=I2C_AcknowledgedAddress_7bit;
	I2C_InitStructure.I2C_ClockSpeed=100000;
	I2C_InitStructure.I2C_DutyCycle=I2C_DutyCycle_2;
	I2C_InitStructure.I2C_Mode=I2C_Mode_I2C;
	I2C_InitStructure.I2C_OwnAddress1=0x00;
	I2C_Init(I2C1,&I2C_InitStructure);
	
	I2C_Cmd(I2C1,ENABLE);
	
	MPU6050_WriteReg(MPU6050_PWR_MGMT_1,0x01);
	MPU6050_WriteReg(MPU6050_PWR_MGMT_2,0x00);
	MPU6050_WriteReg(MPU6050_SMPLRT_DIV,0x09);
	MPU6050_WriteReg(MPU6050_CONFIG,0x06);
	MPU6050_WriteReg(MPU6050_GYRO_CONFIG,0x18);
	MPU6050_WriteReg(MPU6050_ACCEL_CONFIG,0x18);
}

uint8_t MPU6050_Get_ID(void)
{
	return MPU6050_ReadReg(MPU6050_WHO_AM_I);
}

void MPU6050_GetData(void)
{
	uint8_t DataH,DataL;
	int16_t temp;
	
	DataH = MPU6050_ReadReg(MPU6050_ACCEL_XOUT_H);
	DataL = MPU6050_ReadReg(MPU6050_ACCEL_XOUT_L);
	mpu6050.Accel_Original[0]=(DataH<<8)|DataL;
	
	DataH = MPU6050_ReadReg(MPU6050_ACCEL_YOUT_H);
	DataL = MPU6050_ReadReg(MPU6050_ACCEL_YOUT_L);
	mpu6050.Accel_Original[1]=(DataH<<8)|DataL;
	
	DataH = MPU6050_ReadReg(MPU6050_ACCEL_ZOUT_H);
	DataL = MPU6050_ReadReg(MPU6050_ACCEL_ZOUT_L);
	mpu6050.Accel_Original[2]=(DataH<<8)|DataL;
	
	DataH = MPU6050_ReadReg(MPU6050_GYRO_XOUT_H);
	DataL = MPU6050_ReadReg(MPU6050_GYRO_XOUT_L);
	mpu6050.Gyro_Original[0]=(DataH<<8)|DataL;
	
	DataH = MPU6050_ReadReg(MPU6050_GYRO_YOUT_H);
	DataL = MPU6050_ReadReg(MPU6050_GYRO_YOUT_L);
	mpu6050.Gyro_Original[1]=(DataH<<8)|DataL;
	
	DataH = MPU6050_ReadReg(MPU6050_GYRO_ZOUT_H);
	DataL = MPU6050_ReadReg(MPU6050_GYRO_ZOUT_L);
	mpu6050.Gyro_Original[2]=(DataH<<8)|DataL;
	
	DataH = MPU6050_ReadReg(MPU6050_TEMP_OUT_H);
	DataL = MPU6050_ReadReg(MPU6050_TEMP_OUT_L);
	temp = (DataH<<8)|DataL;
	mpu6050.temperature = 36.53f+(float)(temp/340.0f);	
	
}


/*---------------陀螺仪采集---------------------*/
#define GYRO_GATHER   	700 
#define RtA 				57.324841f				
#define AtR    			0.0174533f				
#define Acc_G 			0.0011963f				
#define Gyro_G 			0.03051756f				
#define Gyro_Gr			0.0005426f

#define Offset_Times 	200.0		//上电校准次数
#define Sampling_Time	0.01		//采样读取时间10ms

void MPU6050_ReadDatas_Proc(void)
{
	static uint16_t time=0;//初始化校准次数
	MPU6050_GetData();
	if(time<Offset_Times)//计算初始校准值
	{time++;
		mpu6050.Accel_Offset[0]+=(float)mpu6050.Accel_Original[0]/Offset_Times;//读取数据计算偏差
		mpu6050.Accel_Offset[1]+=(float)mpu6050.Accel_Original[1]/Offset_Times;//读取数据计算偏差
		mpu6050.Accel_Offset[2]+=(float)mpu6050.Accel_Original[2]/Offset_Times;//读取数据计算偏差
		mpu6050.Gyro_Offset[0] +=(float)mpu6050.Gyro_Original[0]/Offset_Times;//读取数据计算偏差
		mpu6050.Gyro_Offset[1] +=(float)mpu6050.Gyro_Original[1]/Offset_Times;//读取数据计算偏差
		mpu6050.Gyro_Offset[2] +=(float)mpu6050.Gyro_Original[2]/Offset_Times;//读取数据计算偏差
	}
	else
	{	// 加速度值赋值（减去零漂）
		mpu6050.Accel_Calulate[0] = mpu6050.Accel_Original[0];// - mpu6050.Accel_Offset[0];//角加速度不用
		mpu6050.Accel_Calulate[1] = mpu6050.Accel_Original[1];// - mpu6050.Accel_Offset[1];
		mpu6050.Accel_Calulate[2] = mpu6050.Accel_Original[2];// - mpu6050.Accel_Offset[2];
		// 陀螺仪值赋值（减去零漂）
		mpu6050.Gyro_Calulate[0] = mpu6050.Gyro_Original[0] - mpu6050.Gyro_Offset[0];
		mpu6050.Gyro_Calulate[1] = mpu6050.Gyro_Original[1] - mpu6050.Gyro_Offset[1];
		mpu6050.Gyro_Calulate[2] = mpu6050.Gyro_Original[2] - mpu6050.Gyro_Offset[2];
		
		/***********角加速度滤波***********/
		//加速度卡尔曼滤波方法：
		static struct KalmanFilter EKF[3]={{0.02,0,0,0,0.001,0.543},{0.02,0,0,0,0.001,0.543},{0.02,0,0,0,0.001,0.543}};
		kalmanfiter(&EKF[0],(float)mpu6050.Accel_Calulate[0]);  
		mpu6050.Accel_Average[0] =  (int16_t)EKF[0].Out;
		kalmanfiter(&EKF[1],(float)mpu6050.Accel_Calulate[1]);  
		mpu6050.Accel_Average[1] =  (int16_t)EKF[1].Out;
		kalmanfiter(&EKF[2],(float)mpu6050.Accel_Calulate[2]);  
		mpu6050.Accel_Average[2] =  (int16_t)EKF[2].Out;

		/*******************角速度滤波********************/
		static float x,y,z;
		//陀螺仪值一阶低通滤波（上个数据，现在的数据，互补滤波的系数）也称互补滤波法
		mpu6050.Gyro_Average[0] = LPF_1st(x,mpu6050.Gyro_Calulate[0],0.386f);	x = mpu6050.Gyro_Average[0];
		mpu6050.Gyro_Average[1]=  LPF_1st(y,mpu6050.Gyro_Calulate[1],0.386f);	y = mpu6050.Gyro_Average[1];
		mpu6050.Gyro_Average[2] = LPF_1st(z,mpu6050.Gyro_Calulate[2],0.386f);   z = mpu6050.Gyro_Average[2];
	}
	
	
}


#define MPU_Aceel_Gyro_Kp	0.95
float pitch2,roll2,Yaw;
float Gyro_Z_Measeure = 0;

void AHRS_Geteuler(void)
{
	MPU6050_ReadDatas_Proc();//读取滤波数据
	
	float ax,ay,az;
	ax=mpu6050.Accel_Average[0];
	ay=mpu6050.Accel_Average[1];
	az=mpu6050.Accel_Average[2];
	
	//角加速度和角速度解算的 俯仰角 和 横滚角 进行结合
	float pitch1	= 	RtA*atan(ay/sqrtf(ax*ax+az*az));  // 俯仰角
	float roll1		=	-RtA*atan(ax/sqrtf(ay*ay+az*az)); // 横滚角
	pitch2  += (mpu6050.Gyro_Average[0])*2000.0f/32768.0f*Sampling_Time;//俯仰角
	roll2	+= (mpu6050.Gyro_Average[1])*2000.0f/32768.0f*Sampling_Time;//横滚角
	mpu6050.Pitch =	 pitch1*MPU_Aceel_Gyro_Kp+pitch2*(1-MPU_Aceel_Gyro_Kp);		// 俯仰角
	mpu6050.Roll  =  roll1*MPU_Aceel_Gyro_Kp+roll2*(1-MPU_Aceel_Gyro_Kp);	 		// 横滚角
	
	//z轴不需要更改，足够稳定了
	Gyro_Z_Measeure = (mpu6050.Gyro_Average[2])*2000/32768.0;
	Yaw += Gyro_Z_Measeure*Sampling_Time;
	mpu6050.Yaw  = 	Yaw;
}














float LPF_1st(float oldData, float newData, float lpf_factor)
{
	return oldData * (1 - lpf_factor) + newData * lpf_factor;    //上个数据*一定的比例+现在的数据*一定的比例
}
//一维卡尔曼滤波
void kalmanfiter(struct KalmanFilter *EKF,float input)
{
	EKF->NewP = EKF->LastP + EKF->Q;
	EKF->Kg = EKF->NewP / (EKF->NewP + EKF->R);
	EKF->Out = EKF->Out + EKF->Kg * (input - EKF->Out);
	EKF->LastP = (1 - EKF->Kg) * EKF->NewP;
}
