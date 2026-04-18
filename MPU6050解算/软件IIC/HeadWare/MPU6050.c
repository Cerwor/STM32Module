#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include <math.h>
#include "MPU6050.h"


/*引脚配置*/
#define MyI2C_SCL        GPIO_Pin_10    //PB
#define MyI2C_SDA        GPIO_Pin_11   //PB


/*寄存器定义*/
#define MPU6050_ADDR   (0x68 << 1)
#define imu_adress 0x68

#define	SMPLRT_DIV						0x19
#define	MPU_CONFIG						0x1A
#define	GYRO_CONFIG						0x1B
#define	ACCEL_CONFIG	        0x1C
#define	ACCEL_XOUT_H	        0x3B
#define	ACCEL_XOUT_L	        0x3C
#define	ACCEL_YOUT_H	        0x3D
#define	ACCEL_YOUT_L	        0x3E
#define	ACCEL_ZOUT_H	        0x3F
#define	ACCEL_ZOUT_L	        0x40
#define	TEMP_OUT_H						0x41
#define	TEMP_OUT_L						0x42
#define	GYRO_XOUT_H						0x43
#define	GYRO_XOUT_L						0x44
#define	GYRO_YOUT_H						0x45
#define	GYRO_YOUT_L						0x46
#define	GYRO_ZOUT_H						0x47
#define	GYRO_ZOUT_L						0x48
#define	PWR_MGMT_1						0x6B
#define	WHO_AM_I							0x75
#define USER_CTRL							0x6A
#define INT_PIN_CFG						0x37
#define INT_ENABLE        		0x38
#define INT_PIN_CFG       		0x37

/*参数*/
uint8_t read_imu[5];
MPU6050_DEF mpu6050;

/*函数声明*/
void kalmanfiter(struct KalmanFilter *EKF,float input);
float LPF_1st(float oldData, float newData, float lpf_factor);
double IIR_I_Filter(double InData, double *x, double *y, double *b, short nb, double *a, short na);
void AHRS_Geteuler(void);

//*************************************************IIC代码************************************************//

/**
  * @brief  I2C写SCL函数
  * @param  BitValue 要写入的值，范围：0、1
  * @retval 无
  */
void MyI2C_W_SCL(uint8_t BitValue)
{
	GPIO_WriteBit(GPIOB, MyI2C_SCL, (BitAction)BitValue);
	Delay_us(10);
}

/**
  * @brief  I2C写SDA函数
  * @param  BitValue 要写入的值，范围：0、1
  * @retval 无
  */
void MyI2C_W_SDA(uint8_t BitValue)
{
	GPIO_WriteBit(GPIOB, MyI2C_SDA, (BitAction)BitValue);
	Delay_us(10);
}

/**
  * @brief  I2C读SDA函数
  * @param  无
  * @retval 返回读取的值，范围：0、1
  */
uint8_t MyI2C_R_SDA(void)
{
	uint8_t BitValue;

	BitValue = GPIO_ReadInputDataBit(GPIOB, MyI2C_SDA);
	Delay_us(10);
	return BitValue;
}

/**
  * @brief  I2C初始化函数，初始化PB10为I2C_SCL输出，PB11为I2C_SDA输出
  * @param  无
  * @retval 无
  */
void MyI2C_Init(void)
{
	 // 1) 开 AFIO 时钟，关 JTAG (保留 SWD)
//    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
//    AFIO->MAPR |= AFIO_MAPR_SWJ_CFG_JTAGDISABLE;  // 仅 SWD 保留，JTAG 全部关闭

    //开 GPIOB 时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    //配置 PB3/SDA, PB4/SCL 为开漏输出
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
    GPIO_InitStructure.GPIO_Pin  = MyI2C_SCL | MyI2C_SDA;
    GPIO_InitStructure.GPIO_Speed= GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_SetBits(GPIOB, MyI2C_SCL | MyI2C_SDA);
}

/**
  * @brief  I2C起始函数，SCL高电平期间，SDA由高电平切换到低电平
  * @param  无
  * @retval 无
  */
void MyI2C_Start(void)
{
	MyI2C_W_SDA(1);
	MyI2C_W_SCL(1);
	MyI2C_W_SDA(0);
	MyI2C_W_SCL(0);
}

/**
  * @brief  I2C终止函数，SCL高电平期间，SDA由低电平切换到高电平
  * @param  无
  * @retval 无
  */
void MyI2C_Stop(void)
{
	MyI2C_W_SDA(0);
	MyI2C_W_SCL(1);
	MyI2C_W_SDA(1);
}

/**
  * @brief  I2C发送一个字节函数，SCL低电平期间，主机将数据位依次放到SDA上（高位在前），拉高SCL，循环8次，
  *         从机在SCL高电平期间读取数据位
  * @param  Byte 要发送的字节
  * @retval 无
  */
void MyI2C_SendByte(uint8_t Byte)
{
	uint8_t i;

	for(i = 0; i < 8; i++)
	{
		MyI2C_W_SDA(Byte & (0x80 >> i));
		MyI2C_W_SCL(1);
		MyI2C_W_SCL(0);
	}
}

/**
  * @brief  I2C接收一个字节函数，SCL低电平期间，从机将数据位依次放到SDA上（高位在前），拉高SCL，循环8次，
  *         主机在SCL高电平期间读取数据位（主机在接收之前，需要释放SDA）
  * @param  无
  * @retval 返回接收的字节
  */
uint8_t MyI2C_ReceiveByte(void)
{
	uint8_t i, Byte = 0x00;

	MyI2C_W_SDA(1);
	for(i = 0; i < 8; i++)
	{
		MyI2C_W_SCL(1);
		if(MyI2C_R_SDA() == 1)  {Byte |= (0x80 >> i);}
		MyI2C_W_SCL(0);
	}
	return Byte;
}

/**
  * @brief  I2C发送应答函数，在接收完一个字节之后，主机在下一个时钟发送应答
  * @param  AckBit 要发送的应答位，0表示应答，1表示非应答
  * @retval 
  */
void MyI2C_SendAck(uint8_t AckBit)
{
	MyI2C_W_SDA(AckBit);
	MyI2C_W_SCL(1);
	MyI2C_W_SCL(0);
}

/**
  * @brief  I2C接收应答函数，在发送完一个字节之后，主机在下一个时钟接收一位数据，判断从机是否应答
  *         （主机在接收之前，需要释放SDA）
  * @param  无
  * @retval 返回接收的应答位，0表示应答，1表示非应答
  */
uint8_t MyI2C_ReceiveAck(void)
{
	uint8_t AckBit;
	
	MyI2C_W_SDA(1);
	MyI2C_W_SCL(1);
	AckBit = MyI2C_R_SDA();
	MyI2C_W_SCL(0);
	return AckBit;
}

//***********************************MPU6050代码********************************************//

void I2C_WriteReg_Byte(uint8_t reg_addr, uint8_t data) {
    MyI2C_Start();
    MyI2C_SendByte(MPU6050_ADDR | 0);    // 写模式
    MyI2C_ReceiveAck();
    MyI2C_SendByte(reg_addr);
    MyI2C_ReceiveAck();
    MyI2C_SendByte(data);
    MyI2C_ReceiveAck();
    MyI2C_Stop();
}

uint8_t I2C_ReadReg_Byte(uint8_t reg_addr) {
    uint8_t val;
    // 步骤 1：写要读的寄存器地址
    MyI2C_Start();
    MyI2C_SendByte(MPU6050_ADDR | 0);
    MyI2C_ReceiveAck();
    MyI2C_SendByte(reg_addr);
    MyI2C_ReceiveAck();
    // 步骤 2：重复起始，进入读模式
    MyI2C_Start();
    MyI2C_SendByte(MPU6050_ADDR | 1);
    MyI2C_ReceiveAck();
    val = MyI2C_ReceiveByte();
    MyI2C_SendAck(1); // NACK
    MyI2C_Stop();
    return val;
}

void I2C_WriteReg(uint8_t reg_addr, uint8_t *buf, uint8_t count) {
    MyI2C_Start();
    MyI2C_SendByte(MPU6050_ADDR | 0);
    MyI2C_ReceiveAck();
    MyI2C_SendByte(reg_addr);
    MyI2C_ReceiveAck();
    for (uint8_t i = 0; i < count; i++) {
        MyI2C_SendByte(buf[i]);
        MyI2C_ReceiveAck();
    }
    MyI2C_Stop();
}

void I2C_ReadReg(uint8_t reg_addr, uint8_t *buf, uint8_t count) {
    // 写寄存器地址
    MyI2C_Start();
    MyI2C_SendByte(MPU6050_ADDR | 0);
    MyI2C_ReceiveAck();
    MyI2C_SendByte(reg_addr);
    MyI2C_ReceiveAck();
    // 重启并读数据
    MyI2C_Start();
    MyI2C_SendByte(MPU6050_ADDR | 1);
    MyI2C_ReceiveAck();
    for (uint8_t i = 0; i < count; i++) {
        buf[i] = MyI2C_ReceiveByte();
        MyI2C_SendAck(i == count - 1 ? 1 : 0); // 最后一个 NACK，其它 ACK
    }
    MyI2C_Stop();
}

void Single_WriteI2C(uint8_t SlaveAddress, uint8_t REG_Address, uint8_t REG_data) {
    (void)SlaveAddress;               // 不再使用，地址固定
    I2C_WriteReg_Byte(REG_Address, REG_data);
}

uint8_t Single_ReadI2C(uint8_t SlaveAddress, uint8_t REG_Address) {
    (void)SlaveAddress;
    return I2C_ReadReg_Byte(REG_Address);
}

void mpu6050_init(void)
{
	MyI2C_Init();
	
  Single_WriteI2C(imu_adress,PWR_MGMT_1  , 0x00);//关闭所有中断,解除休眠
  Single_WriteI2C(imu_adress,SMPLRT_DIV  , 0x09);// sample rate.  Fsample= 1Khz/(<this value>+1) = 1000Hz
  Single_WriteI2C(imu_adress,MPU_CONFIG  , 0x06);//
  Single_WriteI2C(imu_adress,GYRO_CONFIG , 0x18);//
  Single_WriteI2C(imu_adress,ACCEL_CONFIG, 0x18);// 
	Single_WriteI2C(imu_adress,INT_ENABLE   , 0x01);
	
	read_imu[0]=Single_ReadI2C(imu_adress,PWR_MGMT_1);
	read_imu[1]=Single_ReadI2C(imu_adress,SMPLRT_DIV);
	read_imu[2]=Single_ReadI2C(imu_adress,MPU_CONFIG);
	read_imu[3]=Single_ReadI2C(imu_adress,GYRO_CONFIG);
	read_imu[4]=Single_ReadI2C(imu_adress,ACCEL_CONFIG);
	mpu6050.Accel_Offset[0] = mpu6050.Accel_Offset[1] = mpu6050.Accel_Offset[2] = 0.0f;
	mpu6050.Gyro_Offset[0]  = mpu6050.Gyro_Offset[1]  = mpu6050.Gyro_Offset[2]  = 0.0f;
	mpu6050.Pitch = mpu6050.Roll = mpu6050.Yaw = 0.0f;
	
}

void mpu6050_read(int16_t *gyro,int16_t *accel,float *temperature){
	uint8_t buf[14];
	int16_t temp;
	I2C_ReadReg(ACCEL_XOUT_H, buf, 14);
	accel[0]=(int16_t)((buf[0]<<8)|buf[1]);
	accel[1]=(int16_t)((buf[2]<<8)|buf[3]);
	accel[2]=(int16_t)((buf[4]<<8)|buf[5]);	
	temp		=(int16_t)((buf[6]<<8)|buf[7]);
	gyro[0]	=(int16_t)((buf[8]<<8)|buf[9]);
	gyro[1]	=(int16_t)((buf[10]<<8)|buf[11]);
	gyro[2]	=(int16_t)((buf[12]<<8)|buf[13]);	
	*temperature=36.53f+(float)(temp/340.0f);	
}


#define IIR_ORDER     4      //使用IIR滤波器的阶数
double b_IIR[IIR_ORDER+1] ={0.0008f, 0.0032f, 0.0048f, 0.0032f, 0.0008f};  //系数b
double a_IIR[IIR_ORDER+1] ={1.0000f, -3.0176f, 3.5072f, -1.8476f, 0.3708f};//系数a
double InPut_IIR[3][IIR_ORDER+1]  = {0};
double OutPut_IIR[3][IIR_ORDER+1] = {0};

/*---------------陀螺仪采集---------------------*/
#define GYRO_GATHER   	700 //原来是100
#define RtA 			57.324841f				
#define AtR    			0.0174533f				
#define Acc_G 			0.0011963f				
#define Gyro_G 			0.03051756f				
#define Gyro_Gr			0.0005426f

#define Offset_Times 	200.0		//上电校准次数
#define Sampling_Time	0.01		//采样读取时间10ms

void MPU6050_ReadDatas_Proc(void){
	static uint16_t time=0;//初始化校准次数
	mpu6050_read(mpu6050.Gyro_Original,mpu6050.Accel_Original,&mpu6050.temperature);
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
		
		/***********角加速度滤波（方法二选一）***********/
	
		//一、角加速度IIR滤波
//		mpu6050.Accel_Average[0] = IIR_I_Filter(mpu6050.Accel_Calulate[0], InPut_IIR[0], OutPut_IIR[0], b_IIR, IIR_ORDER+1, a_IIR, IIR_ORDER+1);
//		mpu6050.Accel_Average[1] = IIR_I_Filter(mpu6050.Accel_Calulate[1], InPut_IIR[1], OutPut_IIR[1], b_IIR, IIR_ORDER+1, a_IIR, IIR_ORDER+1);
//		mpu6050.Accel_Average[2] = IIR_I_Filter(mpu6050.Accel_Calulate[2], InPut_IIR[2], OutPut_IIR[2], b_IIR, IIR_ORDER+1, a_IIR, IIR_ORDER+1);
		//二、角加速度卡尔曼滤波方法：
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
	
	//一、角加速度和角速度解算的 俯仰角 和 横滚角 进行结合
	float pitch1	= 	RtA*atan(ay/sqrtf(ax*ax+az*az));  // 俯仰角
	float roll1		=	-RtA*atan(ax/sqrtf(ay*ay+az*az)); // 横滚角
	pitch2  += (mpu6050.Gyro_Average[0])*2000/32768*Sampling_Time;//俯仰角
	roll2	+= (mpu6050.Gyro_Average[1])*2000/32768*Sampling_Time;//横滚角
	mpu6050.Pitch =	 pitch1*MPU_Aceel_Gyro_Kp+pitch2*(1-MPU_Aceel_Gyro_Kp);		// 俯仰角
	mpu6050.Roll  =  roll1*MPU_Aceel_Gyro_Kp+roll2*(1-MPU_Aceel_Gyro_Kp);	 		// 横滚角
	//二、角加速度解算的 俯仰角 和 横滚角
//	mpu6050.Pitch =	RtA*atan(ay/sqrtf(ax*ax+az*az)); // 俯仰角
//	mpu6050.Roll = -RtA*atan(ax/sqrtf(ay*ay+az*az)); // 横滚角
	//三、角速度解算的 俯仰角 和 横滚角
//	mpu6050.Pitch 	+= (mpu6050.Gyro_Average[0])*2000/32768*Sampling_Time; // 俯仰角
//	mpu6050.Roll 	+= (mpu6050.Gyro_Average[1])*2000/32768*Sampling_Time; // 横滚角

	//z轴不需要更改，足够稳定了
	Gyro_Z_Measeure = (mpu6050.Gyro_Calulate[2])*2000/32768.0;
	Yaw += Gyro_Z_Measeure*Sampling_Time;
	mpu6050.Yaw  = 	Yaw + Yaw*0.16667;//后面这个0.16667是为了补偿角度
	
}

double IIR_I_Filter(double InData, double *x, double *y, double *b, short nb, double *a, short na){
  double z1,z2;
  short i;
  double OutData;
  
  for(i=nb-1; i>0; i--)
  {
    x[i]=x[i-1];
  }
  
  x[0] = InData;
  
  for(z1=0,i=0; i<nb; i++)
  {
    z1 += x[i]*b[i];
  }
  
  for(i=na-1; i>0; i--)
  {
    y[i]=y[i-1];
  }
  
  for(z2=0,i=1; i<na; i++)
  {
    z2 += y[i]*a[i];
  }
  
  y[0] = z1 - z2; 
  OutData = y[0];
    
  return OutData;
}

float LPF_1st(float oldData, float newData, float lpf_factor){
	return oldData * (1 - lpf_factor) + newData * lpf_factor;    //上个数据*一定的比例+现在的数据*一定的比例
}
//一维卡尔曼滤波
void kalmanfiter(struct KalmanFilter *EKF,float input){
	EKF->NewP = EKF->LastP + EKF->Q;
	EKF->Kg = EKF->NewP / (EKF->NewP + EKF->R);
	EKF->Out = EKF->Out + EKF->Kg * (input - EKF->Out);
	EKF->LastP = (1 - EKF->Kg) * EKF->NewP;
}
