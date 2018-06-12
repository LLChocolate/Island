#include "mpu6050.h"

//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK精英STM32开发板V3
//MPU6050 驱动代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//创建日期:2015/1/17
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2009-2019
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 
 
//初始化MPU6050
//返回值:0,成功
//    其他,错误代码


s16 mpu6050_Acc_x,mpu6050_Acc_y,mpu6050_Acc_z;
s16 mpu6050_Gyro_x,mpu6050_Gyro_y,mpu6050_Gyro_z;
//short mpu6050_Tempreature;

float mpu6050_Acc_x_ave,mpu6050_Acc_y_ave,mpu6050_Acc_z_ave;
float mpu6050_Gyro_x_ave,mpu6050_Gyro_y_ave,mpu6050_Gyro_z_ave;
//float mpu6050_Tempreature_ave;

float mpu6050_Acc_x_ms2,mpu6050_Acc_y_ms2,mpu6050_Acc_z_ms2;//×a?a?×???t′?·???
float mpu6050_Gyro_x_rs,mpu6050_Gyro_y_rs,mpu6050_Gyro_z_rs;//×a?a???è????


Kalman_Date Acc_x={0,0,0,0,0,1,400,0,0};
Kalman_Date Acc_y={0,0,0,0,0,1,400,0,0};
Kalman_Date Acc_z={0,0,0,0,0,1,400,0,0};
Kalman_Date Gyro_x_Str={0,0,0,0,0,0.03,4,0,0};
Kalman_Date Gyro_y_Str={0,0,0,0,0,0.03,4,0,0};
Kalman_Date Gyro_z_Str={0,0,0,0,0,0.03,4,0,0};

//Filter_1st_Str Acc_x = {M_filter_k,{0,0},{0,0}};
//Filter_1st_Str Acc_y = {M_filter_k,{0,0},{0,0}};
//Filter_1st_Str Acc_z = {M_filter_k,{0,0},{0,0}};
//Filter_1st_Str Gyro_x_Str = {M_filter_k,{0,0},{0,0}};
//Filter_1st_Str Gyro_y_Str = {M_filter_k,{0,0},{0,0}};
//Filter_1st_Str Gyro_z_Str = {M_filter_k,{0,0},{0,0}};

u8 mpu6050_buffer[14];
float Q4[4] = {1,0,0,0};
float Roll, Pitch, Yaw;

void MPU_Init(u16 lpf)
{ 
  u16 res;
  u16 default_filter = 1;	
  MPU_IIC_Init();
  switch(lpf)
  {
  case 5:
    default_filter = MPU6050_DLPF_BW_5;
    break;
  case 10:
    default_filter = MPU6050_DLPF_BW_10;
    break;
  case 20:
    default_filter = MPU6050_DLPF_BW_20;
    break;
  case 42:
    default_filter = MPU6050_DLPF_BW_42;
    break;
  case 98:
    default_filter = MPU6050_DLPF_BW_98;
    break;
  case 188:
    default_filter = MPU6050_DLPF_BW_188;
    break;
  case 256:
    default_filter = MPU6050_DLPF_BW_256;
    break;
  default:
    default_filter = MPU6050_DLPF_BW_42;
    break;
  }
  delayms(10);
  res=MPU_Read_Byte(MPU_DEVICE_ID_REG);
  delayms(10);
  MPU6050_setSleepEnabled(0); //关闭睡眠模式
  delayms(10);
  MPU6050_setClockSource(MPU6050_CLOCK_PLL_ZGYRO); //把Z轴作为时钟源
  delayms(10);
  MPU6050_set_SMPLRT_DIV(1000);  //200hz
  delayms(10);
  MPU6050_setFullScaleGyroRange(MPU6050_GYRO_FS_2000);//??????? +-2000???
  delayms(10);
  MPU6050_setFullScaleAccelRange(MPU6050_ACCEL_FS_2);	//???????? +-2G
  delayms(10);
  MPU6050_setDLPF(default_filter);  //42hz
  delayms(10);
//  MPU6050_setI2CMasterModeEnabled(0);	 //??MPU6050 ??AUXI2C
//  delayms(10);
//  MPU6050_setI2CBypassEnabled(1);	 //?????I2C?	MPU6050?AUXI2C	????????????HMC5883L
//  delayms(10);
        
}
//设置MPU6050陀螺仪传感器满量程范围
//fsr:0,±250dps;1,±500dps;2,±1000dps;3,±2000dps
//返回值:0,设置成功
//    其他,设置失败 

void IICwriteBit(u8 reg, u8 bitNum, u8 data)
{
  u8 b;
  MPU_Read_Len(reg, 1, &b);
  b = (data != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));
  MPU_Write_Byte(reg, b);
}

void IICwriteBits(u8 reg,u8 bitStart,u8 length,u8 data)
{	
  u8 b,mask;
  MPU_Read_Len(reg, 1, &b);
  mask = (0xFF << (bitStart + 1)) | 0xFF >> ((8 - bitStart) + length - 1);
  data <<= (8 - length);
  data >>= (7 - bitStart);
  b &= mask;
  b |= data;
  MPU_Write_Byte(reg, b);
}

void MPU6050_setSleepEnabled(uint8_t enabled) 
{
  IICwriteBit(MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, enabled);
}

void MPU6050_setClockSource(uint8_t source)
{
  IICwriteBits(MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_CLKSEL_BIT, MPU6050_PWR1_CLKSEL_LENGTH, source);
}

void MPU6050_set_SMPLRT_DIV(uint16_t hz)
{
  MPU_Write_Byte(MPU6050_RA_SMPLRT_DIV,1000/hz - 1);
}

void MPU6050_setFullScaleGyroRange(uint8_t range) 
{
  IICwriteBits(MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_FS_SEL_BIT, MPU6050_GCONFIG_FS_SEL_LENGTH, range);
  IICwriteBits(MPU6050_RA_GYRO_CONFIG,7, 3, 0x00);   //不自检
}

void MPU6050_setFullScaleAccelRange(uint8_t range) 
{
  IICwriteBits(MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_AFS_SEL_BIT, MPU6050_ACONFIG_AFS_SEL_LENGTH, range);
  IICwriteBits(MPU6050_RA_ACCEL_CONFIG,7, 3, 0x00);   //不自检
}

void MPU6050_setDLPF(uint8_t mode)
{
  IICwriteBits(MPU6050_RA_CONFIG, MPU6050_CFG_DLPF_CFG_BIT, MPU6050_CFG_DLPF_CFG_LENGTH, mode);
}

void MPU6050_setI2CMasterModeEnabled(uint8_t enabled) 
{
  IICwriteBit(MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_I2C_MST_EN_BIT, enabled);
}

void MPU6050_setI2CBypassEnabled(uint8_t enabled) 
{
  IICwriteBit(MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_I2C_BYPASS_EN_BIT, enabled);
}

u8 MPU_Set_Gyro_Fsr(u8 fsr)
{
	return MPU_Write_Byte(MPU_GYRO_CFG_REG,fsr<<3);//设置陀螺仪满量程范围  
}
//设置MPU6050加速度传感器满量程范围
//fsr:0,±2g;1,±4g;2,±8g;3,±16g
//返回值:0,设置成功
//    其他,设置失败 
u8 MPU_Set_Accel_Fsr(u8 fsr)
{
	return MPU_Write_Byte(MPU_ACCEL_CFG_REG,fsr<<3);//设置加速度传感器满量程范围  
}
//设置MPU6050的数字低通滤波器
//lpf:数字低通滤波频率(Hz)
//返回值:0,设置成功
//    其他,设置失败 
u8 MPU_Set_LPF(u16 lpf)
{
	u8 data=0;
	if(lpf>=188)data=1;
	else if(lpf>=98)data=2;
	else if(lpf>=42)data=3;
	else if(lpf>=20)data=4;
	else if(lpf>=10)data=5;
	else data=6; 
	return MPU_Write_Byte(MPU_CFG_REG,data);//设置数字低通滤波器  
}
//设置MPU6050的采样率(假定Fs=1KHz)
//rate:4~1000(Hz)
//返回值:0,设置成功
//    其他,设置失败 
u8 MPU_Set_Rate(u16 rate)
{
	u8 data;
	if(rate>1000)rate=1000;
	if(rate<4)rate=4;
	data=1000/rate-1;
	data=MPU_Write_Byte(MPU_SAMPLE_RATE_REG,data);	//设置数字低通滤波器
 	return MPU_Set_LPF(rate/2);	//自动设置LPF为采样率的一半
}

//得到温度值
//返回值:温度值(扩大了100倍)
short MPU_Get_Temperature(void)
{
    u8 buf[2]; 
    short raw;
	float temp;
	MPU_Read_Len(MPU_TEMP_OUTH_REG,2,buf); 
    raw=((u16)buf[0]<<8)|buf[1];  
    temp=36.53+((double)raw)/340;  
    return temp*100;;
}
//得到陀螺仪值(原始值)
//gx,gy,gz:陀螺仪x,y,z轴的原始读数(带符号)
//返回值:0,成功
//    其他,错误代码
u8 MPU_Get_Gyroscope(s16 *gx,s16 *gy,s16 *gz)
{
    u8 buf[6],res;  
	res=MPU_Read_Len(MPU_GYRO_XOUTH_REG,6,buf);
	if(res==0)
	{
              *gx=((u16)buf[0]<<8)|buf[1];  
              *gy=((u16)buf[2]<<8)|buf[3];  
              *gz=((u16)buf[4]<<8)|buf[5];
	} 	
    return res;;
}
//得到加速度值(原始值)
//gx,gy,gz:陀螺仪x,y,z轴的原始读数(带符号)
//返回值:0,成功
//    其他,错误代码
u8 MPU_Get_Accelerometer(s16 *ax,s16 *ay,s16 *az)
{
    u8 buf[6],res;  
	res=MPU_Read_Len(MPU_ACCEL_XOUTH_REG,6,buf);
	if(res==0)
	{
		*ax=((u16)buf[0]<<8)|buf[1];  
		*ay=((u16)buf[2]<<8)|buf[3];  
		*az=((u16)buf[4]<<8)|buf[5];
	} 	
    return res;;
}
//IIC连续写
//addr:器件地址 
//reg:寄存器地址
//len:写入长度
//buf:数据区
//返回值:0,正常
//    其他,错误代码
u8 MPU_Write_Len(u8 reg,u8 len,u8 *buf)
{
	u8 i; 
    MPU_IIC_Start(); 
	MPU_IIC_Send_Byte((MPU6050_ADDR<<1)|0);//发送器件地址+写命令	
	if(MPU_IIC_Wait_Ack())	//等待应答
	{
		MPU_IIC_Stop();		 
		return 1;		
	}
    MPU_IIC_Send_Byte(reg);	//写寄存器地址
    MPU_IIC_Wait_Ack();		//等待应答
	for(i=0;i<len;i++)
	{
		MPU_IIC_Send_Byte(buf[i]);	//发送数据
		if(MPU_IIC_Wait_Ack())		//等待ACK
		{
			MPU_IIC_Stop();	 
			return 1;		 
		}		
	}    
    MPU_IIC_Stop();	 
	return 0;	
} 
//IIC连续读
//addr:器件地址
//reg:要读取的寄存器地址
//len:要读取的长度
//buf:读取到的数据存储区
//返回值:0,正常
//    其他,错误代码
u8 MPU_Read_Len(u8 reg,u8 len,u8 *buf)
{
  u8 i;
  for (i=0; i<len; i++)
  {
    buf[i] = MPU_Read_Byte(reg+i);
  }
//    MPU_IIC_Start(); 
//    MPU_IIC_Send_Byte((MPU6050_ADDR<<1)|0);//发送器件地址+写命令	
//    if(MPU_IIC_Wait_Ack())	//等待应答
//    {
//      MPU_IIC_Stop();		 
//      return 1;		
//    }
//    MPU_IIC_Send_Byte(reg);	//写寄存器地址
//    MPU_IIC_Wait_Ack();		//等待应答
//    MPU_IIC_Start();
//      MPU_IIC_Send_Byte((MPU6050_ADDR<<1)|1);//发送器件地址+读命令	
//    MPU_IIC_Wait_Ack();		//等待应答 
//    while(len)
//    {
//      if(len==1)*buf=MPU_IIC_Read_Byte(0);//读数据,发送nACK 
//      else *buf=MPU_IIC_Read_Byte(1);		//读数据,发送ACK  
//      len--;
//      buf++; 
//      delayus(1);
//    }    
//    MPU_IIC_Stop();	//产生一个停止条件 
    return 0;	
}
//IIC写一个字节 
//reg:寄存器地址
//data:数据
//返回值:0,正常
//    其他,错误代码
u8 MPU_Write_Byte(u8 reg,u8 data) 				 
{
    MPU_IIC_Start(); 
	MPU_IIC_Send_Byte((MPU_ADDR<<1)|0);//发送器件地址+写命令	
	if(MPU_IIC_Wait_Ack())	//等待应答
	{
		MPU_IIC_Stop();		 
		return 1;		
	}
    MPU_IIC_Send_Byte(reg);	//写寄存器地址
    MPU_IIC_Wait_Ack();		//等待应答 
	MPU_IIC_Send_Byte(data);//发送数据
	if(MPU_IIC_Wait_Ack())	//等待ACK
	{
		MPU_IIC_Stop();	 
		return 1;		 
	}		 
    MPU_IIC_Stop();	 
	return 0;
}
//IIC读一个字节 
//reg:寄存器地址 
//返回值:读到的数据
u8 MPU_Read_Byte(u8 reg)
{
	u8 res;
    MPU_IIC_Start(); 
	MPU_IIC_Send_Byte((MPU_ADDR<<1)|0);//发送器件地址+写命令	
	MPU_IIC_Wait_Ack();		//等待应答 
    MPU_IIC_Send_Byte(reg);	//写寄存器地址
    MPU_IIC_Wait_Ack();		//等待应答
    MPU_IIC_Start();
	MPU_IIC_Send_Byte((MPU_ADDR<<1)|1);//发送器件地址+读命令	
    MPU_IIC_Wait_Ack();		//等待应答 
	res=MPU_IIC_Read_Byte(0);//读取数据,发送nACK 
    MPU_IIC_Stop();			//产生一个停止条件 
	return res;		
}

void MPU6050_Read(void)
{ 
//  I2C_FastMode = 1;
  MPU_Read_Len(MPU_ACCEL_XOUTH_REG,14,mpu6050_buffer);
}

void MPU6050_Data_Prepare(void)
{
// // int i;
//  int mpu_count;
//	
//  float Gyro_tmp[3];
	/*------------------------------------MPU6050?áè?------------------------------------*/
	
  MPU6050_Read();
  //
	/*??buffer????*/
//  mpu6050_Acc_x = ((((int16_t)mpu6050_buffer[0]) << 8) | mpu6050_buffer[1]) ;
//  mpu6050_Acc_y = ((((int16_t)mpu6050_buffer[2]) << 8) | mpu6050_buffer[3]) ;
//  mpu6050_Acc_z = ((((int16_t)mpu6050_buffer[4]) << 8) | mpu6050_buffer[5]) ;

  
  mpu6050_Gyro_x = ((((int16_t)mpu6050_buffer[ 8]) << 8) | mpu6050_buffer[ 9]) ;
//  mpu6050_Gyro_y = ((((int16_t)mpu6050_buffer[10]) << 8) | mpu6050_buffer[11]) ;
//  mpu6050_Gyro_z = ((((int16_t)mpu6050_buffer[12]) << 8) | mpu6050_buffer[13]) ;
//  MPU_Get_Accelerometer(&mpu6050_Acc_x,&mpu6050_Acc_y,&mpu6050_Acc_z);
//  MPU_Get_Gyroscope(&mpu6050_Gyro_x,&mpu6050_Gyro_y,&mpu6050_Gyro_z);
//  Gyro_tmp[0] = mpu6050_Gyro_x ;//
//  Gyro_tmp[1] = mpu6050_Gyro_y ;//
//  Gyro_tmp[2] = mpu6050_Gyro_z ;//

//  mpu6050_Tempreature = ((((int16_t)mpu6050_buffer[6]) << 8) | mpu6050_buffer[7]); //tempreature

	/*------------------------------------------------------------------------------------*/
	
  mpu6050_Gyro_x=mpu6050_Gyro_x+30;
//  mpu6050_Gyro_y=mpu6050_Gyro_y+2;
//  mpu6050_Gyro_z=mpu6050_Gyro_z+2;//调零

  mpu6050_Gyro_x_ave=KalmanFilter(mpu6050_Gyro_x,&Gyro_x_Str);
//  mpu6050_Gyro_y_ave=KalmanFilter(mpu6050_Gyro_y,&Gyro_y_Str);
//  mpu6050_Gyro_z_ave=KalmanFilter(mpu6050_Gyro_z,&Gyro_z_Str);
//
//  mpu6050_Acc_x_ave=KalmanFilter(mpu6050_Acc_x,&Acc_x);   //0.02 3
//  mpu6050_Acc_y_ave=KalmanFilter(mpu6050_Acc_y,&Acc_y);
//  mpu6050_Acc_z_ave=KalmanFilter(mpu6050_Acc_z,&Acc_z);
//          
//  mpu6050_Acc_x_ms2=(float)(mpu6050_Acc_x_ave*5.98211669921875e-4);      //5.98211669921875e-4     1/16384*9.8011
//  mpu6050_Acc_y_ms2=(float)(mpu6050_Acc_y_ave*5.98211669921875e-4);
//  mpu6050_Acc_z_ms2=(float)(mpu6050_Acc_z_ave*5.98211669921875e-4);//转化为米每二次方秒
//  if(mpu6050_Acc_z_ms2>12)mpu6050_Acc_z_ms2 = 12;
////  else if()
//
//  mpu6050_Gyro_x_rs=(float)((mpu6050_Gyro_x_ave)*0.001065264436031695);         //0.001065264436031695     4000/65536/180*3.14            
//  mpu6050_Gyro_y_rs=(float)((mpu6050_Gyro_y_ave)*0.001065264436031695);
//  mpu6050_Gyro_z_rs=(float)((mpu6050_Gyro_z_ave)*0.001065264436031695);//转化为弧度每秒

//  imuUpdate(mpu6050_Gyro_x_rs,mpu6050_Gyro_y_rs,mpu6050_Gyro_z_rs,mpu6050_Acc_x_ms2,mpu6050_Acc_y_ms2,mpu6050_Acc_z_ms2,0.005,&Roll,&Pitch,&Yaw);


}
//void mix_gyrAcc_crossMethod(float * attitude,short gyr[3],short acc[3])
//{
//const static float FACTOR = 0.001;//两个重力矢量叉积后所乘的系数p，用于和陀螺仪积分角度相叠加来修正陀螺仪（这里只用了比例p，没用积分i，）
////FACTOR 为1，则完全信任加速度计，为0，则完全信任陀螺仪
//float w_q = attitude[0];//w=cos(alpha/2)
//float x_q = attitude[1];//x=ax*sin(alpha/2)
//float y_q = attitude[2];//y=ay*sin(alpha/2)
//float z_q = attitude[3];//z=az*sin(alpha/2)
//float x_q_2 = x_q * 2;
//float y_q_2 = y_q * 2;
//float z_q_2 = z_q * 2;
////
//// 加速度计的读数，单位化。
//float norm = sqrt((acc[0]*acc[0]+acc[1]*acc[1]+acc[2]*acc[2])*1.0);
//float x_aa = acc[0]*1.0 * norm;
//float y_aa = acc[1]*1.0 * norm;
//float z_aa = acc[2]*1.0 * norm;
////
//// 载体坐标下的重力加速度常量，单位化。//用旋转矩阵将世界坐标系的单位化重力矢量(0,0,1）不是（0,0，-1）,mpu6050 只感应非重力加速度)转换到机载坐标系中。
////机载坐标下的重力矢量旋转矩阵（坐标系转换矩阵的逆矩阵也就是转置矩阵，因为欧拉角解得的旋转矩阵必是正交阵） 世界坐标下的重力矢量
//// x cos(T)cos(K) cos(T)sin(C) -sin(C) 0
//// [ y ] = [ sin(F)sin(T)cos(K)-cos(F)sin(K) sin(F)sin(T)sin(K)+cos(F)cos(K) sin(F)cos(T) ] * [ 0 ]
//// z cos(F)sin(T)cos(K)+sin(F)sin(K) cos(F)sin(T)sin(K)-sin(F)cos(K) cos(F)cos(T) 1
////K 是yaw,T 是pitch,F 是roll,旋转顺序为ZYX
//// w^2+x^2-y^2-z^2 2*(x*y+w*z) 2*(x*z-w*y)
////上式中的旋转矩阵用四元数表示即为：[ 2*(x*y-w*z) w^2-x^2+y^2-z^2 2*(y*z+w*x) ]
//// 2*(x*z+w*y) 2*（y*z-w*x） w^2-x^2-y^2+z^2
////
//float x_ac = x_q*z_q_2 - w_q*y_q_2;// 2*(x*z-w*y) =ax*az(1-cos(alpha))-ay*sin(alpha)
//float y_ac = y_q*z_q_2 + w_q*x_q_2;// 2*(y*z+w*x) =az*ay(1-cos(alpha))+ax*sin(alpha)
//float z_ac = 1 - x_q*x_q_2 - y_q*y_q_2;// w^2+x^2-y^2-z^2 =1-2*x^2-2*y^2 = cos(alpha)+(1-cos(alpha)*z^2)
////
//// 测量值与常量的叉积。//测量值叉乘常量值,并以此向量表示误差角度大小与转轴方向，用于修正陀螺仪积分角度
//float x_ca = y_aa * z_ac - z_aa * y_ac;
//float y_ca = z_aa * x_ac - x_aa * z_ac;
//float z_ca = x_aa * y_ac - y_aa * x_ac;
////
//// 构造增量旋转。//可看成分别绕xyz 轴的三次旋转的叠加。sin(delta/2)近似为delta/2,cos(delta/2)近似为0
//float delta_x = gyr[0]*1.0 * halfT + x_ca * FACTOR;//绕x 轴旋转角度的一半，记d_x 看作绕x 轴的一次旋转：w=1,x=d_x,y=0,z=0
//float delta_y = gyr[1]*1.0 * halfT + y_ca * FACTOR;//绕y 轴旋转角度的一半，记d_y 看作绕y 轴的一次旋转：w=1,x=0,y=d_y,z=0
//float delta_z = gyr[2]*1.0 * halfT + z_ca * FACTOR;//绕z 轴旋转角度的一半，记d_z 看作绕z 轴的一次旋转：w=1,x=0,y=0,z=d_z
////三次旋转叠加为一次旋转，即三个四元数相乘
////四元数乘法公式：q3=q1*q2
////(w1*w2 - x1*x2 - y1*y2 - z1*z2) = w3
////(w1*x2 + x1*w2 + y1*z2 - z1*y2) = x3
////(w1*y2 - x1*z2 + y1*w2 + z1*x2) = y3
////(w1*z2 + x1*y2 - y1*x2 + z1*w2) = z3
////合成的一次旋转：
//// w=1 - d_x*d_y*d_z(多个小角度相乘，忽略，下同) =1
//// x=d_x + d_y*d_z（忽略） =d_x
//// y=d_y - d_x*d_z（忽略） =d_y
//// z=d_z + d_x*d_y（忽略） =d_z
////
//// 融合，四元数乘法。//将上面合成的旋转四元数与之前的姿态四元数相乘，得到新的姿态四元数并归一化为单位四元数。
//attitude[0] = w_q - x_q*delta_x - y_q*delta_y - z_q*delta_z;
//attitude[1] = w_q*delta_x + x_q + y_q*delta_z - z_q*delta_y;
//attitude[2] = w_q*delta_y - x_q*delta_z + y_q + z_q*delta_x;
//attitude[3] = w_q*delta_z + x_q*delta_y - y_q*delta_x + z_q;
//norm=sqrt(attitude[0]*attitude[0]+attitude[1]*attitude[1]+attitude[2]*attitude[2]+attitude[3]*attitude[3]);
//attitude[0]=norm*attitude[0];
//attitude[1]=norm*attitude[1];
//attitude[2]=norm*attitude[2];
//attitude[3]=norm*attitude[3];
//		Pitch = asin(-2 * attitude[1] * attitude[3] + 2 * attitude[0]* attitude[2])* 57.3;	// pitch
//		Roll  = atan2(2 * attitude[2] * attitude[3] + 2 * attitude[0] * attitude[1], -2 * attitude[1] * attitude[1] - 2 * attitude[2]* attitude[2] + 1)* 57.3;	// roll
//		Yaw   = atan2(2*(attitude[1]*attitude[2] + attitude[0]*attitude[3]),attitude[0]*attitude[0]+attitude[1]*attitude[1]-attitude[2]*attitude[2]-attitude[3]*attitude[3]) * 57.3;	//yaw
////四元数归一化
//}