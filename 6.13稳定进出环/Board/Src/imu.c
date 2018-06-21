#include "imu.h"
#include "math.h"
#include "mymath.h"


#define  YAW_ERROR 0




float Kp =1.2f ;		/*比例增益*///0.4//200.0f
float Ki = 0.01f;		/*积分增益*///0.001f//1.0f
float exInt = 0.0f;
float eyInt = 0.0f;
float ezInt = 0.0f;		/*积分误差累计*/

static float q0 = 1.0f;	/*四元数*/
static float q1 = 0.0f;
static float q2 = 0.0f;
static float q3 = 0.0f;	
static float vecxZ, vecyZ, veczZ;	/*机体坐标系下的Z方向向量*/

//static float baseZacc = 1.0;		/*静态Z轴加速度*/


#define DEG2RAD		0.017453293f	/* 度转弧度 π/180 */
#define RAD2DEG		57.29578f		/* 弧度转度 180/π */




//static float invSqrt(float x);	/*快速开平方求倒*/

void imuUpdate(float gx, float gy, float gz, float ax, float ay, float az, float dt,float *roll,float *pitch,float *yaw)	/*数据融合 互补滤波*///gx gy gx单位为弧度每秒 ax ay az为米每二次方秒 dt为周期单位ms
{
	float normalise;
	float ex, ey, ez;
	float q0s, q1s, q2s, q3s;	/*四元数的平方*/
	static float R11,R21;		/*矩阵(1,1),(2,1)项*/

	float halfT =0.5f * dt;
	


	/* 某一个方向加速度不为0 */
	if((ax != 0.0f) || (ay != 0.0f) || (az != 0.0f))
	{
          
		/*单位化加速计测量值*/
		normalise = myInvSqrt(ax * ax + ay * ay + az * az);
		ax *= normalise;
		ay *= normalise;
		az *= normalise;

		/*加速计读取的方向与重力加速计方向的差值，用向量叉乘计算*/
		ex = (ay * veczZ - az * vecyZ);
		ey = (az * vecxZ - ax * veczZ);
		ez = (ax * vecyZ - ay * vecxZ);
		
		/*误差累计，与积分常数相乘*/
		exInt += Ki * ex * dt ;  
		eyInt += Ki * ey * dt ;
		ezInt += Ki * ez * dt ;
		
		/*用叉积误差来做PI修正陀螺零偏，即抵消陀螺读数中的偏移量*/
		gx += Kp * ex + exInt;
		gy += Kp * ey + eyInt;
		gz += Kp * ez + ezInt;
	}
	/* 一阶近似算法，四元数运动学方程的离散化形式和积分 */
	q0 += (-q1 * gx - q2 * gy - q3 * gz) * halfT;
	q1 += (q0 * gx + q2 * gz - q3 * gy) * halfT;
	q2 += (q0 * gy - q1 * gz + q3 * gx) * halfT;
	q3 += (q0 * gz + q1 * gy - q2 * gx) * halfT;
	
	/*单位化四元数*/
	normalise = myInvSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= normalise;
	q1 *= normalise;
	q2 *= normalise;
	q3 *= normalise;
	/*四元数的平方*/
	q0s = q0 * q0;
	q1s = q1 * q1;
	q2s = q2 * q2;
	q3s = q3 * q3;
	
	R11 = q0s + q1s - q2s - q3s;	/*矩阵(1,1)项*/
	R21 = 2 * (q1 * q2 + q0 * q3);	/*矩阵(2,1)项*/

	/*机体坐标系下的Z方向向量*/
	vecxZ = 2 * (q1 * q3 - q0 * q2);/*矩阵(3,1)项*/
	vecyZ = 2 * (q0 * q1 + q2 * q3);/*矩阵(3,2)项*/
	veczZ = q0s - q1s - q2s + q3s;	/*矩阵(3,3)项*/
	
	if (vecxZ>1) vecxZ=1;
	if (vecxZ<-1) vecxZ=-1;
	
	/*计算roll pitch yaw 欧拉角*/
	*pitch = -asinf(vecxZ) * RAD2DEG;//+PITCH_ERROR; 
	*roll = atan2f(vecyZ, veczZ) * RAD2DEG;//+ROLL_ERROR;
	*yaw = atan2f(R21, R11) * RAD2DEG+YAW_ERROR;
	
	//test_yaw+=	((Gyro_z-GyrooffsetX)*MPU6050G_s2000dps*0.0174)*RAD2DEG*dt;
	

  // az= ax*vecxZ + ay * vecyZ + az * veczZ - baseZacc;	/*Z轴加速度(去除重力加速度)*/
	
	
}





//	Body_To_Earth(&earth_acc_x,&earth_acc_y,&earth_acc_z,mpu6050_Acc_x_ms2,mpu6050_Acc_y_ms2,mpu6050_Acc_z_ms2,test_yaw,test_pitch,test_roll);yaw和pitch反用
void Body_To_Earth(float *Earth_x,float *Earth_y,float *Earth_z,float Body_x,float Body_y,float Body_z,float Yaw,float Roll,float Pitch)
{
	float COS_PSI;
	float COS_Theta;
	float COS_Phi;
	float SIN_PSI;	
	float SIN_Theta;
	float SIN_Phi;

	Yaw  *= DEG_TO_RAD;
	Roll *= DEG_TO_RAD;
	Pitch*= DEG_TO_RAD;
	
	COS_PSI   = cos(Yaw);
	COS_Theta = cos(Roll);
	COS_Phi   = cos(Pitch);
	SIN_PSI   = sin(Yaw);	
	SIN_Theta = sin(Roll);
	SIN_Phi	  = sin(Pitch);
	
	*Earth_x =  COS_PSI * COS_Theta * Body_x + (-COS_Phi * SIN_PSI + SIN_Phi * SIN_Theta * COS_PSI) * Body_y + ( SIN_PSI * SIN_Phi + COS_Phi * SIN_Theta * COS_PSI) * Body_z;
	*Earth_y =  COS_Theta * SIN_PSI * Body_x + ( COS_Phi * COS_PSI + SIN_Phi * SIN_Theta * SIN_PSI) * Body_y + (-COS_PSI * SIN_Phi + COS_Phi * SIN_Theta * SIN_PSI) * Body_z;
	*Earth_z = -SIN_Theta 					 * Body_x + 																 COS_Theta * SIN_Phi * Body_y + 																 COS_Theta * COS_Phi * Body_z;

};

float gravity=9.7;
float speed_change;
void acc_measure_upspeed(void)
{

//speed_change=(earth_acc_z-gravity)*0.002*1000;
//if((speed_change<3)&&(speed_change>-3))
//	speed_change=0;
//	
//up_speed=up_speed+speed_change;



}




///****气压计三阶互补滤波方案——参考开源飞控APM****/
//#define TIME_CONTANST_ZER 2.5f
//#define K_ACC_ZER 	    (1.0f / (TIME_CONTANST_ZER * TIME_CONTANST_ZER * TIME_CONTANST_ZER))
//#define K_VEL_ZER	        (3.0f / (TIME_CONTANST_ZER * TIME_CONTANST_ZER))														
//#define K_POS_ZER         (2.0f / TIME_CONTANST_ZER)
//#define Delay_Cnt 60
//void Altitude_Update(u16 Time_Ms,float ACC_Earth_Z,float ultr_get_cm)
//{
//	static float Origion_Acc_z;
//	static float acc_correction = 0;
//	static float vel_correction = 0;
//	static float pos_correction = 0;
//	static float Last_Acc_z = 0;
//	static float Origion_Speed_z = 0;
//	static float Origion_Position_z = 0;
//	static float SpeedDealt = 0;
//	static float Altitude_Dealt=0;
//	static float Altitude_Estimate=0;
//	static float Altitude_History[Delay_Cnt+1];
//	static float Delta_T;
//	u16 Cnt=0;
//	
//	static uint16_t Save_Cnt=0;
//	Save_Cnt++;//数据存储周期
//	
//	Delta_T = Time_Ms/1000.0;//ms转s 单位 s
//	Altitude_Estimate = ultr_get_cm;//高度观测量 m转cm 单位cm
//	Origion_Acc_z = ACC_Earth_Z;//加速度  单位cm/s2
//	
//	//由观测量（气压计）得到状态误差
//	Altitude_Dealt = Altitude_Estimate - hight_now;//气压计(超声波)与SINS估计量的差，单位cm
//	//三路积分反馈量修正惯导
//	acc_correction +=Altitude_Dealt* K_ACC_ZER*Delta_T ;//加速度矫正量
//	vel_correction +=Altitude_Dealt* K_VEL_ZER*Delta_T ;//速度矫正量
//	pos_correction +=Altitude_Dealt* K_POS_ZER*Delta_T ;//位置矫正量
//	//加速度计矫正后更新
//	Last_Acc_z = up_acc;//上一次加速度量
//	up_acc = Origion_Acc_z + acc_correction;// 加速度单位cm/s2
//	//速度增量矫正后更新，用于更新位置,由于步长h=0.005,相对较长，
//	//这里采用二阶龙格库塔法更新微分方程，不建议用更高阶段，因为加速度信号非平滑
//	SpeedDealt = (Last_Acc_z + up_acc) * Delta_T/2.0;
//	//原始位置更新
//	Origion_Position_z += (up_speed + 0.5*SpeedDealt) * Delta_T;
//	//位置矫正后更新
//	hight_now = Origion_Position_z + pos_correction;    
//	//原始速度更新
//	Origion_Speed_z += SpeedDealt;
//	//速度矫正后更新
//	up_speed = Origion_Speed_z + vel_correction;

//	if(Save_Cnt>=1)
//	{
//		for(Cnt = Delay_Cnt;Cnt > 0;Cnt--)//滑动
//		{
//			Altitude_History[Cnt]=Altitude_History[Cnt-1];
//		}
//		Altitude_History[0] = hight_now;
//		Save_Cnt=0;
//	}
//}





