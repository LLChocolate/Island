#ifndef _DEFINE_H_
#define _DEFINE_H_

#include  "common.h"
/*************************************************************************
*  模块名称：define模块
*  功能说明：Include 用户自定义的宏
*  参数说明：无
*  函数返回：无
*  修改时间：2012-2-14
*************************************************************************/

//#define LCD_DISPLAY

//道路状态标志
#define Speed_filter_Period       5//均值滤波周期

typedef enum
{
  Nstart,//未开始状态
  Straight,//走直线
  Left_turn,//左转
  Right_turn,//右转
  Cross,//十字
  Uphill,//坡道
  Left_Island,//左环岛
  Right_Island//右环岛
  
}Road_Status;
 
typedef enum
{
  up   =1,
  left =2,
  right=4,
  down =8 
}Edge_Str_DIR;
typedef struct{
  int x;
  int y;
}Pixel;


typedef struct{
  int x;
  int y1;
  int y2;
}Edge_Str;

//电机舵机
#define MOTOR_1                         TPM0,TPM_CH0//电机接口
#define MOTOR_2                         TPM0,TPM_CH4//电机接口
//#define STEER_                          FTM1,CH0//舵机接口
#define MOTOR1_DIR                       PTA6_OUT//电机方向控制
#define MOTOR2_DIR                       PTA7_OUT//电机方向控制
typedef struct{
  u8 Dir;
  s16 Speed;
}Motor_Status;          //电机状态结构体




//ADC

#define MYADC_1                          ADC0_SE1
#define MYADC_2                          ADC0_DM1
#define MYADC_3                          ADC0_SE2
#define MYADC_4                          ADC0_DM2

//拨码
#define SW1              PTB9_IN
#define SW2              PTB10_IN
#define SW3              PTB11_IN
#define SW4              PTB16_IN
#define SW5              PTB17_IN
#define SW6              PTB18_IN
#define SW7              PTB19_IN
#define SW8              PTB20_IN

//按键
#define KEY1             PTB23_IN
#define KEY1_PRES        1
#define KEY2             PTB22_IN
#define KEY2_PRES        2
#define KEY3             PTC0_IN
#define KEY3_PRES        3   

//蜂鸣器
#define BEEP PTE21_OUT

//流水灯
#define LED1             PTC5_OUT
#define LED2             PTC6_OUT
#define LED3             PTC7_OUT
#define LED4             PTC8_OUT

//PID结构体
typedef struct                    //结构体，存放PID相关变量
{
  float P;                        //参数P
  float I;                        //参数I
  float D;                        //参数D
  float error[3];                 //误差存储数组
  float delat;                    //每次的调节结果
  float derr;                     //一阶误差
  float dderr;                    //二阶误差
  float result;                      //PID的输出，由于是增量式的所以初值请设为导轨平衡时的输出值，重要！不能是0要不就炸了
  
  float target;                   //PID调节的目标值     
  float feedback;
  float UP_Limit;
  float LOW_Limit;
}PID_Struct;

typedef struct 
{
  float x_mid;
  float x_now;
  float p_mid ;
  float p_now;
  float kg;
  float ProcessNoise_Q;
  float MeasureNoise_R;
  float x_last1;
  float p_last1;
}Kalman_Date;

typedef struct
{
  float m_filter;
  float ResrcData_mem[2];
  float output_mem[2];
}Filter_1st_Str;


//typedef struct
//{
//  float Kp;		/*比例增益*///0.4//200.0f
//  float Ki;		/*积分增益*///0.001f//1.0f
//  float exInt;
//  float eyInt;
//  float ezInt;		/*积分误差累计*/
//  float Q4[4];
//}


////系统状态结构体
//typedef struct
//{
//  long int Time_1ms;
//  
//}System_Status

#define     pit_delay_ms(PITn,ms)          pit_delay(PITn,ms * bus_clk_khz);        //PIT延时 ms
#define     pit_delay_us(PITn,us)          pit_delay(PITn,us * bus_clk_khz/1000);   //PIT延时 us

typedef enum{
  Unlock,
  Left_Lock,
  Right_Lock
}Black_View_Run_Status;


#define ALL_LINE            (240)
typedef struct
{
  int center[ALL_LINE];
  u8 halfwidth[ALL_LINE];
  int black_L[ALL_LINE];
  int black_R[ALL_LINE];
  u8 getLeft_flag[ALL_LINE];
  u8 getRight_flag[ALL_LINE];
  u8 hang_use;
}Image_hangData;


typedef struct
{
  u8 Three_Lie[3];
  u8 Three_lie_end[3];
}Image_lieData;


enum ISLAND_STATE
{
  NoIsland,
  Left_Island_pre,
  Right_Island_pre,
  Left_Island_in,
  Right_Island_in,
  Left_Island_out,
  Right_Island_out,
  Wait_Next
};


typedef struct
{
  enum ISLAND_STATE State;
  
  u16 black_L[10];
  u16 black_R[10];
  
  u8 Image_Start_hang;
  
  u8  Correct_hang;     //用来补线的行
  int In_Center;
  u8  In2Stay_cnt;
  
  int Stay_Center;//用作出环岛的标准
  
  u8  Stay2Out_flag;
  u16 Stay2Out_flag_delay;
  u16 Stay2Out_flag_delay_const;
  u8  Out_Allow_flag;
  
  u8  Next_Island_flag;         //和下一个环岛之间的时间间隔
  u16 Next_Island_flag_delay;
  u16 Next_Island_flag_delay_const;
}Island_Data;

typedef struct
{
  u8 Flag;
  u8 Delay;
  u8 Delay_const;
}Beep_Str;


#endif